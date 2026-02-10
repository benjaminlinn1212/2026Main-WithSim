package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ShooterSetpoint;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Turret subsystem that controls a rotating turret mechanism. Supports two states: STOW and AIMING.
 * Uses Team 254's wrapping logic for smooth continuous rotation.
 *
 * <p>Uses ShooterSetpoint utility for aim calculations, which handles smart target selection
 * including neutral zone detection and alliance-aware aiming.
 */
public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private double positionSetpointRad = 0.0;
  private double lastModeChange = 0.0;
  private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

  // ShooterSetpoint supplier for coordinated aiming
  private Supplier<ShooterSetpoint> setpointSupplier =
      () -> new ShooterSetpoint(0, 0, 0, 0, 0, false);

  public enum TurretState {
    STOW,
    AIMING
  }

  private TurretState currentState = TurretState.STOW;

  public TurretSubsystem(TurretIO io) {
    this.io = io;
  }

  /**
   * Set the robot pose supplier for field-relative angle calculation. Call this from RobotContainer
   * after constructing the drive subsystem.
   */
  public void setRobotPoseSupplier(Supplier<Pose2d> poseSupplier) {
    this.robotPoseSupplier = poseSupplier;
  }

  /**
   * Set the shooter setpoint supplier for coordinated aiming. Call this from RobotContainer to
   * connect all aiming subsystems through ShooterSetpoint.
   */
  public void setShooterSetpointSupplier(Supplier<ShooterSetpoint> supplier) {
    this.setpointSupplier = supplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    Logger.recordOutput("Turret/State", currentState.toString());
    Logger.recordOutput("Turret/SetpointRad", positionSetpointRad);

    // Calculate turret absolute field heading (robot heading + turret position)
    double turretPositionRad = Units.rotationsToRadians(inputs.positionRot);
    double robotHeadingRad = robotPoseSupplier.get().getRotation().getRadians();
    double turretFieldHeadingRad = robotHeadingRad + turretPositionRad;
    Logger.recordOutput("Turret/FieldHeadingRad", turretFieldHeadingRad);
    Logger.recordOutput("Turret/FieldHeadingDeg", Math.toDegrees(turretFieldHeadingRad));

    // Calculate and log position error
    double errorRad = positionSetpointRad - turretPositionRad;
    Logger.recordOutput("Turret/ErrorRad", errorRad);
    Logger.recordOutput("Turret/ErrorDeg", Math.toDegrees(errorRad));
  }

  /** Command to set turret to stow position (forward-facing). */
  public Command stow() {
    return runOnce(
            () -> {
              currentState = TurretState.STOW;
              updateModeChange();
            })
        .andThen(positionSetpointCommand(() -> Constants.TurretConstants.STOW_POSITION, () -> 0.0))
        .withName("Turret Stow");
  }

  /**
   * Command to aim turret using ShooterSetpoint calculations. ShooterSetpoint now handles smart
   * targeting logic including neutral zone detection and alliance-aware aiming.
   *
   * <p>Benefits: - Coordinated aiming with hood and shooter - Motion compensation with robot
   * velocity - Smart target selection (hub or alliance wall) - No duplicate calculation logic
   */
  public Command aiming() {
    return runOnce(
            () -> {
              currentState = TurretState.AIMING;
              updateModeChange();
            })
        .andThen(
            run(
                () -> {
                  // Get setpoint from ShooterSetpoint utility
                  ShooterSetpoint setpoint = setpointSupplier.get();

                  // Extract turret angle and feedforward from setpoint
                  double targetRad = setpoint.getTurretAngleRad();
                  double feedforwardRadPerSec = setpoint.getTurretFeedforwardRadPerSec();

                  // Apply wrapping logic to target
                  double wrappedTarget = adjustSetpointForWrap(targetRad);

                  // Log setpoint validity and shot type
                  Logger.recordOutput("Turret/Aiming/SetpointValid", setpoint.isValid());
                  Logger.recordOutput(
                      "Turret/Aiming/IsNeutralZoneShot", setpoint.isNeutralZoneShot());

                  // Convert feedforward velocity to volts for motor control
                  // Phoenix 6 SensorToMechanismRatio handles unit conversion automatically
                  // KV is in volts per (mechanism rotations/second)
                  double feedforwardVolts = feedforwardRadPerSec * Constants.TurretConstants.KV;

                  setPositionSetpointImpl(wrappedTarget, feedforwardVolts);
                  positionSetpointRad = wrappedTarget;
                }))
        .withName("Turret Aim Hub");
  }

  /**
   * Command to set turret position with feedforward voltage. Implements Team 254's wrapping logic
   * for smooth continuous rotation.
   */
  public Command positionSetpointCommand(
      DoubleSupplier radiansFromCenter, DoubleSupplier feedforwardVolts) {
    return positionSetpointUntilUnwrapped(radiansFromCenter, feedforwardVolts)
        .andThen(
            run(
                () -> {
                  double setpoint = adjustSetpointForWrap(radiansFromCenter.getAsDouble());
                  setPositionSetpointImpl(setpoint, feedforwardVolts.getAsDouble());
                  positionSetpointRad = setpoint;
                }))
        .withName("Turret Position Setpoint");
  }

  /** Runs position control until the turret is "unwrapped" (not crossing the wrap point). */
  private Command positionSetpointUntilUnwrapped(
      DoubleSupplier radiansFromCenter, DoubleSupplier feedforwardVolts) {
    return run(() -> {
          double setpoint = radiansFromCenter.getAsDouble();
          // Don't use feedforward until unwrapped to prevent oscillation
          setPositionSetpointImpl(
              setpoint, isUnwrapped(setpoint) ? feedforwardVolts.getAsDouble() : 0.0);
          positionSetpointRad = setpoint;
        })
        .until(() -> isUnwrapped(radiansFromCenter.getAsDouble()));
  }

  /**
   * Adjusts setpoint to minimize rotation while respecting physical limits. If the shortest path
   * through wrapping would exceed limits, takes the long way instead.
   */
  private double adjustSetpointForWrap(double targetAngle) {
    double currentPos = getCurrentPosition();

    // Normalize target angle to [-π, π]
    double normalizedTarget = Math.atan2(Math.sin(targetAngle), Math.cos(targetAngle));

    // Calculate alternative angle (wrapped by ±2π)
    double alternative =
        normalizedTarget > 0.0
            ? normalizedTarget - 2.0 * Math.PI
            : normalizedTarget + 2.0 * Math.PI;

    // Check which path is shorter
    double distanceToTarget = Math.abs(currentPos - normalizedTarget);
    double distanceToAlternative = Math.abs(currentPos - alternative);

    // Choose the path that:
    // 1. Is shorter
    // 2. Doesn't exceed physical limits
    // Limits are now in radians directly
    double minLimit = Constants.TurretConstants.MIN_POSITION_RAD;
    double maxLimit = Constants.TurretConstants.MAX_POSITION_RAD;

    // Check if normalized target is within limits
    boolean targetInBounds = normalizedTarget >= minLimit && normalizedTarget <= maxLimit;

    // Check if alternative is within limits
    boolean alternativeInBounds = alternative >= minLimit && alternative <= maxLimit;

    // Decision logic:
    // 1. If both in bounds, choose shortest path
    // 2. If only one in bounds, choose that one
    // 3. If neither in bounds (shouldn't happen), choose normalized
    if (targetInBounds && alternativeInBounds) {
      return distanceToAlternative < distanceToTarget ? alternative : normalizedTarget;
    } else if (alternativeInBounds) {
      return alternative;
    } else {
      return normalizedTarget;
    }
  }

  /** Checks if turret is unwrapped (not currently crossing the wrap boundary). */
  private boolean isUnwrapped(double setpoint) {
    double timeSinceModeChange = (System.currentTimeMillis() / 1000.0) - lastModeChange;
    return (timeSinceModeChange > 0.5)
        || Math.abs(getCurrentPosition() - setpoint) < Math.toRadians(10.0);
  }

  private void setPositionSetpointImpl(double radiansFromCenter, double feedforwardVolts) {
    // Convert radians to rotations for hardware interface
    // feedforwardVolts is already in volts, pass directly
    io.setPositionSetpoint(Units.radiansToRotations(radiansFromCenter), feedforwardVolts);
  }

  private void updateModeChange() {
    lastModeChange = System.currentTimeMillis() / 1000.0;
  }

  public double getCurrentPosition() {
    return Units.rotationsToRadians(inputs.positionRot);
  }

  public double getSetpoint() {
    return positionSetpointRad;
  }

  public TurretState getCurrentState() {
    return currentState;
  }

  /** Checks if turret is at the desired setpoint within tolerance. */
  public boolean atSetpoint() {
    return Math.abs(getCurrentPosition() - positionSetpointRad)
        < Constants.TurretConstants.AIMING_TOLERANCE_RAD;
  }

  /** Command for open-loop duty cycle control (for testing). */
  public Command setDutyCycle(DoubleSupplier dutyCycle) {
    return run(() -> io.setOpenLoopDutyCycle(dutyCycle.getAsDouble()))
        .finallyDo(() -> io.stop())
        .withName("Turret Duty Cycle");
  }
}
