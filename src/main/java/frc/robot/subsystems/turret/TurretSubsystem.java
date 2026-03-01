package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
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
  private final RobotState robotState;

  private double positionSetpointRad = Constants.TurretConstants.BOOT_POSITION_RAD;
  private double lastModeChange = 0.0;
  private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

  // ShooterSetpoint supplier for coordinated aiming
  private Supplier<ShooterSetpoint> setpointSupplier = ShooterSetpoint::invalid;

  public enum TurretState {
    STOW,
    AIMING
  }

  private TurretState currentState = TurretState.STOW;

  public TurretSubsystem(TurretIO io, RobotState robotState) {
    this.io = io;
    this.robotState = robotState;
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

    // Report turret rotation to RobotState so vision can use it
    double turretPositionRad = Units.rotationsToRadians(inputs.positionRot);
    double turretVelocityRadPerSec = Units.rotationsToRadians(inputs.velocityRotPerSec);
    robotState.addTurretUpdates(
        Timer.getFPGATimestamp(), turretPositionRad, turretVelocityRadPerSec);

    double robotHeadingRad = robotPoseSupplier.get().getRotation().getRadians();
    double turretFieldHeadingRad = robotHeadingRad + turretPositionRad;
    Logger.recordOutput("Turret/FieldHeadingRad", turretFieldHeadingRad);
    Logger.recordOutput("Turret/FieldHeadingDeg", Math.toDegrees(turretFieldHeadingRad));

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
                  ShooterSetpoint setpoint = setpointSupplier.get();
                  double targetRad = setpoint.getTurretAngleRad();
                  double feedforwardRadPerSec = setpoint.getTurretFeedforwardRadPerSec();
                  double wrappedTarget = adjustSetpointForWrap(targetRad);

                  Logger.recordOutput("Turret/Aiming/SetpointValid", setpoint.isValid());
                  Logger.recordOutput(
                      "Turret/Aiming/IsNeutralZoneShot", setpoint.isNeutralZoneShot());

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
   * Adjusts setpoint to minimize rotation while respecting physical limits. Finds the 2π-equivalent
   * of the target angle that is closest to the current turret position, then verifies it lies
   * within the turret's travel range. If not, the closest in-bounds equivalent is used instead.
   *
   * <p>Key insight: the raw target from ShooterSetpoint is always in [-π, π], but the turret can
   * physically travel [-2π, ~1.745 rad]. By picking the equivalent nearest the current position we
   * allow the turret to take the shortest-path through the extended range instead of always staying
   * within [-π, π].
   */
  private double adjustSetpointForWrap(double targetAngle) {
    double currentPos = getCurrentPosition();
    double minLimit = Constants.TurretConstants.MIN_POSITION_RAD;
    double maxLimit = Constants.TurretConstants.MAX_POSITION_RAD;

    // 1. Find the 2π-equivalent of targetAngle closest to currentPos.
    //    This is the "shortest rotation" candidate.
    double diff = targetAngle - currentPos;
    // Normalize diff to (-π, π]
    diff = diff - 2.0 * Math.PI * Math.floor((diff + Math.PI) / (2.0 * Math.PI));
    double closest = currentPos + diff;

    // Log wrapping result only (internals removed to reduce bandwidth — re-enable for debugging)
    Logger.recordOutput("Turret/Wrap/TargetAngleDeg", Math.toDegrees(targetAngle));

    // 2. Check if the closest candidate is in bounds — if so, use it.
    if (closest >= minLimit && closest <= maxLimit) {
      Logger.recordOutput("Turret/Wrap/ResultDeg", Math.toDegrees(closest));
      return closest;
    }

    // 3. Otherwise sweep ±2π offsets from `closest` looking for the nearest
    //    candidate that IS in bounds.
    double bestCandidate = closest;
    double bestDistance = Double.MAX_VALUE;
    boolean foundInBounds = false;

    for (int k = -2; k <= 2; k++) {
      double candidate = closest + k * 2.0 * Math.PI;
      boolean inBounds = candidate >= minLimit && candidate <= maxLimit;

      if (inBounds) {
        double dist = Math.abs(currentPos - candidate);
        if (!foundInBounds || dist < bestDistance) {
          bestCandidate = candidate;
          bestDistance = dist;
          foundInBounds = true;
        }
      } else if (!foundInBounds) {
        double dist = Math.abs(currentPos - candidate);
        if (dist < bestDistance) {
          bestCandidate = candidate;
          bestDistance = dist;
        }
      }
    }

    // 4. If still nothing in bounds, clamp to limits.
    if (!foundInBounds) {
      bestCandidate = Math.max(minLimit, Math.min(maxLimit, bestCandidate));
    }

    Logger.recordOutput("Turret/Wrap/ResultDeg", Math.toDegrees(bestCandidate));
    Logger.recordOutput("Turret/Wrap/UsedFallback", true);
    return bestCandidate;
  }

  /** Checks if turret is unwrapped (not currently crossing the wrap boundary). */
  private boolean isUnwrapped(double setpoint) {
    double timeSinceModeChange = Timer.getFPGATimestamp() - lastModeChange;
    return (timeSinceModeChange > 0.5)
        || Math.abs(getCurrentPosition() - setpoint) < Math.toRadians(10.0);
  }

  private void setPositionSetpointImpl(double radiansFromCenter, double feedforwardVolts) {
    io.setPositionSetpoint(Units.radiansToRotations(radiansFromCenter), feedforwardVolts);
  }

  private void updateModeChange() {
    lastModeChange = Timer.getFPGATimestamp();
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

  /**
   * Directly apply the stow position. Called by Superstructure.periodic() every cycle when the
   * wanted state requires the turret to be stowed. Unlike the stow() command, this is a plain void
   * method — no command scheduling overhead.
   */
  public void applyStow() {
    if (currentState != TurretState.STOW) {
      currentState = TurretState.STOW;
      updateModeChange();
    }
    double setpoint = adjustSetpointForWrap(Constants.TurretConstants.STOW_POSITION);
    setPositionSetpointImpl(setpoint, 0.0);
    positionSetpointRad = setpoint;
  }

  /**
   * Directly apply the aiming position using ShooterSetpoint. Called by Superstructure.periodic()
   * every cycle when the wanted state requires the turret to aim. Unlike the aiming() command, this
   * is a plain void method — no command scheduling overhead.
   */
  public void applyAiming() {
    if (currentState != TurretState.AIMING) {
      currentState = TurretState.AIMING;
      updateModeChange();
    }
    ShooterSetpoint setpoint = setpointSupplier.get();
    double targetRad = setpoint.getTurretAngleRad();
    double feedforwardRadPerSec = setpoint.getTurretFeedforwardRadPerSec();
    double wrappedTarget = adjustSetpointForWrap(targetRad);

    Logger.recordOutput("Turret/Aiming/SetpointValid", setpoint.isValid());
    Logger.recordOutput("Turret/Aiming/IsNeutralZoneShot", setpoint.isNeutralZoneShot());

    double feedforwardVolts = feedforwardRadPerSec * Constants.TurretConstants.KV;
    setPositionSetpointImpl(wrappedTarget, feedforwardVolts);
    positionSetpointRad = wrappedTarget;
  }

  /** Command for open-loop duty cycle control (for testing). */
  public Command setDutyCycle(DoubleSupplier dutyCycle) {
    return run(() -> io.setOpenLoopDutyCycle(dutyCycle.getAsDouble()))
        .finallyDo(() -> io.stop())
        .withName("Turret Duty Cycle");
  }
}
