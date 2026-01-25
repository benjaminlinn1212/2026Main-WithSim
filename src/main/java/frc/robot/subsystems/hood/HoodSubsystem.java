package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Hood subsystem that controls the hood angle for shot trajectory adjustment. Supports multiple
 * states: stow, shootBackFromNeutralZone, and aimHub.
 */
public class HoodSubsystem extends SubsystemBase {
  private final HoodIO io;
  private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();

  private double positionSetpointRad = Constants.HoodConstants.STOW_POSITION;
  private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

  public enum HoodState {
    STOW,
    SHOOT_BACK_FROM_NEUTRAL_ZONE,
    AIM_HUB
  }

  private HoodState currentState = HoodState.STOW;

  public HoodSubsystem(HoodIO io) {
    this.io = io;
  }

  /**
   * Set the robot pose supplier for field-relative distance calculation. Call this from
   * RobotContainer after constructing the drive subsystem.
   */
  public void setRobotPoseSupplier(Supplier<Pose2d> poseSupplier) {
    this.robotPoseSupplier = poseSupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Log all hood inputs
    Logger.recordOutput("Hood/PositionRad", inputs.positionRad);
    Logger.recordOutput("Hood/VelocityRadPerSec", inputs.velocityRadPerSec);
    Logger.recordOutput("Hood/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("Hood/CurrentStatorAmps", inputs.currentStatorAmps);
    Logger.recordOutput("Hood/CurrentSupplyAmps", inputs.currentSupplyAmps);
    Logger.recordOutput("Hood/TemperatureCelsius", inputs.temperatureCelsius);

    Logger.recordOutput("Hood/State", currentState.toString());
    Logger.recordOutput("Hood/SetpointRad", positionSetpointRad);
  }

  /** Command to set hood to stow position (safe angle). */
  public Command stow() {
    return runOnce(
            () -> {
              currentState = HoodState.STOW;
            })
        .andThen(positionSetpointCommand(() -> Constants.HoodConstants.STOW_POSITION, () -> 0.0))
        .withName("Hood Stow");
  }

  /**
   * Command to set hood angle for shooting back from neutral zone. Uses a fixed angle optimized for
   * neutral zone shots.
   */
  public Command shootBackFromNeutralZone() {
    return runOnce(
            () -> {
              currentState = HoodState.SHOOT_BACK_FROM_NEUTRAL_ZONE;
            })
        .andThen(
            positionSetpointCommand(() -> Constants.HoodConstants.SHOOT_BACK_POSITION, () -> 0.0))
        .withName("Hood Shoot Back From Neutral Zone");
  }

  /**
   * Command to aim hood at the hub based on robot pose and distance. Calculates optimal angle based
   * on distance to target.
   */
  public Command aimHub() {
    return runOnce(
            () -> {
              currentState = HoodState.AIM_HUB;
            })
        .andThen(
            positionSetpointCommand(
                () -> {
                  // Get robot position from pose supplier
                  Pose2d robotPose = robotPoseSupplier.get();
                  Translation2d robotPosition = robotPose.getTranslation();

                  // Get target position based on alliance
                  Translation3d targetPosition;
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    targetPosition = Constants.FieldPoses.RED_AIM_TARGET;
                  } else {
                    targetPosition = Constants.FieldPoses.BLUE_AIM_TARGET;
                  }

                  // Calculate distance to target
                  Translation2d robotToTarget =
                      new Translation2d(targetPosition.getX(), targetPosition.getY())
                          .minus(robotPosition);
                  double distanceToTarget = robotToTarget.getNorm();

                  // Calculate hood angle based on distance
                  // This is a simplified ballistic calculation
                  // In reality, you'd use a lookup table or more complex physics
                  // double targetHeight = targetPosition.getZ();
                  // double launchHeight = 0.5; // Approximate shooter height in meters
                  // TODO: Use height difference for ballistic calculations

                  // Simple angle calculation (can be improved with ballistics)
                  // For now, use a linear interpolation based on distance
                  double minDistance = 1.0; // meters
                  double maxDistance = 6.0; // meters

                  double normalizedDistance =
                      (distanceToTarget - minDistance) / (maxDistance - minDistance);
                  normalizedDistance = Math.max(0.0, Math.min(1.0, normalizedDistance));

                  // Interpolate between min and max hood angles
                  double hoodAngle =
                      Constants.HoodConstants.MIN_AIM_ANGLE
                          + normalizedDistance
                              * (Constants.HoodConstants.MAX_POSITION_RAD
                                  - Constants.HoodConstants.MIN_AIM_ANGLE);

                  return hoodAngle;
                },
                () -> 0.0)) // TODO: Add feedforward for moving shots
        .withName("Hood Aim Hub");
  }

  /** Command to set hood position with feedforward velocity. */
  public Command positionSetpointCommand(
      DoubleSupplier radiansFromHorizontal, DoubleSupplier ffVelocity) {
    return run(() -> {
          double setpoint = radiansFromHorizontal.getAsDouble();
          setPositionSetpointImpl(setpoint, ffVelocity.getAsDouble());
          positionSetpointRad = setpoint;
        })
        .withName("Hood Position Setpoint");
  }

  private void setPositionSetpointImpl(double radiansFromHorizontal, double radPerSecond) {
    Logger.recordOutput("Hood/SetPositionSetpoint/radiansFromHorizontal", radiansFromHorizontal);
    Logger.recordOutput("Hood/SetPositionSetpoint/radPerSecond", radPerSecond);
    io.setPositionSetpoint(radiansFromHorizontal, radPerSecond);
  }

  public double getCurrentPosition() {
    return inputs.positionRad;
  }

  public double getSetpoint() {
    return positionSetpointRad;
  }

  public HoodState getCurrentState() {
    return currentState;
  }

  /** Checks if hood is at the desired setpoint within tolerance. */
  public boolean atSetpoint() {
    return Math.abs(getCurrentPosition() - positionSetpointRad)
        < Constants.HoodConstants.AIMING_TOLERANCE_RAD;
  }

  /** Command for open-loop duty cycle control (for testing). */
  public Command setDutyCycle(DoubleSupplier dutyCycle) {
    return run(() -> io.setOpenLoopDutyCycle(dutyCycle.getAsDouble()))
        .finallyDo(() -> io.stop())
        .withName("Hood Duty Cycle");
  }
}
