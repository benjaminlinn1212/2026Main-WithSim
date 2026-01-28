package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Turret subsystem that controls a rotating turret mechanism. Supports multiple states: stow,
 * shootBackFromNeutralZone, and aimHub. Uses Team 254's wrapping logic for smooth continuous
 * rotation.
 */
public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private double positionSetpointRad = 0.0;
  private double lastModeChange = 0.0;
  private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();
  private Supplier<ChassisSpeeds> chassisSpeedsSupplier = () -> new ChassisSpeeds();

  public enum TurretState {
    STOW,
    SHOOT_BACK_FROM_NEUTRAL_ZONE,
    AIM_HUB
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
   * Set the chassis speeds supplier for velocity feedforward compensation. Call this from
   * RobotContainer after constructing the drive subsystem.
   */
  public void setChassisSpeedsSupplier(Supplier<ChassisSpeeds> speedsSupplier) {
    this.chassisSpeedsSupplier = speedsSupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    Logger.recordOutput("Turret/State", currentState.toString());
    Logger.recordOutput("Turret/SetpointRad", positionSetpointRad);

    // Calculate and log position error
    double errorRad = positionSetpointRad - Units.rotationsToRadians(inputs.positionRot);
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
   * Command to aim turret for shooting back from neutral zone. Aims at 0 degrees for blue alliance,
   * 180 degrees for red alliance.
   */
  public Command shootBackFromNeutralZone() {
    return runOnce(
            () -> {
              currentState = TurretState.SHOOT_BACK_FROM_NEUTRAL_ZONE;
              updateModeChange();
            })
        .andThen(
            run(
                () -> {
                  // Get robot pose
                  Pose2d robotPose = robotPoseSupplier.get();
                  Rotation2d robotHeading = robotPose.getRotation();

                  // Determine field-relative direction to face alliance wall
                  Rotation2d fieldDirection;
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    // Red: face toward red wall (X+ direction, 0°)
                    fieldDirection = Rotation2d.fromDegrees(0);
                  } else {
                    // Blue: face toward blue wall (X- direction, 180°)
                    fieldDirection = Rotation2d.fromDegrees(180);
                  }

                  // Convert field direction to turret-relative angle
                  Rotation2d turretAngle = fieldDirection.minus(robotHeading);

                  // Get the target angle and apply wrapping logic for shortest path
                  double targetRad = turretAngle.getRadians();
                  double wrappedTarget = adjustSetpointForWrap(targetRad);

                  // Set position with wrapping support
                  setPositionSetpointImpl(wrappedTarget, 0.0);
                  positionSetpointRad = wrappedTarget;
                }))
        .withName("Turret Shoot Back From Neutral Zone");
  }

  /**
   * Command to aim turret at the hub based on robot pose. Uses the appropriate aim target based on
   * alliance color. Continuously tracks target even when robot rotates.
   *
   * <p>IMPORTANT: Accounts for turret offset from robot center. When the turret is not at the
   * robot's center, the aiming angle must be calculated from the turret's actual field position,
   * not the robot's center position.
   *
   * <p>Calculation steps: 1. Get robot center position and heading 2. Transform turret offset
   * through robot rotation to get turret's field position 3. Calculate angle from turret position
   * to target (not from robot center) 4. Convert field-relative angle to turret-relative angle
   */
  public Command aimHub() {
    return runOnce(
            () -> {
              currentState = TurretState.AIM_HUB;
              updateModeChange();
            })
        .andThen(
            run(
                () -> {
                  // Get robot pose
                  Pose2d robotPose = robotPoseSupplier.get();
                  Translation2d robotPosition = robotPose.getTranslation();
                  Rotation2d robotHeading = robotPose.getRotation();

                  // Calculate turret's actual field position by transforming offset through robot
                  // rotation
                  Translation2d turretOffset =
                      Constants.TurretConstants.TURRET_OFFSET_FROM_ROBOT_CENTER;
                  Translation2d turretOffsetRotated = turretOffset.rotateBy(robotHeading);
                  Translation2d turretPosition = robotPosition.plus(turretOffsetRotated);

                  // Get target position based on alliance
                  Translation3d targetPosition;
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    targetPosition = Constants.FieldPoses.RED_AIM_TARGET;
                  } else {
                    targetPosition = Constants.FieldPoses.BLUE_AIM_TARGET;
                  }

                  // Calculate angle to target (field-relative) from TURRET position
                  Translation2d turretToTarget =
                      new Translation2d(targetPosition.getX(), targetPosition.getY())
                          .minus(turretPosition);
                  Rotation2d angleToTargetFieldRelative = turretToTarget.getAngle();

                  // Convert to turret-relative angle by subtracting robot heading
                  // Turret angle = field angle to target - robot heading
                  Rotation2d turretAngle = angleToTargetFieldRelative.minus(robotHeading);

                  // Calculate feedforward velocity to compensate for robot motion
                  ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();

                  // Component 1: Robot rotation (turret must counter-rotate)
                  double rotationAngularVel = -chassisSpeeds.omegaRadiansPerSecond;

                  // Component 2: Translation around target (changes angle to target)
                  double distanceToTarget = turretToTarget.getNorm();
                  double translationAngularVel = 0.0;

                  if (distanceToTarget > 0.01) { // Avoid division by zero
                    // Get velocity in field frame
                    double vxField = chassisSpeeds.vxMetersPerSecond;
                    double vyField = chassisSpeeds.vyMetersPerSecond;

                    // Project velocity onto perpendicular direction to target
                    // Perpendicular direction is 90° from angle to target
                    double perpAngle = angleToTargetFieldRelative.getRadians() + Math.PI / 2.0;
                    double vPerpendicular =
                        vxField * Math.cos(perpAngle) + vyField * Math.sin(perpAngle);

                    // Angular rate of change of target angle due to translation
                    translationAngularVel = vPerpendicular / distanceToTarget;
                  }

                  // Total angular velocity of target angle
                  double totalAngularVel = rotationAngularVel + translationAngularVel;

                  // Predict future angle to compensate for latency (lookahead)
                  // Assume ~50ms total system latency (controls + motor response)
                  double latencySeconds = Constants.TurretConstants.AIMING_LATENCY_COMPENSATION;
                  double targetRad = turretAngle.getRadians();
                  double predictedTargetRad = targetRad + (totalAngularVel * latencySeconds);

                  // Apply wrapping logic to predicted target
                  double wrappedTarget = adjustSetpointForWrap(predictedTargetRad);

                  // Log aiming details for tuning
                  Logger.recordOutput("Turret/Aiming/RobotPosition", robotPosition);
                  Logger.recordOutput("Turret/Aiming/TurretPosition", turretPosition);
                  Logger.recordOutput("Turret/Aiming/TargetAngleRad", targetRad);
                  Logger.recordOutput("Turret/Aiming/PredictedAngleRad", predictedTargetRad);
                  Logger.recordOutput("Turret/Aiming/RotationAngularVel", rotationAngularVel);
                  Logger.recordOutput("Turret/Aiming/TranslationAngularVel", translationAngularVel);
                  Logger.recordOutput("Turret/Aiming/TotalAngularVel", totalAngularVel);
                  Logger.recordOutput("Turret/Aiming/FeedforwardVel", totalAngularVel);
                  Logger.recordOutput("Turret/Aiming/DistanceToTarget", distanceToTarget);

                  // Use total angular velocity as feedforward
                  setPositionSetpointImpl(wrappedTarget, totalAngularVel);
                  positionSetpointRad = wrappedTarget;
                }))
        .withName("Turret Aim Hub");
  }

  /**
   * Command to set turret position with feedforward velocity. Implements Team 254's wrapping logic
   * for smooth continuous rotation.
   */
  public Command positionSetpointCommand(
      DoubleSupplier radiansFromCenter, DoubleSupplier ffVelocity) {
    return positionSetpointUntilUnwrapped(radiansFromCenter, ffVelocity)
        .andThen(
            run(
                () -> {
                  double setpoint = adjustSetpointForWrap(radiansFromCenter.getAsDouble());
                  setPositionSetpointImpl(setpoint, ffVelocity.getAsDouble());
                  positionSetpointRad = setpoint;
                }))
        .withName("Turret Position Setpoint");
  }

  /** Runs position control until the turret is "unwrapped" (not crossing the wrap point). */
  private Command positionSetpointUntilUnwrapped(
      DoubleSupplier radiansFromCenter, DoubleSupplier ffVelocity) {
    return run(() -> {
          double setpoint = radiansFromCenter.getAsDouble();
          // Don't use feedforward until unwrapped to prevent oscillation
          setPositionSetpointImpl(setpoint, isUnwrapped(setpoint) ? ffVelocity.getAsDouble() : 0.0);
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

  private void setPositionSetpointImpl(double radiansFromCenter, double radPerSecond) {
    Logger.recordOutput("Turret/SetPositionSetpoint/radiansFromCenter", radiansFromCenter);
    Logger.recordOutput("Turret/SetPositionSetpoint/radPerSecond", radPerSecond);
    // Convert radians to rotations for hardware interface
    io.setPositionSetpoint(
        Units.radiansToRotations(radiansFromCenter), Units.radiansToRotations(radPerSecond));
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
        < Constants.TurretConstants.AIMING_TOLERANCE_ROT;
  }

  /** Command for open-loop duty cycle control (for testing). */
  public Command setDutyCycle(DoubleSupplier dutyCycle) {
    return run(() -> io.setOpenLoopDutyCycle(dutyCycle.getAsDouble()))
        .finallyDo(() -> io.stop())
        .withName("Turret Duty Cycle");
  }
}
