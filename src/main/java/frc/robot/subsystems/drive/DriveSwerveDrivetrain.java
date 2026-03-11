// Copyright (c) 2021-2026 Littleton Robotics
// Adapted from Team 254's 2025 code
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/**
 * Simple Drive subsystem that wraps DriveIOHardware/DriveIOSim (which extend SwerveDrivetrain).
 * This matches Team 254's architecture of using CTRE's SwerveDrivetrain class directly.
 */
public class DriveSwerveDrivetrain extends SubsystemBase {
  private final DriveIOHardware driveIO;
  private final RobotState robotState;

  // Shuffleboard field visualization (works without AdvantageScope)
  private final Field2d field2d = new Field2d();

  // BlueAlliance perspective — teleop code handles alliance flipping manually
  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric().withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();

  // ── DriveToPoint (2910-style) ──────────────────────────────────────────────
  // Uses FieldCentricFacingAngle so CTRE's 250 Hz odometry thread handles heading,
  // while a software PID (auto or teleop gains) computes the translation velocity.
  private final SwerveRequest.FieldCentricFacingAngle driveToPointRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  private final PIDController autoDriveToPointController =
      new PIDController(
          AutoConstants.DRIVE_TO_POINT_AUTO_KP, 0, AutoConstants.DRIVE_TO_POINT_AUTO_KD);
  private final PIDController teleopDriveToPointController =
      new PIDController(
          AutoConstants.DRIVE_TO_POINT_TELEOP_KP, 0, AutoConstants.DRIVE_TO_POINT_TELEOP_KD);

  /** Whether the drive-to-point state is currently active. */
  private boolean driveToPointActive = false;

  private Pose2d desiredPoseForDriveToPoint = new Pose2d();
  private double maxVelocityOutputForDriveToPoint =
      AutoConstants.DRIVE_TO_POINT_DEFAULT_MAX_VELOCITY_MPS;
  /** NaN = no angular velocity constraint (use default PID). */
  private double maximumAngularVelocityForDriveToPoint = Double.NaN;

  // Acceleration tracking (computed from speed deltas each periodic cycle)
  private ChassisSpeeds previousSpeeds = new ChassisSpeeds();
  private double previousTimestamp = 0.0;
  private double linearAccelerationMagnitude = 0.0;

  /** Cached pose (lock-free read per cycle, avoids contention on RobotState's TreeMap). */
  private volatile Pose2d cachedPose = new Pose2d();

  /** EMA of acceleration (smooths noise for feeding gate). */
  private static final double ACCEL_FILTER_ALPHA = 0.10;

  private double filteredAcceleration = 0.0;

  public DriveSwerveDrivetrain(DriveIOHardware driveIO, RobotState robotState) {
    this.driveIO = driveIO;
    this.robotState = robotState;

    // Configure the CTRE PhoenixPIDController for heading (runs at 250 Hz in odometry thread)
    driveToPointRequest.HeadingController =
        new PhoenixPIDController(AutoConstants.DRIVE_TO_POINT_HEADING_KP, 0, 0);
    driveToPointRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    // Publish Field2d to SmartDashboard/Shuffleboard for field visualization at comp
    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void periodic() {
    // Cache pose once per cycle to avoid repeated lock acquisitions on RobotState
    var latestEntry = robotState.getLatestFieldToRobot();
    cachedPose = (latestEntry != null) ? latestEntry.getValue() : new Pose2d();
    // CTRE's SwerveDriveState.Speeds are robot-relative
    var driveState = driveIO.getState();
    var robotRelativeSpeeds = driveState.Speeds;

    // Log swerve module states for AdvantageScope visualization
    Logger.recordOutput("Drive/SwerveStates/Measured", driveState.ModuleStates);
    Logger.recordOutput("Drive/SwerveStates/Setpoints", driveState.ModuleTargets);

    // Compute field-relative speeds by rotating by robot heading
    var heading = getPose().getRotation();
    var fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, heading);

    // Compute linear acceleration magnitude (m/s²) from speed deltas
    double now = Timer.getFPGATimestamp();
    double dt = now - previousTimestamp;
    if (dt > 0.005 && dt < 0.5) { // Guard against first-cycle and large gaps
      double dvx = fieldRelativeSpeeds.vxMetersPerSecond - previousSpeeds.vxMetersPerSecond;
      double dvy = fieldRelativeSpeeds.vyMetersPerSecond - previousSpeeds.vyMetersPerSecond;
      linearAccelerationMagnitude = Math.sqrt(dvx * dvx + dvy * dvy) / dt;
      // Exponential moving average to smooth out frame-to-frame noise
      filteredAcceleration =
          ACCEL_FILTER_ALPHA * linearAccelerationMagnitude
              + (1.0 - ACCEL_FILTER_ALPHA) * filteredAcceleration;
    }
    previousSpeeds = fieldRelativeSpeeds;
    previousTimestamp = now;
    Logger.recordOutput("Drive/LinearAccelMagnitude", linearAccelerationMagnitude);
    Logger.recordOutput("Drive/FilteredAcceleration", filteredAcceleration);

    robotState.updateChassisSpeeds(fieldRelativeSpeeds, robotRelativeSpeeds);

    // Log robot state
    robotState.log();

    // Update Shuffleboard field visualization
    field2d.setRobotPose(getPose());

    // Log simulated pose if using DriveIOSim
    if (driveIO instanceof DriveIOSim) {
      ((DriveIOSim) driveIO).logSimulatedPose();
    }

    // ── DriveToPoint state application ─────────────────────────────────────
    if (driveToPointActive) {
      applyDriveToPoint();
    }
  }

  /** Get the current robot pose (cached per periodic cycle — lock-free). */
  public Pose2d getPose() {
    return cachedPose;
  }

  /** Get RobotState object for commands */
  public RobotState getRobotState() {
    return robotState;
  }

  /** Get current chassis speeds */
  public ChassisSpeeds getChassisSpeeds() {
    return driveIO.getState().Speeds;
  }

  /** Linear speed (m/s), ignoring rotation. Used for stop/move gating. */
  public double getLinearSpeedMps() {
    ChassisSpeeds speeds = getChassisSpeeds();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  /** Raw linear acceleration magnitude (m/s²). Used for Superstructure feeding gate. */
  public double getLinearAccelerationMagnitude() {
    return linearAccelerationMagnitude;
  }

  /** Smoothed acceleration (m/s²). Avoids stutter in gating decisions. */
  public double getFilteredAcceleration() {
    return filteredAcceleration;
  }

  /** Reset the robot's pose. */
  public void setPose(Pose2d pose) {
    driveIO.resetOdometry(pose);
  }

  /**
   * Drive the robot in field-relative mode.
   *
   * @param vx Forward velocity in m/s
   * @param vy Sideways velocity in m/s
   * @param omega Rotational velocity in rad/s
   */
  public void driveFieldRelative(double vx, double vy, double omega) {
    driveToPointActive = false;
    driveIO.setControl(
        fieldCentricDrive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
  }

  /**
   * Drive the robot in robot-relative mode.
   *
   * @param vx Forward velocity in m/s
   * @param vy Sideways velocity in m/s
   * @param omega Rotational velocity in rad/s
   */
  public void driveRobotRelative(double vx, double vy, double omega) {
    driveIO.setControl(
        robotCentricDrive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
  }

  /** Called by PathPlanner's AutoBuilder output. Robot-centric drive. */
  public void runVelocity(ChassisSpeeds chassisSpeeds) {
    driveRobotRelative(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond);
  }

  /** Apply a custom swerve request. */
  public void applyRequest(SwerveRequest request) {
    driveIO.setControl(request);
  }

  /** Stop the drivetrain and clear any drive-to-point state. */
  public void stop() {
    driveToPointActive = false;
    driveRobotRelative(0, 0, 0);
  }

  // ── DriveToPoint public API (2910-style) ─────────────────────────────────

  /**
   * Activate drive-to-point mode with default max velocity and no angular velocity constraint.
   *
   * @param pose The target pose (translation + heading)
   */
  public void setDesiredPoseForDriveToPoint(Pose2d pose) {
    this.desiredPoseForDriveToPoint = pose;
    this.maxVelocityOutputForDriveToPoint = AutoConstants.DRIVE_TO_POINT_DEFAULT_MAX_VELOCITY_MPS;
    this.maximumAngularVelocityForDriveToPoint = Double.NaN;
    this.driveToPointActive = true;
  }

  /**
   * Activate drive-to-point mode with a custom max velocity output.
   *
   * @param pose The target pose (translation + heading)
   * @param maxVelocityOutput Maximum translation velocity (m/s)
   */
  public void setDesiredPoseForDriveToPoint(Pose2d pose, double maxVelocityOutput) {
    this.desiredPoseForDriveToPoint = pose;
    this.maxVelocityOutputForDriveToPoint = maxVelocityOutput;
    this.maximumAngularVelocityForDriveToPoint = Double.NaN;
    this.driveToPointActive = true;
  }

  /** Activate drive-to-point with custom max velocity and angular velocity constraint. */
  public void setDesiredPoseForDriveToPointWithConstraints(
      Pose2d pose, double maxVelocityOutput, double maxAngularVelocity) {
    this.desiredPoseForDriveToPoint = pose;
    this.maxVelocityOutputForDriveToPoint = maxVelocityOutput;
    this.maximumAngularVelocityForDriveToPoint = maxAngularVelocity;
    this.driveToPointActive = true;
  }

  /** Deactivate drive-to-point mode. The next teleop command will resume normal driving. */
  public void clearDriveToPoint() {
    this.driveToPointActive = false;
  }

  /** Whether the drive-to-point state is currently active. */
  public boolean isDriveToPointActive() {
    return driveToPointActive;
  }

  /**
   * Check whether the robot is within tolerance of the drive-to-point setpoint (translation only).
   */
  public boolean isAtDriveToPointSetpoint() {
    double distance = getDistanceFromDriveToPointSetpoint();
    Logger.recordOutput("Drive/DriveToPoint/DistanceFromEndpoint", distance);
    return MathUtil.isNear(0.0, distance, AutoConstants.DRIVE_TO_POINT_POSITION_TOLERANCE_M);
  }

  /**
   * Check whether the robot is within the given tolerances of the drive-to-point setpoint
   * (translation and heading).
   *
   * @param positionTolerance Position tolerance in meters
   * @param headingTolerance Heading tolerance in radians
   */
  public boolean isAtDriveToPointSetpoint(double positionTolerance, double headingTolerance) {
    double distance = getDistanceFromDriveToPointSetpoint();
    double headingError =
        Math.abs(
            MathUtil.angleModulus(
                cachedPose.getRotation().getRadians()
                    - desiredPoseForDriveToPoint.getRotation().getRadians()));
    Logger.recordOutput("Drive/DriveToPoint/DistanceFromEndpoint", distance);
    Logger.recordOutput("Drive/DriveToPoint/HeadingError", Math.toDegrees(headingError));
    return distance <= positionTolerance && headingError <= headingTolerance;
  }

  /** Get the scalar distance (m) from the current position to the drive-to-point setpoint. */
  public double getDistanceFromDriveToPointSetpoint() {
    return desiredPoseForDriveToPoint.getTranslation().minus(cachedPose.getTranslation()).getNorm();
  }

  /** Core drive-to-point logic (2910-style PID on scalar distance, CTRE 250Hz heading). */
  private void applyDriveToPoint() {
    Translation2d translationToDesiredPoint =
        desiredPoseForDriveToPoint.getTranslation().minus(cachedPose.getTranslation());
    double linearDistance = translationToDesiredPoint.getNorm();

    // Static friction feedforward: only apply when > 0.5 inches to prevent oscillation at zero
    double frictionFF = 0.0;
    if (linearDistance >= Units.inchesToMeters(0.5)) {
      frictionFF = AutoConstants.DRIVE_TO_POINT_FRICTION_FF * maxVelocityOutputForDriveToPoint;
    }

    // Direction of travel (angle from current position toward target)
    Rotation2d directionOfTravel = translationToDesiredPoint.getAngle();

    // Select auto or teleop PID gains based on current mode
    double velocityOutput;
    if (DriverStation.isAutonomous()) {
      velocityOutput =
          Math.min(
              Math.abs(autoDriveToPointController.calculate(linearDistance, 0.0)) + frictionFF,
              maxVelocityOutputForDriveToPoint);
    } else {
      velocityOutput =
          Math.min(
              Math.abs(teleopDriveToPointController.calculate(linearDistance, 0.0)) + frictionFF,
              maxVelocityOutputForDriveToPoint);
    }

    // Decompose scalar velocity into field-relative X/Y components
    double xComponent = velocityOutput * directionOfTravel.getCos();
    double yComponent = velocityOutput * directionOfTravel.getSin();

    // Log telemetry
    Logger.recordOutput("Drive/DriveToPoint/XVelocity", xComponent);
    Logger.recordOutput("Drive/DriveToPoint/YVelocity", yComponent);
    Logger.recordOutput("Drive/DriveToPoint/VelocityOutput", velocityOutput);
    Logger.recordOutput("Drive/DriveToPoint/LinearDistance", linearDistance);
    Logger.recordOutput("Drive/DriveToPoint/DirectionOfTravel", directionOfTravel);
    Logger.recordOutput("Drive/DriveToPoint/DesiredPoint", desiredPoseForDriveToPoint);

    // Apply using FieldCentricFacingAngle — heading PID runs at 250 Hz in CTRE odometry thread
    if (Double.isNaN(maximumAngularVelocityForDriveToPoint)) {
      driveIO.setControl(
          driveToPointRequest
              .withVelocityX(xComponent)
              .withVelocityY(yComponent)
              .withTargetDirection(desiredPoseForDriveToPoint.getRotation()));
    } else {
      driveIO.setControl(
          driveToPointRequest
              .withVelocityX(xComponent)
              .withVelocityY(yComponent)
              .withTargetDirection(desiredPoseForDriveToPoint.getRotation())
              .withMaxAbsRotationalRate(maximumAngularVelocityForDriveToPoint));
    }
  }

  /** Get the underlying DriveIO instance. */
  public DriveIOHardware getDriveIO() {
    return driveIO;
  }

  /**
   * Get path constraints for PathPlanner pathfinding.
   *
   * @return PathConstraints for autonomous navigation
   */
  public com.pathplanner.lib.path.PathConstraints getPathConstraints() {
    return new com.pathplanner.lib.path.PathConstraints(
        Constants.AutoConstants.PATHFINDING_MAX_VELOCITY_MPS,
        Constants.AutoConstants.PATHFINDING_MAX_ACCELERATION_MPS2,
        Constants.AutoConstants.PATHFINDING_MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
        Constants.AutoConstants.PATHFINDING_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC2);
  }
}
