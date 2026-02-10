// Copyright (c) 2021-2026 Littleton Robotics
// Adapted from Team 254's 2025 code
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

/**
 * Simple Drive subsystem that wraps DriveIOHardware/DriveIOSim (which extend SwerveDrivetrain).
 * This matches Team 254's architecture of using CTRE's SwerveDrivetrain class directly.
 */
public class DriveSwerveDrivetrain extends SubsystemBase {
  private final DriveIOHardware driveIO;
  private final RobotState robotState;

  // SwerveRequest objects for different drive modes
  private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric();
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();

  public DriveSwerveDrivetrain(DriveIOHardware driveIO, RobotState robotState) {
    this.driveIO = driveIO;
    this.robotState = robotState;
  }

  @Override
  public void periodic() {
    // CTRE's SwerveDriveState.Speeds are robot-relative
    var robotRelativeSpeeds = driveIO.getState().Speeds;

    // Compute field-relative speeds by rotating by robot heading
    var heading = getPose().getRotation();
    var fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, heading);

    robotState.updateChassisSpeeds(fieldRelativeSpeeds, robotRelativeSpeeds);

    // Log robot state
    robotState.log();

    // Log simulated pose if using DriveIOSim
    if (driveIO instanceof DriveIOSim) {
      ((DriveIOSim) driveIO).logSimulatedPose();
    }
  }

  /** Get the current robot pose from RobotState (matches 254) */
  public Pose2d getPose() {
    if (robotState.getLatestFieldToRobot() != null) {
      return robotState.getLatestFieldToRobot().getValue();
    }
    return new Pose2d(); // Fallback
  }

  /** Get RobotState object for commands */
  public RobotState getRobotState() {
    return robotState;
  }

  /** Get current chassis speeds */
  public ChassisSpeeds getChassisSpeeds() {
    return driveIO.getState().Speeds;
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

  /** Drive using chassis speeds. */
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

  /** Stop the drivetrain. */
  public void stop() {
    driveRobotRelative(0, 0, 0);
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
