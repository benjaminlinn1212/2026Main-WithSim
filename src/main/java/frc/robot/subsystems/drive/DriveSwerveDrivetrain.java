// Copyright (c) 2021-2026 Littleton Robotics
// Adapted from Team 254's 2025 code
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  // SwerveRequest objects for different drive modes
  // Use BlueAlliance perspective because teleop code already handles alliance flipping manually
  // (negating vx/vy for red). OperatorPerspective (default) would apply an additional rotation
  // via operatorForwardDirection, which is never set and can cause wrong headings in sim.
  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric().withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();

  // Acceleration tracking (computed from speed deltas each periodic cycle)
  private ChassisSpeeds previousSpeeds = new ChassisSpeeds();
  private double previousTimestamp = 0.0;
  private double linearAccelerationMagnitude = 0.0;

  /**
   * Exponential moving average of acceleration. Smooths out frame-to-frame noise so the zone-aware
   * feeding gate doesn't stutter. Alpha controls responsiveness: lower = smoother but more lag,
   * higher = noisier but faster response.
   */
  private static final double ACCEL_FILTER_ALPHA = 0.10;

  private double filteredAcceleration = 0.0;

  public DriveSwerveDrivetrain(DriveIOHardware driveIO, RobotState robotState) {
    this.driveIO = driveIO;
    this.robotState = robotState;

    // Publish Field2d to SmartDashboard/Shuffleboard for field visualization at comp
    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void periodic() {
    // CTRE's SwerveDriveState.Speeds are robot-relative
    var robotRelativeSpeeds = driveIO.getState().Speeds;

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

  /**
   * Get the magnitude of the robot's linear acceleration (m/s²). Computed from field-relative speed
   * deltas each periodic cycle. Used by the Superstructure to gate conveyor/indexer feeding — FUEL
   * should only be fed to the shooter when the robot is at low acceleration (steady-state driving).
   */
  public double getLinearAccelerationMagnitude() {
    return linearAccelerationMagnitude;
  }

  /**
   * Get the filtered (smoothed) linear acceleration magnitude. Uses an exponential moving average
   * to eliminate frame-to-frame noise. Use this for gating decisions (e.g. when to start feeding
   * FUEL) to avoid stutter from noisy raw acceleration.
   */
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
