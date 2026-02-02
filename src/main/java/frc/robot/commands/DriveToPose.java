// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

/** Command to drive to a specific pose using PID control. */
public class DriveToPose extends Command {
  private final DriveSwerveDrivetrain drive;
  private final Pose2d targetPose;
  private final ProfiledPIDController driveController;
  private final ProfiledPIDController thetaController;

  private static final double MIN_OUTPUT = 0.05;
  private static final double MAX_OUTPUT = 1.0;

  private double lastError = 0.0;
  private int stableCount = 0;
  private static final int STABLE_THRESHOLD = 10;

  public DriveToPose(DriveSwerveDrivetrain drive, Pose2d targetPose) {
    this.drive = drive;
    this.targetPose = targetPose;
    addRequirements(drive);

    // Create translation PID controller with trapezoidal motion profile
    driveController =
        new ProfiledPIDController(
            Constants.DriveConstants.DriveToPose.TRANSLATION_KP,
            Constants.DriveConstants.DriveToPose.TRANSLATION_KI,
            Constants.DriveConstants.DriveToPose.TRANSLATION_KD,
            new TrapezoidProfile.Constraints(
                Constants.DriveConstants.DriveToPose.MAX_VELOCITY, 10.0),
            0.02);

    // Create rotation PID controller with trapezoidal motion profile
    thetaController =
        new ProfiledPIDController(
            Constants.DriveConstants.DriveToPose.ROTATION_KP,
            Constants.DriveConstants.DriveToPose.ROTATION_KI,
            Constants.DriveConstants.DriveToPose.ROTATION_KD,
            new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 4),
            0.02);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveController.setTolerance(Constants.DriveConstants.DriveToPose.TRANSLATION_TOLERANCE);
    thetaController.setTolerance(Constants.DriveConstants.DriveToPose.ROTATION_TOLERANCE);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drive.getPose();

    // Reset PID controllers
    driveController.reset(0.0);
    thetaController.reset(currentPose.getRotation().getRadians());

    // Set the goal
    double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    driveController.setGoal(distance);
    thetaController.setGoal(targetPose.getRotation().getRadians());

    lastError = distance;
    stableCount = 0;

    Logger.recordOutput("DriveToPose/Initialized", true);
    Logger.recordOutput("DriveToPose/TargetPose", targetPose);
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    Logger.recordOutput("DriveToPose/CurrentPose", currentPose);
    Logger.recordOutput("DriveToPose/TargetPose", targetPose);

    // Calculate error
    Translation2d driveError = targetPose.getTranslation().minus(currentPose.getTranslation());
    double driveErrorAbs = driveError.getNorm();
    double thetaError = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();

    // Calculate feedforward scaler based on distance - reduces oscillation near target
    double ffScaler = Math.min(driveErrorAbs / 0.5, 1.0);
    Logger.recordOutput("DriveToPose/FFScaler", ffScaler);

    // Calculate PID outputs
    double driveOutput = driveController.calculate(0.0);
    double thetaOutput = thetaController.calculate(currentPose.getRotation().getRadians());

    // Add static friction compensation when moving
    if (Math.abs(driveOutput) > 0.01) {
      driveOutput +=
          Math.signum(driveOutput) * Constants.DriveConstants.DriveToPose.STATIC_FRICTION_CONSTANT;
    }

    // Clamp drive output
    driveOutput = Math.max(Math.min(driveOutput, MAX_OUTPUT), -MAX_OUTPUT);
    if (Math.abs(driveOutput) < MIN_OUTPUT && Math.abs(driveErrorAbs) > 0.05) {
      driveOutput = Math.signum(driveOutput) * MIN_OUTPUT;
    }

    // Convert to field-relative velocities
    double xVelocity = driveOutput * driveError.getX() / driveErrorAbs * ffScaler;
    double yVelocity = driveOutput * driveError.getY() / driveErrorAbs * ffScaler;

    // Apply to robot
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity, yVelocity, thetaOutput, currentPose.getRotation());
    drive.runVelocity(speeds);

    Logger.recordOutput("DriveToPose/DriveError", driveErrorAbs);
    Logger.recordOutput("DriveToPose/ThetaError", thetaError);
    Logger.recordOutput("DriveToPose/DriveOutput", driveOutput);
    Logger.recordOutput("DriveToPose/ThetaOutput", thetaOutput);

    // Check if error is stable
    if (Math.abs(driveErrorAbs - lastError) < 0.01) {
      stableCount++;
    } else {
      stableCount = 0;
    }
    lastError = driveErrorAbs;
  }

  @Override
  public boolean isFinished() {
    return driveController.atGoal() && thetaController.atGoal() && stableCount > STABLE_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    Logger.recordOutput("DriveToPose/Initialized", false);
  }
}
