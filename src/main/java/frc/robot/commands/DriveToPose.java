// Copyright (c) 2021-2026 Littleton Robotics
// Adapted from Team 254''s AutoAlignToPoseCommand
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private final DriveSwerveDrivetrain drive;
  private final RobotState robotState;
  private final Pose2d targetPose;
  private final ProfiledPIDController driveController;
  private final ProfiledPIDController thetaController;
  private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private static final double FF_MIN_RADIUS = 0.0;
  private static final double FF_MAX_RADIUS = 0.1;

  public DriveToPose(DriveSwerveDrivetrain drive, Pose2d targetPose) {
    this.drive = drive;
    this.robotState = drive.getRobotState();
    this.targetPose = targetPose;
    this.driveController =
        new ProfiledPIDController(
            Constants.DriveConstants.DriveToPose.TRANSLATION_KP,
            Constants.DriveConstants.DriveToPose.TRANSLATION_KI,
            Constants.DriveConstants.DriveToPose.TRANSLATION_KD,
            new TrapezoidProfile.Constraints(
                Constants.DriveConstants.DriveToPose.MAX_VELOCITY, 10.0),
            0.02);
    this.thetaController =
        new ProfiledPIDController(
            Constants.DriveConstants.DriveToPose.ROTATION_KP,
            Constants.DriveConstants.DriveToPose.ROTATION_KI,
            Constants.DriveConstants.DriveToPose.ROTATION_KD,
            new TrapezoidProfile.Constraints(Math.PI * 2, 20.0),
            0.02);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveController.setTolerance(Constants.DriveConstants.DriveToPose.TRANSLATION_TOLERANCE);
    thetaController.setTolerance(Constants.DriveConstants.DriveToPose.ROTATION_TOLERANCE);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = robotState.getLatestFieldToRobot().getValue();
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    ChassisSpeeds currentSpeeds = robotState.getLatestMeasuredFieldRelativeChassisSpeeds();
    Translation2d currentVelocity =
        new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    Translation2d directionToTarget =
        targetPose.getTranslation().minus(currentPose.getTranslation());
    double velocityTowardTarget =
        Math.min(0.0, -currentVelocity.rotateBy(directionToTarget.getAngle().unaryMinus()).getX());
    driveController.reset(currentDistance, velocityTowardTarget);
    thetaController.reset(
        currentPose.getRotation().getRadians(),
        robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);
    Logger.recordOutput("DriveToPose/Initialized", true);
    Logger.recordOutput("DriveToPose/TargetPose", targetPose);
  }

  @Override
  public void execute() {
    Pose2d currentPose = robotState.getLatestFieldToRobot().getValue();
    Logger.recordOutput("DriveToPose/CurrentPose", currentPose);
    Logger.recordOutput("DriveToPose/TargetPose", targetPose);
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    driveErrorAbs = currentDistance;
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - FF_MIN_RADIUS) / (FF_MAX_RADIUS - FF_MIN_RADIUS), 0.0, 1.0);
    Logger.recordOutput("DriveToPose/FFScaler", ffScaler);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) {
      driveVelocityScalar = 0.0;
    }
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) {
      thetaVelocity = 0.0;
    }
    Rotation2d angleToTarget =
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
    Translation2d driveVelocity =
        new Pose2d(Translation2d.kZero, angleToTarget)
            .transformBy(
                new Transform2d(new Translation2d(driveVelocityScalar, 0.0), Rotation2d.kZero))
            .getTranslation();
    drive.applyRequest(
        applyRobotSpeeds.withSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(),
                driveVelocity.getY(),
                thetaVelocity,
                currentPose.getRotation())));
    Logger.recordOutput("DriveToPose/DriveError", driveErrorAbs);
    Logger.recordOutput("DriveToPose/ThetaError", Math.toDegrees(thetaErrorAbs));
    Logger.recordOutput("DriveToPose/DriveVelocityScalar", driveVelocityScalar);
    Logger.recordOutput("DriveToPose/ThetaVelocity", thetaVelocity);
  }

  @Override
  public boolean isFinished() {
    return driveController.atGoal() && thetaController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    Logger.recordOutput("DriveToPose/Initialized", false);
  }
}
