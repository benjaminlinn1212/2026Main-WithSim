// Copyright (c) 2021-2026 Littleton Robotics
// Adapted from Team 254's 2025 code
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotState;
import java.util.function.Consumer;

/**
 * DriveIOHardware extends CTRE's SwerveDrivetrain class to provide hardware-level control for the
 * swerve drive system. This matches Team 254's approach of using SwerveDrivetrain directly.
 */
public class DriveIOHardware extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {

  protected RobotState robotState;
  protected Consumer<SwerveDriveState> telemetryConsumer_ =
      swerveDriveState -> {
        // Update RobotState with pose from telemetry
        if (robotState != null) {
          robotState.addFieldToRobot(swerveDriveState.Pose);
        }
      };

  public DriveIOHardware(
      RobotState robotState,
      SwerveDrivetrainConstants driveTrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(TalonFX::new, TalonFX::new, CANcoder::new, driveTrainConstants, 250.0, modules);

    this.robotState = robotState;
    this.getOdometryThread().setThreadPriority(99);

    registerTelemetry(telemetryConsumer_);
  }

  public void resetOdometry(Pose2d pose) {
    super.resetPose(pose);
  }
}
