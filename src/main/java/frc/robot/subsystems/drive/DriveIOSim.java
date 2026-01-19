// Copyright (c) 2021-2026 Littleton Robotics
// Adapted from Team 254's 2025 code
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.generated.TunerConstants;
import frc.robot.util.sim.MapleSimSwerveDrivetrain;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/**
 * DriveIOSim extends DriveIOHardware to provide simulation-specific functionality. This matches
 * Team 254's approach of extending hardware IO and injecting Maple-Sim pose.
 */
public class DriveIOSim extends DriveIOHardware {

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier simNotifier = null;
  private double lastSimTime;
  public MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

  Consumer<SwerveDriveState> simTelemetryConsumer =
      swerveDriveState -> {
        if (Constants.DriveConstants.USE_MAPLE_SIM && mapleSimSwerveDrivetrain != null) {
          // Inject Maple-Sim's pose into the telemetry (matches 254)
          swerveDriveState.Pose =
              mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
        }
        // Update RobotState with the pose
        if (robotState != null) {
          robotState.addFieldToRobot(swerveDriveState.Pose);
        }
        telemetryConsumer_.accept(swerveDriveState);
      };

  public DriveIOSim(
      RobotState robotState,
      SwerveDrivetrainConstants driveTrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(robotState, driveTrainConstants, modules);

    // Rewrite the telemetry consumer with a consumer for sim
    registerTelemetry(simTelemetryConsumer);
    startSimThread();
  }

  @SuppressWarnings("unchecked")
  public void startSimThread() {
    if (Constants.DriveConstants.USE_MAPLE_SIM) {
      mapleSimSwerveDrivetrain =
          new MapleSimSwerveDrivetrain(
              Units.Seconds.of(kSimLoopPeriod),
              Units.Kilograms.of(Constants.DriveConstants.ROBOT_WEIGHT_KILOGRAMS),
              Units.Inches.of(Constants.DriveConstants.BUMPER_WIDTH_INCHES),
              Units.Inches.of(Constants.DriveConstants.BUMPER_LENGTH_INCHES),
              DCMotor.getKrakenX60(Constants.DriveConstants.DRIVE_MOTOR_COUNT),
              DCMotor.getKrakenX60(Constants.DriveConstants.DRIVE_MOTOR_COUNT),
              Constants.DriveConstants.WHEEL_COEFFICIENT_OF_FRICTION,
              getModuleLocations(),
              getPigeon2(),
              getModules(),
              TunerConstants.FrontLeft,
              TunerConstants.FrontRight,
              TunerConstants.BackLeft,
              TunerConstants.BackRight);
      simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
    } else {
      lastSimTime = Utils.getCurrentTimeSeconds();
      simNotifier =
          new Notifier(
              () -> {
                final double currentTime = Utils.getCurrentTimeSeconds();
                double deltaTime = currentTime - lastSimTime;
                lastSimTime = currentTime;
                updateSimState(deltaTime, RobotController.getBatteryVoltage());
              });
    }
    simNotifier.startPeriodic(kSimLoopPeriod);
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    if (Constants.DriveConstants.USE_MAPLE_SIM && mapleSimSwerveDrivetrain != null) {
      mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
      Timer.delay(0.05);
    }
    super.resetOdometry(pose);
  }

  public MapleSimSwerveDrivetrain getMapleSimDrive() {
    return mapleSimSwerveDrivetrain;
  }

  public Pose2d getSimulatedPose() {
    if (Constants.DriveConstants.USE_MAPLE_SIM && mapleSimSwerveDrivetrain != null) {
      return mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
    }
    return getState().Pose;
  }

  public void logSimulatedPose() {
    Logger.recordOutput("Drive/Viz/SimPose", getSimulatedPose());
  }
}
