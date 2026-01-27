// Copyright (c) 2021-2026 Littleton Robotics
// Adapted from Team 254's 2024 code
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotState;
import java.util.concurrent.atomic.AtomicReference;

public class VisionIOHardwareLimelight implements VisionIO {
  NetworkTable frontTable =
      NetworkTableInstance.getDefault().getTable(Constants.Vision.FRONT_LIMELIGHT_NAME);
  NetworkTable backTable =
      NetworkTableInstance.getDefault().getTable(Constants.Vision.BACK_LIMELIGHT_NAME);
  NetworkTable turretTable =
      NetworkTableInstance.getDefault().getTable(Constants.Vision.TURRET_LIMELIGHT_NAME);

  RobotState robotState;
  AtomicReference<VisionIOInputs> latestInputs = new AtomicReference<>(new VisionIOInputs());

  public VisionIOHardwareLimelight(RobotState robotState) {
    this.robotState = robotState;
    setLLSettings();
  }

  private void setLLSettings() {
    // Configure camera poses relative to robot
    double[] frontCamerapose = {
      Constants.Vision.FRONT_CAMERA_TO_ROBOT.getX(),
      Constants.Vision.FRONT_CAMERA_TO_ROBOT.getY(),
      Constants.Vision.FRONT_CAMERA_TO_ROBOT.getZ(),
      Units.radiansToDegrees(Constants.Vision.FRONT_CAMERA_TO_ROBOT.getRotation().getX()),
      Units.radiansToDegrees(Constants.Vision.FRONT_CAMERA_TO_ROBOT.getRotation().getY()),
      Units.radiansToDegrees(Constants.Vision.FRONT_CAMERA_TO_ROBOT.getRotation().getZ())
    };
    frontTable.getEntry("camerapose_robotspace_set").setDoubleArray(frontCamerapose);

    double[] backCamerapose = {
      Constants.Vision.BACK_CAMERA_TO_ROBOT.getX(),
      Constants.Vision.BACK_CAMERA_TO_ROBOT.getY(),
      Constants.Vision.BACK_CAMERA_TO_ROBOT.getZ(),
      Units.radiansToDegrees(Constants.Vision.BACK_CAMERA_TO_ROBOT.getRotation().getX()),
      Units.radiansToDegrees(Constants.Vision.BACK_CAMERA_TO_ROBOT.getRotation().getY()),
      Units.radiansToDegrees(Constants.Vision.BACK_CAMERA_TO_ROBOT.getRotation().getZ())
    };
    backTable.getEntry("camerapose_robotspace_set").setDoubleArray(backCamerapose);

    // Turret camera orientation will be updated dynamically
    updateTurretCameraOrientation();
  }

  private void updateTurretCameraOrientation() {
    // Update robot orientation for non-turret cameras
    var gyroAngle = robotState.getLatestFieldToRobot().getValue().getRotation();
    var gyroAngularVelocity =
        Units.radiansToDegrees(
            robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);

    LimelightHelpers.SetRobotOrientation(
        Constants.Vision.FRONT_LIMELIGHT_NAME,
        gyroAngle.getDegrees(),
        gyroAngularVelocity,
        0,
        0,
        0,
        0);
    LimelightHelpers.SetRobotOrientation(
        Constants.Vision.BACK_LIMELIGHT_NAME,
        gyroAngle.getDegrees(),
        gyroAngularVelocity,
        0,
        0,
        0,
        0);

    // For turret camera, send field-to-turret orientation
    var fieldToTurretRotation =
        robotState
            .getLatestFieldToRobot()
            .getValue()
            .getRotation()
            .rotateBy(robotState.getLatestRobotToTurret().getValue());
    var fieldToTurretVelocity =
        Units.radiansToDegrees(
            robotState.getLatestTurretAngularVelocity()
                + robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);

    LimelightHelpers.SetRobotOrientation(
        Constants.Vision.TURRET_LIMELIGHT_NAME,
        fieldToTurretRotation.getDegrees(),
        fieldToTurretVelocity,
        0,
        0,
        0,
        0);
  }

  @Override
  public void readInputs(VisionIOInputs inputs) {
    // Update camera orientations before reading
    updateTurretCameraOrientation();

    // Front camera
    inputs.frontCameraSeesTarget = frontTable.getEntry("tv").getDouble(0) == 1.0;
    if (inputs.frontCameraSeesTarget) {
      var megatag =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag1(
              Constants.Vision.FRONT_LIMELIGHT_NAME);
      var megatag2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
              Constants.Vision.FRONT_LIMELIGHT_NAME);
      inputs.frontCameraMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
      inputs.frontCameraMegatagCount = megatag.tagCount;
      inputs.frontCameraMegatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
      inputs.frontCameraFiducialObservations =
          FiducialObservation.fromLimelight(megatag.rawFiducials);
    }

    // Back camera
    inputs.backCameraSeesTarget = backTable.getEntry("tv").getDouble(0) == 1.0;
    if (inputs.backCameraSeesTarget) {
      var megatag =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag1(
              Constants.Vision.BACK_LIMELIGHT_NAME);
      var megatag2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
              Constants.Vision.BACK_LIMELIGHT_NAME);
      inputs.backCameraMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
      inputs.backCameraMegatagCount = megatag.tagCount;
      inputs.backCameraMegatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
      inputs.backCameraFiducialObservations =
          FiducialObservation.fromLimelight(megatag.rawFiducials);
    }

    // Turret camera
    inputs.turretCameraSeesTarget = turretTable.getEntry("tv").getDouble(0) == 1.0;
    if (inputs.turretCameraSeesTarget) {
      var megatag =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag1(
              Constants.Vision.TURRET_LIMELIGHT_NAME);
      var megatag2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
              Constants.Vision.TURRET_LIMELIGHT_NAME);
      inputs.turretCameraMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
      inputs.turretCameraMegatagCount = megatag.tagCount;
      inputs.turretCameraMegatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
      inputs.turretCameraFiducialObservations =
          FiducialObservation.fromLimelight(megatag.rawFiducials);
    }
  }

  @Override
  public void pollNetworkTables() {
    VisionIOInputs inputs = new VisionIOInputs();

    // Update camera orientations
    updateTurretCameraOrientation();

    // Front camera
    inputs.frontCameraSeesTarget = frontTable.getEntry("tv").getDouble(0) == 1.0;
    if (inputs.frontCameraSeesTarget) {
      var megatag =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag1(
              Constants.Vision.FRONT_LIMELIGHT_NAME);
      var megatag2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
              Constants.Vision.FRONT_LIMELIGHT_NAME);
      inputs.frontCameraMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
      inputs.frontCameraMegatagCount = megatag.tagCount;
      inputs.frontCameraMegatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
      inputs.frontCameraFiducialObservations =
          FiducialObservation.fromLimelight(megatag.rawFiducials);
    }

    // Back camera
    inputs.backCameraSeesTarget = backTable.getEntry("tv").getDouble(0) == 1.0;
    if (inputs.backCameraSeesTarget) {
      var megatag =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag1(
              Constants.Vision.BACK_LIMELIGHT_NAME);
      var megatag2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
              Constants.Vision.BACK_LIMELIGHT_NAME);
      inputs.backCameraMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
      inputs.backCameraMegatagCount = megatag.tagCount;
      inputs.backCameraMegatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
      inputs.backCameraFiducialObservations =
          FiducialObservation.fromLimelight(megatag.rawFiducials);
    }

    // Turret camera
    inputs.turretCameraSeesTarget = turretTable.getEntry("tv").getDouble(0) == 1.0;
    if (inputs.turretCameraSeesTarget) {
      var megatag =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag1(
              Constants.Vision.TURRET_LIMELIGHT_NAME);
      var megatag2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
              Constants.Vision.TURRET_LIMELIGHT_NAME);
      inputs.turretCameraMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
      inputs.turretCameraMegatagCount = megatag.tagCount;
      inputs.turretCameraMegatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
      inputs.turretCameraFiducialObservations =
          FiducialObservation.fromLimelight(megatag.rawFiducials);
    }

    latestInputs.set(inputs);
  }
}
