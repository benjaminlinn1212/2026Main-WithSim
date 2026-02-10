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

public class VisionIOHardwareLimelight implements VisionIO {
  private final NetworkTable frontTable =
      NetworkTableInstance.getDefault().getTable(Constants.Vision.FRONT_LIMELIGHT_NAME);
  private final NetworkTable backTable =
      NetworkTableInstance.getDefault().getTable(Constants.Vision.BACK_LIMELIGHT_NAME);
  private final NetworkTable turretTable =
      NetworkTableInstance.getDefault().getTable(Constants.Vision.TURRET_LIMELIGHT_NAME);

  private final RobotState robotState;

  public VisionIOHardwareLimelight(RobotState robotState) {
    this.robotState = robotState;
    configureDrivetrainCameras();
  }

  /** Set camerapose_robotspace_set for drivetrain-mounted cameras (one-time at startup). */
  private void configureDrivetrainCameras() {
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
  }

  /**
   * Send robot/turret orientation to all Limelights for MegaTag2.
   *
   * <p>Drivetrain cameras receive the robot's field heading. The turret camera receives the
   * turret's field heading (robot heading + turret angle) so the Limelight can solve for the
   * camera's field pose.
   */
  private void updateOrientations() {
    var robotHeading = robotState.getLatestFieldToRobot().getValue().getRotation();
    double robotOmegaDegPerSec =
        Units.radiansToDegrees(
            robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);

    // Drivetrain cameras — send robot heading
    LimelightHelpers.SetRobotOrientation(
        Constants.Vision.FRONT_LIMELIGHT_NAME,
        robotHeading.getDegrees(),
        robotOmegaDegPerSec,
        0,
        0,
        0,
        0);
    LimelightHelpers.SetRobotOrientation(
        Constants.Vision.BACK_LIMELIGHT_NAME,
        robotHeading.getDegrees(),
        robotOmegaDegPerSec,
        0,
        0,
        0,
        0);

    // Turret camera — send turret's field-relative heading
    var turretFieldHeading = robotHeading.rotateBy(robotState.getLatestRobotToTurret().getValue());
    double turretOmegaDegPerSec =
        Units.radiansToDegrees(
            robotState.getLatestTurretAngularVelocity()
                + robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);

    LimelightHelpers.SetRobotOrientation(
        Constants.Vision.TURRET_LIMELIGHT_NAME,
        turretFieldHeading.getDegrees(),
        turretOmegaDegPerSec,
        0,
        0,
        0,
        0);
  }

  @Override
  public void readInputs(VisionIOInputs inputs) {
    updateOrientations();

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
      inputs.frontCameraMegatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
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
      inputs.backCameraMegatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
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
      inputs.turretCameraMegatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
    }
  }
}
