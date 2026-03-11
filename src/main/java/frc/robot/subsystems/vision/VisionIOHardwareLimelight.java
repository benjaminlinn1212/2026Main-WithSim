// Copyright (c) 2021-2026 Littleton Robotics
// Adapted from Team 254's 2024 code
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotState;

public class VisionIOHardwareLimelight implements VisionIO {
  private final NetworkTable frontTable =
      NetworkTableInstance.getDefault().getTable(Constants.Vision.FRONT_LIMELIGHT_NAME);
  private final NetworkTable turretTable =
      NetworkTableInstance.getDefault().getTable(Constants.Vision.TURRET_LIMELIGHT_NAME);

  // Cache NT entry objects to avoid repeated table lookups every cycle
  private final NetworkTableEntry frontTvEntry = frontTable.getEntry("tv");
  private final NetworkTableEntry turretTvEntry = turretTable.getEntry("tv");

  private final RobotState robotState;

  public VisionIOHardwareLimelight(RobotState robotState) {
    this.robotState = robotState;
    configureDrivetrainCameras();
    configureTurretCamera();

    // Drivetrain camera: Mode 0 — always use external heading from SetRobotOrientation().
    // This camera is rigidly mounted to the chassis, so the robot gyro heading is
    // perfectly time-aligned (no moving joint).
    LimelightHelpers.SetIMUMode(Constants.Vision.FRONT_LIMELIGHT_NAME, 0);

    // Turret camera: Mode 0 — use external heading only (internal IMU disabled).
    // The Pigeon2 gyro is far more accurate than the LL onboard IMU, so we keep
    // mode 0. To compensate for the heading-vs-image timing mismatch,
    // updateOrientations() estimates the capture timestamp and interpolates the
    // turret angle back to that instant.
    LimelightHelpers.SetIMUMode(Constants.Vision.TURRET_LIMELIGHT_NAME, 0);
  }

  /** Set camerapose_robotspace_set for drivetrain-mounted cameras (one-time at startup). */
  private void configureDrivetrainCameras() {
    // camerapose_robotspace_set uses WPILib coordinates (x=forward, y=left, z=up) via NT API,
    // NOT the web UI's "forward/right/up" convention. No axis negation needed.
    double[] frontCamerapose = {
      Constants.Vision.RIGHT_CAMERA_TO_ROBOT.getX(),
      Constants.Vision.RIGHT_CAMERA_TO_ROBOT.getY(),
      Constants.Vision.RIGHT_CAMERA_TO_ROBOT.getZ(),
      Units.radiansToDegrees(Constants.Vision.RIGHT_CAMERA_TO_ROBOT.getRotation().getX()),
      Units.radiansToDegrees(Constants.Vision.RIGHT_CAMERA_TO_ROBOT.getRotation().getY()),
      Units.radiansToDegrees(Constants.Vision.RIGHT_CAMERA_TO_ROBOT.getRotation().getZ())
    };
    frontTable.getEntry("camerapose_robotspace_set").setDoubleArray(frontCamerapose);
  }

  /**
   * Set camerapose_robotspace_set for turret camera. Uses TURRET_TO_CAMERA since we send the
   * turret's heading via SetRobotOrientation, making MT2 return turret-center field pose.
   */
  private void configureTurretCamera() {
    // camerapose_robotspace_set uses WPILib coordinates (x=forward, y=left, z=up) via NT API,
    // NOT the web UI's "forward/right/up" convention. No axis negation needed.
    double[] turretCamerapose = {
      Constants.Vision.TURRET_TO_CAMERA.getX(),
      Constants.Vision.TURRET_TO_CAMERA.getY(),
      Constants.Vision.TURRET_TO_CAMERA.getZ(),
      Units.radiansToDegrees(Constants.Vision.TURRET_TO_CAMERA.getRotation().getX()),
      Units.radiansToDegrees(Constants.Vision.TURRET_TO_CAMERA.getRotation().getY()),
      Units.radiansToDegrees(Constants.Vision.TURRET_TO_CAMERA.getRotation().getZ())
    };
    turretTable.getEntry("camerapose_robotspace_set").setDoubleArray(turretCamerapose);
  }

  /**
   * Send heading to all Limelights for MT2. Drivetrain cameras get robot heading. Turret camera
   * gets time-corrected turret field heading (robot heading + turret angle at estimated capture
   * time).
   */
  private void updateOrientations() {
    var latestEntry = robotState.getLatestFieldToRobot();
    if (latestEntry == null) {
      // No pose data yet (first cycle before odometry runs) — skip to avoid NPE
      return;
    }
    var robotHeading = latestEntry.getValue().getRotation();
    double robotOmegaDegPerSec =
        Units.radiansToDegrees(
            robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);

    // Drivetrain camera — send robot heading (gyro is time-aligned, no correction needed)
    LimelightHelpers.SetRobotOrientation(
        Constants.Vision.FRONT_LIMELIGHT_NAME,
        robotHeading.getDegrees(),
        robotOmegaDegPerSec,
        0,
        0,
        0,
        0);

    // Turret camera — estimate the turret heading at the image capture instant.
    // Capture + pipeline latency (ms → s) tells us how far back the image was taken.
    double captureLatencyS =
        (LimelightHelpers.getLatency_Capture(Constants.Vision.TURRET_LIMELIGHT_NAME)
                + LimelightHelpers.getLatency_Pipeline(Constants.Vision.TURRET_LIMELIGHT_NAME))
            / 1000.0;
    double estimatedCaptureTimestamp = Timer.getFPGATimestamp() - captureLatencyS;

    // Look up the interpolated turret angle at the estimated capture time
    double turretAngleRad =
        robotState
            .getInterpolatedRobotToTurret(estimatedCaptureTimestamp)
            .orElse(robotState.getLatestRobotToTurret().getValue());
    double turretFieldHeadingDeg = robotHeading.getDegrees() + Math.toDegrees(turretAngleRad);
    double turretOmegaDegPerSec =
        Units.radiansToDegrees(
            robotState.getLatestTurretAngularVelocity()
                + robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);

    LimelightHelpers.SetRobotOrientation(
        Constants.Vision.TURRET_LIMELIGHT_NAME,
        turretFieldHeadingDeg,
        turretOmegaDegPerSec,
        0,
        0,
        0,
        0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    updateOrientations();

    // Front camera
    inputs.frontCameraSeesTarget = frontTvEntry.getDouble(0) == 1.0;
    if (inputs.frontCameraSeesTarget) {
      var megatag =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.FRONT_LIMELIGHT_NAME);
      var megatag2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
              Constants.Vision.FRONT_LIMELIGHT_NAME);
      inputs.frontCameraMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
      inputs.frontCameraMegatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
    }

    // Turret camera
    inputs.turretCameraSeesTarget = turretTvEntry.getDouble(0) == 1.0;
    if (inputs.turretCameraSeesTarget) {
      var megatag =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.TURRET_LIMELIGHT_NAME);
      var megatag2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
              Constants.Vision.TURRET_LIMELIGHT_NAME);
      inputs.turretCameraMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
      inputs.turretCameraMegatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
    }
  }
}
