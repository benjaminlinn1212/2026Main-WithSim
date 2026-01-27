// Copyright (c) 2021-2026 Littleton Robotics
// Adapted from Team 254's 2024 code
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  // Vision processing constants
  private static final double MIN_TAG_AREA_MULTI_TAG = 0.1;
  private static final double MIN_TAG_AREA_LARGE = 0.8;
  private static final double MAX_POSE_DIFFERENCE_CLOSE = 0.3;
  private static final double MAX_POSE_DIFFERENCE_FAR = 0.5;
  private static final double XY_STD_DEV_MULTI_TAG_LARGE_AREA = 0.2;
  private static final double XY_STD_DEV_LARGE_AREA = 0.5;
  private static final double XY_STD_DEV_FAR = 1.0;
  private static final double XY_STD_DEV_MULTI_TAG = 1.5;
  private static final double XY_STD_DEV_DEFAULT = 0.5;
  private static final double ROTATION_STD_DEV_DEGREES = 12.0;

  private final VisionIO io;
  private final RobotState state;
  private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

  private double lastProcessedFrontTimestamp = 0.0;
  private double lastProcessedBackTimestamp = 0.0;
  private double lastProcessedTurretTimestamp = 0.0;

  private final Consumer<VisionFieldPoseEstimate> poseEstimateConsumer;

  public VisionSubsystem(
      VisionIO io, RobotState state, Consumer<VisionFieldPoseEstimate> poseEstimateConsumer) {
    this.io = io;
    this.state = state;
    this.poseEstimateConsumer = poseEstimateConsumer;
  }

  @Override
  public void periodic() {
    io.readInputs(inputs);
    // Manual logging of important vision data
    Logger.recordOutput("Vision/Front/SeesTarget", inputs.frontCameraSeesTarget);
    Logger.recordOutput("Vision/Back/SeesTarget", inputs.backCameraSeesTarget);
    Logger.recordOutput("Vision/Turret/SeesTarget", inputs.turretCameraSeesTarget);

    // Process each camera
    if (inputs.frontCameraSeesTarget) {
      updateVision(
          inputs.frontCameraFiducialObservations,
          inputs.frontCameraMegatagPoseEstimate,
          inputs.frontCameraMegatag2PoseEstimate,
          false,
          "Vision/Front/");
    }

    if (inputs.backCameraSeesTarget) {
      updateVision(
          inputs.backCameraFiducialObservations,
          inputs.backCameraMegatagPoseEstimate,
          inputs.backCameraMegatag2PoseEstimate,
          false,
          "Vision/Back/");
    }

    if (inputs.turretCameraSeesTarget) {
      updateVision(
          inputs.turretCameraFiducialObservations,
          inputs.turretCameraMegatagPoseEstimate,
          inputs.turretCameraMegatag2PoseEstimate,
          true,
          "Vision/Turret/");
    }
  }

  private void updateVision(
      FiducialObservation[] fiducialObservations,
      MegatagPoseEstimate megatagPoseEstimate,
      MegatagPoseEstimate megatag2PoseEstimate,
      boolean isTurretCamera,
      String logPreface) {

    if (megatagPoseEstimate == null) {
      return;
    }

    var updateTimestamp = megatagPoseEstimate.timestampSeconds;
    boolean alreadyProcessed = false;

    if (isTurretCamera) {
      alreadyProcessed = lastProcessedTurretTimestamp == updateTimestamp;
      lastProcessedTurretTimestamp = updateTimestamp;
    } else if (logPreface.contains("Front")) {
      alreadyProcessed = lastProcessedFrontTimestamp == updateTimestamp;
      lastProcessedFrontTimestamp = updateTimestamp;
    } else {
      alreadyProcessed = lastProcessedBackTimestamp == updateTimestamp;
      lastProcessedBackTimestamp = updateTimestamp;
    }

    if (alreadyProcessed) {
      return;
    }

    // Process MegaTag2 (preferred)
    Optional<VisionFieldPoseEstimate> megatag2Estimate =
        processMegatag2PoseEstimate(megatag2PoseEstimate, isTurretCamera, logPreface);

    if (megatag2Estimate.isPresent()) {
      Logger.recordOutput(
          logPreface + "Megatag2Estimate", megatag2Estimate.get().getVisionRobotPoseMeters());
      poseEstimateConsumer.accept(megatag2Estimate.get());
    } else {
      // Fall back to MegaTag if MegaTag2 is not available
      Optional<VisionFieldPoseEstimate> megatagEstimate =
          processMegatagPoseEstimate(megatagPoseEstimate, isTurretCamera, logPreface);

      if (megatagEstimate.isPresent()) {
        Logger.recordOutput(
            logPreface + "MegatagEstimate", megatagEstimate.get().getVisionRobotPoseMeters());
        poseEstimateConsumer.accept(megatagEstimate.get());
      }
    }
  }

  private Optional<VisionFieldPoseEstimate> processMegatag2PoseEstimate(
      MegatagPoseEstimate poseEstimate, boolean isTurretCamera, String logPreface) {

    if (poseEstimate == null || poseEstimate.fieldToCamera == null) {
      return Optional.empty();
    }

    var loggedFieldToRobot = state.getFieldToRobot(poseEstimate.timestampSeconds);
    if (loggedFieldToRobot.isEmpty()) {
      return Optional.empty();
    }

    var maybeFieldToRobotEstimate = getFieldToRobotEstimate(poseEstimate, isTurretCamera);
    if (maybeFieldToRobotEstimate.isEmpty()) {
      return Optional.empty();
    }

    var fieldToRobotEstimate = maybeFieldToRobotEstimate.get();

    // Calculate distance from current pose to vision estimated pose
    double poseDifference =
        fieldToRobotEstimate
            .getTranslation()
            .getDistance(loggedFieldToRobot.get().getTranslation());

    double xyStds = XY_STD_DEV_DEFAULT;

    if (poseEstimate.fiducialIds.length > 0) {
      // Multiple targets detected
      if (poseEstimate.fiducialIds.length >= 2
          && poseEstimate.avgTagArea > MIN_TAG_AREA_MULTI_TAG) {
        xyStds = XY_STD_DEV_MULTI_TAG_LARGE_AREA;
      }
      // Large area and close to estimated pose
      else if (poseEstimate.avgTagArea > MIN_TAG_AREA_LARGE
          && poseDifference < MAX_POSE_DIFFERENCE_FAR) {
        xyStds = XY_STD_DEV_LARGE_AREA;
      }
      // Farther away and estimated pose is close
      else if (poseEstimate.avgTagArea > MIN_TAG_AREA_MULTI_TAG
          && poseDifference < MAX_POSE_DIFFERENCE_CLOSE) {
        xyStds = XY_STD_DEV_FAR;
      } else if (poseEstimate.fiducialIds.length > 1) {
        xyStds = XY_STD_DEV_MULTI_TAG;
      } else {
        return Optional.empty();
      }

      Matrix<N3, N1> visionMeasurementStdDevs =
          VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(ROTATION_STD_DEV_DEGREES));

      Logger.recordOutput(logPreface + "StdDevs", visionMeasurementStdDevs.getData());
      Logger.recordOutput(logPreface + "PoseDifference", poseDifference);

      return Optional.of(
          new VisionFieldPoseEstimate(
              fieldToRobotEstimate, poseEstimate.timestampSeconds, visionMeasurementStdDevs));
    }

    return Optional.empty();
  }

  private Optional<VisionFieldPoseEstimate> processMegatagPoseEstimate(
      MegatagPoseEstimate poseEstimate, boolean isTurretCamera, String logPreface) {

    if (poseEstimate == null || poseEstimate.fieldToCamera == null) {
      return Optional.empty();
    }

    var loggedFieldToRobot = state.getFieldToRobot(poseEstimate.timestampSeconds);
    if (loggedFieldToRobot.isEmpty()) {
      return Optional.empty();
    }

    var maybeFieldToRobotEstimate = getFieldToRobotEstimate(poseEstimate, isTurretCamera);
    if (maybeFieldToRobotEstimate.isEmpty()) {
      return Optional.empty();
    }

    var fieldToRobotEstimate = maybeFieldToRobotEstimate.get();

    double poseDifference =
        fieldToRobotEstimate
            .getTranslation()
            .getDistance(loggedFieldToRobot.get().getTranslation());

    if (poseEstimate.fiducialIds.length > 0) {
      double xyStds = 1.0;
      double degStds = 12;

      // Multiple targets detected
      if (poseEstimate.fiducialIds.length >= 2) {
        xyStds = 0.5;
        degStds = 6;
      }
      // 1 target with large area and close to estimated pose
      else if (poseEstimate.avgTagArea > 0.8 && poseDifference < 0.5) {
        xyStds = 1.0;
        degStds = 12;
      }
      // 1 target farther away and estimated pose is close
      else if (poseEstimate.avgTagArea > 0.1 && poseDifference < 0.3) {
        xyStds = 2.0;
        degStds = 30;
      } else {
        return Optional.empty();
      }

      Matrix<N3, N1> visionMeasurementStdDevs =
          VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds));

      return Optional.of(
          new VisionFieldPoseEstimate(
              fieldToRobotEstimate, poseEstimate.timestampSeconds, visionMeasurementStdDevs));
    }

    return Optional.empty();
  }

  /**
   * Calculate field-to-robot pose from vision data. For turret cameras, this accounts for: 1.
   * Field-to-camera (from Limelight MegaTag) 2. Camera-to-turret (camera offset from turret center)
   * 3. Turret-to-robot (turret rotation and position on robot)
   */
  private Optional<Pose2d> getFieldToRobotEstimate(
      MegatagPoseEstimate poseEstimate, boolean isTurretCamera) {

    var fieldToCamera = poseEstimate.fieldToCamera;
    if (fieldToCamera.getX() == 0.0) {
      return Optional.empty();
    }

    if (isTurretCamera) {
      // Get turret rotation at the timestamp of this vision measurement
      var robotToTurretObservation = state.getRobotToTurret(poseEstimate.timestampSeconds);
      if (robotToTurretObservation.isEmpty()) {
        return Optional.empty();
      }

      // Transform chain: Field -> Camera -> Turret -> Robot
      // 1. Get camera-to-turret transform (inverse of turret-to-camera)
      var turretToCameraTransform = state.getTurretToCamera(true);
      var cameraToTurretTransform = turretToCameraTransform.inverse();

      // 2. Get field-to-turret
      var fieldToTurretPose = fieldToCamera.plus(cameraToTurretTransform);

      // 3. Account for turret rotation to get field-to-robot
      // The turret is at turret_to_robot_center offset from robot center
      var turretToRobot =
          new Transform2d(
              Constants.Vision.TURRET_TO_ROBOT_CENTER
                  .unaryMinus(), // Negative because we go turret->robot
              robotToTurretObservation
                  .get()
                  .unaryMinus()); // Negative to get turret->robot rotation

      var fieldToRobotEstimate = fieldToTurretPose.plus(turretToRobot);

      return Optional.of(fieldToRobotEstimate);
    } else {
      // Drivetrain camera - camera is fixed to robot body
      // Field -> Camera is directly Field -> Robot
      // The limelight already accounts for the camera position via camerapose_robotspace_set
      return Optional.of(fieldToCamera);
    }
  }
}
