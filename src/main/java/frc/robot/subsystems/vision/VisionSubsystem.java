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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private static final double MIN_TAG_AREA_MULTI_TAG = 0.1;
  private static final double MIN_TAG_AREA_LARGE = 0.8;
  private static final double MAX_POSE_DIFFERENCE_CLOSE = 0.3;
  private static final double MAX_POSE_DIFFERENCE_FAR = 0.5;
  private static final double XY_STD_DEV_MULTI_TAG_LARGE_AREA = 0.2;
  private static final double XY_STD_DEV_LARGE_AREA = 0.5;
  private static final double XY_STD_DEV_FAR = 1.0;
  private static final double XY_STD_DEV_MULTI_TAG = 1.5;
  private static final double XY_STD_DEV_DEFAULT = 0.5;
  // MegaTag2 uses the gyro heading we feed it, so the rotation in the result is just our own
  // gyro echoed back. Use a massive std dev so the pose estimator ignores MT2 rotation entirely.
  private static final double MEGATAG2_ROTATION_STD_DEV = 999999.0;

  // Turret camera std dev multiplier — extra camera->turret->robot transform amplifies noise.
  // Also compensates for turret heading timing mismatch (~20-50ms stale).
  private static final double TURRET_CAMERA_STD_DEV_MULTIPLIER = 6.0;

  private final VisionIO io;
  private final RobotState state;
  private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

  private double lastProcessedFrontTimestamp = 0.0;
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
    io.updateInputs(inputs);
    Logger.recordOutput("Vision/Front/SeesTarget", inputs.frontCameraSeesTarget);
    Logger.recordOutput("Vision/Turret/SeesTarget", inputs.turretCameraSeesTarget);

    if (inputs.frontCameraSeesTarget) {
      updateVision(
          inputs.frontCameraMegatagPoseEstimate,
          inputs.frontCameraMegatag2PoseEstimate,
          false,
          "Vision/Front/");
    }

    if (inputs.turretCameraSeesTarget) {
      // Turret camera: camerapose_robotspace_set is configured to TURRET_TO_CAMERA, so
      // MT2's botpose returns the turret center's field pose.  getFieldToRobotEstimate()
      // then applies only turret→robot for MT2, or camera→turret→robot for MT1.
      updateVision(
          inputs.turretCameraMegatagPoseEstimate,
          inputs.turretCameraMegatag2PoseEstimate,
          true,
          "Vision/Turret/");
    }
  }

  private void updateVision(
      MegatagPoseEstimate megatagPoseEstimate,
      MegatagPoseEstimate megatag2PoseEstimate,
      boolean isTurretCamera,
      String logPreface) {

    // Use whichever estimate is available for dedup timestamp (prefer MT2)
    MegatagPoseEstimate primaryEstimate =
        (megatag2PoseEstimate != null) ? megatag2PoseEstimate : megatagPoseEstimate;

    if (primaryEstimate == null) {
      return;
    }

    var updateTimestamp = primaryEstimate.timestampSeconds;
    boolean alreadyProcessed = false;

    if (isTurretCamera) {
      alreadyProcessed = lastProcessedTurretTimestamp == updateTimestamp;
      lastProcessedTurretTimestamp = updateTimestamp;
    } else {
      alreadyProcessed = lastProcessedFrontTimestamp == updateTimestamp;
      lastProcessedFrontTimestamp = updateTimestamp;
    }

    if (alreadyProcessed) {
      return;
    }

    // Reject turret camera frames captured while the turret or chassis was spinning
    // too fast — the heading sent via SetRobotOrientation can't keep up with rapid
    // rotation, producing large pose errors.  254 does the same in shouldUsePinhole().
    if (isTurretCamera && shouldRejectTurretVision(updateTimestamp, logPreface)) {
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

    if (poseEstimate == null || poseEstimate.fieldPose == null) {
      Logger.recordOutput(logPreface + "RejectReason", "null estimate or pose");
      return Optional.empty();
    }

    var loggedFieldToRobot = state.getFieldToRobot(poseEstimate.timestampSeconds);
    if (loggedFieldToRobot.isEmpty()) {
      Logger.recordOutput(logPreface + "RejectReason", "no robot pose at timestamp");
      return Optional.empty();
    }

    var maybeFieldToRobotEstimate = getFieldToRobotEstimate(poseEstimate, isTurretCamera, true);
    if (maybeFieldToRobotEstimate.isEmpty()) {
      Logger.recordOutput(logPreface + "RejectReason", "transform chain failed");
      return Optional.empty();
    }

    var fieldToRobotEstimate = maybeFieldToRobotEstimate.get();

    // Calculate distance from current pose to vision estimated pose
    double poseDifference =
        fieldToRobotEstimate
            .getTranslation()
            .getDistance(loggedFieldToRobot.get().getTranslation());

    Logger.recordOutput(logPreface + "PoseDifference", poseDifference);
    Logger.recordOutput(logPreface + "TagCount", poseEstimate.tagCount);
    Logger.recordOutput(logPreface + "AvgTagArea", poseEstimate.avgTagArea);

    double xyStds = XY_STD_DEV_DEFAULT;

    if (poseEstimate.tagCount > 0) {
      // Multiple targets detected
      if (poseEstimate.tagCount >= 2 && poseEstimate.avgTagArea > MIN_TAG_AREA_MULTI_TAG) {
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
      } else if (poseEstimate.tagCount > 1) {
        xyStds = XY_STD_DEV_MULTI_TAG;
      } else {
        // Single tag, far from current estimate — accept with very high std devs
        // so vision can still nudge the pose instead of being silently dropped
        xyStds = 4.0;
      }

      // Turret camera has extra transform chain noise (camera→turret→robot),
      // so trust it less than drivetrain cameras.
      if (isTurretCamera) {
        xyStds *= TURRET_CAMERA_STD_DEV_MULTIPLIER;
      }

      Matrix<N3, N1> visionMeasurementStdDevs =
          VecBuilder.fill(xyStds, xyStds, MEGATAG2_ROTATION_STD_DEV);

      Logger.recordOutput(logPreface + "StdDevs", visionMeasurementStdDevs.getData());

      return Optional.of(
          new VisionFieldPoseEstimate(
              fieldToRobotEstimate, poseEstimate.timestampSeconds, visionMeasurementStdDevs));
    }

    return Optional.empty();
  }

  private Optional<VisionFieldPoseEstimate> processMegatagPoseEstimate(
      MegatagPoseEstimate poseEstimate, boolean isTurretCamera, String logPreface) {

    if (poseEstimate == null || poseEstimate.fieldPose == null) {
      return Optional.empty();
    }

    var loggedFieldToRobot = state.getFieldToRobot(poseEstimate.timestampSeconds);
    if (loggedFieldToRobot.isEmpty()) {
      return Optional.empty();
    }

    var maybeFieldToRobotEstimate = getFieldToRobotEstimate(poseEstimate, isTurretCamera, false);
    if (maybeFieldToRobotEstimate.isEmpty()) {
      return Optional.empty();
    }

    var fieldToRobotEstimate = maybeFieldToRobotEstimate.get();

    double poseDifference =
        fieldToRobotEstimate
            .getTranslation()
            .getDistance(loggedFieldToRobot.get().getTranslation());

    if (poseEstimate.tagCount > 0) {
      double xyStds = 1.0;
      double degStds = 12;

      // Multiple targets detected
      if (poseEstimate.tagCount >= 2) {
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
        // Single tag, far from current estimate — accept with high std devs
        xyStds = 4.0;
        degStds = 60;
      }

      // Turret camera has extra transform chain noise
      if (isTurretCamera) {
        xyStds *= TURRET_CAMERA_STD_DEV_MULTIPLIER;
        degStds *= TURRET_CAMERA_STD_DEV_MULTIPLIER;
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
   * Reject turret camera frames captured during high angular velocity (turret or chassis). Mirrors
   * Team 254's shouldUsePinhole() rejection logic.
   */
  private boolean shouldRejectTurretVision(double captureTimestamp, String logPreface) {
    double lookback = Constants.Vision.VELOCITY_REJECTION_LOOKBACK;
    double minTime = captureTimestamp - lookback;
    double maxTime = captureTimestamp + lookback;

    var maxTurretVel = state.getMaxAbsTurretAngularVelocityInRange(minTime, maxTime);
    var maxDriveVel = state.getMaxAbsDriveAngularVelocityInRange(minTime, maxTime);

    boolean turretTooFast =
        maxTurretVel.isPresent()
            && maxTurretVel.get() > Constants.Vision.MAX_TURRET_ANGULAR_VELOCITY_FOR_VISION;
    boolean driveTooFast =
        maxDriveVel.isPresent()
            && maxDriveVel.get() > Constants.Vision.MAX_DRIVE_ANGULAR_VELOCITY_FOR_VISION;

    if (turretTooFast || driveTooFast) {
      Logger.recordOutput(
          logPreface + "RejectReason",
          "angular velocity too high (turret="
              + (maxTurretVel.isPresent()
                  ? String.format("%.1f", Math.toDegrees(maxTurretVel.get()))
                  : "?")
              + "°/s, drive="
              + (maxDriveVel.isPresent()
                  ? String.format("%.1f", Math.toDegrees(maxDriveVel.get()))
                  : "?")
              + "°/s)");
      return true;
    }
    return false;
  }

  /**
   * Calculate field-to-robot from vision. Drivetrain cameras return field-to-robot directly. Turret
   * camera MT2: returns turret field pose (one transform to robot). Turret camera MT1: returns
   * camera field pose (two transforms to robot).
   */
  private Optional<Pose2d> getFieldToRobotEstimate(
      MegatagPoseEstimate poseEstimate, boolean isTurretCamera, boolean isMT2) {

    var fieldPose = poseEstimate.fieldPose;
    if (fieldPose.getX() == 0.0) {
      return Optional.empty();
    }

    if (isTurretCamera) {
      // Use the interpolated turret angle at the vision timestamp for an accurate transform.
      // Interpolation between bracketing entries eliminates up-to-one-period staleness
      // that floorEntry alone would introduce.
      var turretAtTimestamp = state.getInterpolatedRobotToTurret(poseEstimate.timestampSeconds);
      double turretAngleRad = turretAtTimestamp.orElse(state.getLatestRobotToTurret().getValue());
      Rotation2d robotToTurretRotation = new Rotation2d(turretAngleRad);

      Pose2d fieldToTurret;
      if (isMT2) {
        // MT2: camerapose_robotspace_set is configured to TURRET_TO_CAMERA, so
        // botpose_wpiblue already returns the turret center's field pose.
        fieldToTurret = fieldPose;
      } else {
        // MT1: botpose_wpiblue returns the camera's raw field pose.
        // Step 1: Camera → Turret (inverse of turret-to-camera offset)
        var turretToCamera2d = state.getTurretToCamera();
        var cameraToTurret2d = turretToCamera2d.inverse();
        fieldToTurret = fieldPose.plus(cameraToTurret2d);
      }

      // Step 2 (common): Turret → Robot
      // ROBOT_TO_TURRET points robot-center → turret-center in robot frame.
      // Negate to get turret → robot, then rotate into turret-local frame.
      Rotation2d turretToRobotRotation = robotToTurretRotation.unaryMinus();
      Translation2d turretToRobotTranslation =
          Constants.Vision.ROBOT_TO_TURRET.unaryMinus().rotateBy(turretToRobotRotation);
      var turretToRobot = new Transform2d(turretToRobotTranslation, turretToRobotRotation);

      return Optional.of(fieldToTurret.plus(turretToRobot));
    } else {
      // Drivetrain camera — camerapose_robotspace_set is configured, so
      // botpose_wpiblue already returns field-to-robot directly
      return Optional.of(fieldPose);
    }
  }
}
