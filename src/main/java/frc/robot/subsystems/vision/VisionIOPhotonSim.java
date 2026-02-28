// Copyright (c) 2026 FRC Team 10922 (Amped)
// PhotonVision camera simulation for AprilTag detection

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.auto.dashboard.FieldConstants;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * Vision IO implementation using PhotonVision's camera simulation for AprilTag detection. This
 * simulates two cameras (front drivetrain + turret-mounted) detecting AprilTags placed on the
 * REBUILT field using PhotonVision's VisionSystemSim.
 *
 * <p>The front camera is rigidly mounted on the drivetrain. The turret camera moves with the turret
 * — its robot-relative transform is updated every cycle based on the current turret angle.
 */
public class VisionIOPhotonSim implements VisionIO {

  // PhotonVision simulation
  private final VisionSystemSim visionSim;

  // Front camera (drivetrain-mounted, fixed)
  private final PhotonCamera frontCamera;
  private final PhotonCameraSim frontCameraSim;
  private final PhotonPoseEstimator frontPoseEstimator;

  // Turret camera (turret-mounted, rotates with turret)
  private final PhotonCamera turretCamera;
  private final PhotonCameraSim turretCameraSim;
  private final PhotonPoseEstimator turretPoseEstimator;

  // Robot state for pose + turret angle
  private final Supplier<Pose2d> poseSupplier;
  private final RobotState robotState;

  // AprilTag field layout
  private final AprilTagFieldLayout tagLayout;

  // Robot-to-camera transforms (3D)
  // RIGHT_CAMERA_TO_ROBOT stores robot→camera with Limelight pitch convention
  // (positive pitch = camera angled UP). PhotonVision uses WPILib's Rotation3d where
  // positive Ry = pitched DOWN. We must negate the pitch for PhotonVision.
  private static final Transform3d ROBOT_TO_FRONT_CAMERA =
      new Transform3d(
          Constants.Vision.RIGHT_CAMERA_TO_ROBOT.getTranslation(),
          new Rotation3d(
              Constants.Vision.RIGHT_CAMERA_TO_ROBOT.getRotation().getX(),
              -Constants.Vision.RIGHT_CAMERA_TO_ROBOT.getRotation().getY(), // Negate pitch
              Constants.Vision.RIGHT_CAMERA_TO_ROBOT.getRotation().getZ()));

  /**
   * Create a new VisionIOPhotonSim.
   *
   * @param poseSupplier Supplies the current robot pose (from drive/odometry)
   * @param robotState Robot state for turret angle lookups
   */
  public VisionIOPhotonSim(Supplier<Pose2d> poseSupplier, RobotState robotState) {
    this.poseSupplier = poseSupplier;
    this.robotState = robotState;

    // Load the AprilTag field layout for the REBUILT field
    tagLayout = FieldConstants.getAprilTagFieldLayout();

    // Create the vision system simulation
    visionSim = new VisionSystemSim("photonSim");
    visionSim.addAprilTags(tagLayout);

    // ===== Front Camera (drivetrain-mounted) =====
    var frontCamProps = new SimCameraProperties();
    frontCamProps.setCalibration(960, 720, Rotation2d.fromDegrees(70));
    frontCamProps.setCalibError(0.25, 0.08);
    frontCamProps.setFPS(20);
    frontCamProps.setAvgLatencyMs(35);
    frontCamProps.setLatencyStdDevMs(5);

    frontCamera = new PhotonCamera(Constants.Vision.FRONT_LIMELIGHT_NAME);
    frontCameraSim = new PhotonCameraSim(frontCamera, frontCamProps);
    frontCameraSim.enableDrawWireframe(true);
    frontCameraSim.setMinTargetAreaPixels(10.0);
    visionSim.addCamera(frontCameraSim, ROBOT_TO_FRONT_CAMERA);

    frontPoseEstimator = new PhotonPoseEstimator(tagLayout, ROBOT_TO_FRONT_CAMERA);

    // ===== Turret Camera (turret-mounted, dynamic transform) =====
    var turretCamProps = new SimCameraProperties();
    turretCamProps.setCalibration(960, 720, Rotation2d.fromDegrees(70));
    turretCamProps.setCalibError(0.35, 0.10);
    turretCamProps.setFPS(20);
    turretCamProps.setAvgLatencyMs(40);
    turretCamProps.setLatencyStdDevMs(8);

    turretCamera = new PhotonCamera(Constants.Vision.TURRET_LIMELIGHT_NAME);
    turretCameraSim = new PhotonCameraSim(turretCamera, turretCamProps);
    turretCameraSim.enableDrawWireframe(true);
    turretCameraSim.setMinTargetAreaPixels(10.0);

    // Initial turret camera transform (turret at 0 rad)
    Transform3d robotToTurretCamera = computeRobotToTurretCamera(0.0);
    visionSim.addCamera(turretCameraSim, robotToTurretCamera);

    turretPoseEstimator = new PhotonPoseEstimator(tagLayout, robotToTurretCamera);
  }

  /**
   * Compute the robot→turretCamera transform given the current turret angle. This chains:
   * robot→turret (translation + rotation) then turret→camera.
   *
   * @param turretAngleRad The current turret angle in radians (robot-relative)
   * @return The 3D transform from robot center to turret camera
   */
  private Transform3d computeRobotToTurretCamera(double turretAngleRad) {
    // Robot → turret center (2D offset lifted to 3D + turret rotation about Z)
    var turretOffset = Constants.Vision.ROBOT_TO_TURRET;
    Transform3d robotToTurret =
        new Transform3d(
            new Translation3d(turretOffset.getX(), turretOffset.getY(), 0.0),
            new Rotation3d(0, 0, turretAngleRad));

    // Turret center → camera (fixed offset defined in Constants)
    // Negate pitch: Constants uses Limelight convention (positive = up),
    // PhotonVision uses WPILib Rotation3d (positive Ry = down).
    Transform3d turretToCamera =
        new Transform3d(
            Constants.Vision.TURRET_TO_CAMERA.getTranslation(),
            new Rotation3d(
                Constants.Vision.TURRET_TO_CAMERA.getRotation().getX(),
                -Constants.Vision.TURRET_TO_CAMERA.getRotation().getY(),
                Constants.Vision.TURRET_TO_CAMERA.getRotation().getZ()));

    // Chain: robot → turret → camera
    // Transform3d.plus() composes transforms: A.plus(B) = A then B
    return robotToTurret.plus(turretToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Get current robot pose and turret angle
    Pose2d robotPose = poseSupplier.get();
    double turretAngleRad = robotState.getLatestRobotToTurret().getValue();

    // Update turret camera's robot-relative transform based on current turret angle
    Transform3d robotToTurretCamera = computeRobotToTurretCamera(turretAngleRad);
    visionSim.adjustCamera(turretCameraSim, robotToTurretCamera);
    turretPoseEstimator.setRobotToCameraTransform(robotToTurretCamera);

    // Update the simulation with the current robot pose
    visionSim.update(robotPose);

    // Log the simulated field for debugging
    Logger.recordOutput("Vision/Sim/Field", visionSim.getDebugField().getRobotPose());

    // ===== Process Front Camera =====
    var frontResults = frontCamera.getAllUnreadResults();
    inputs.frontCameraSeesTarget = false;
    inputs.frontCameraMegatagPoseEstimate = null;
    inputs.frontCameraMegatag2PoseEstimate = null;

    for (var result : frontResults) {
      if (result.hasTargets()) {
        inputs.frontCameraSeesTarget = true;

        // Use multi-tag PNP from coprocessor as primary strategy
        Optional<EstimatedRobotPose> multiTagEstimate =
            frontPoseEstimator.estimateCoprocMultiTagPose(result);
        if (multiTagEstimate.isPresent()) {
          inputs.frontCameraMegatag2PoseEstimate =
              toMegatagPoseEstimate(multiTagEstimate.get(), result);
        }

        // Use lowest ambiguity as fallback (equivalent to MegaTag1)
        Optional<EstimatedRobotPose> singleTagEstimate =
            frontPoseEstimator.estimateLowestAmbiguityPose(result);
        if (singleTagEstimate.isPresent()) {
          inputs.frontCameraMegatagPoseEstimate =
              toMegatagPoseEstimate(singleTagEstimate.get(), result);
        }
      }
    }

    // ===== Process Turret Camera =====
    var turretResults = turretCamera.getAllUnreadResults();
    inputs.turretCameraSeesTarget = false;
    inputs.turretCameraMegatagPoseEstimate = null;
    inputs.turretCameraMegatag2PoseEstimate = null;

    for (var result : turretResults) {
      if (result.hasTargets()) {
        inputs.turretCameraSeesTarget = true;

        Optional<EstimatedRobotPose> multiTagEstimate =
            turretPoseEstimator.estimateCoprocMultiTagPose(result);
        if (multiTagEstimate.isPresent()) {
          inputs.turretCameraMegatag2PoseEstimate =
              toMegatagPoseEstimate(multiTagEstimate.get(), result);
        }

        Optional<EstimatedRobotPose> singleTagEstimate =
            turretPoseEstimator.estimateLowestAmbiguityPose(result);
        if (singleTagEstimate.isPresent()) {
          inputs.turretCameraMegatagPoseEstimate =
              toMegatagPoseEstimate(singleTagEstimate.get(), result);
        }
      }
    }
  }

  /**
   * Convert a PhotonVision EstimatedRobotPose to our MegatagPoseEstimate format. This bridges
   * PhotonVision's pose estimation output with the existing VisionSubsystem pipeline that expects
   * Limelight-style MegatagPoseEstimate objects.
   */
  private MegatagPoseEstimate toMegatagPoseEstimate(
      EstimatedRobotPose estimatedPose, org.photonvision.targeting.PhotonPipelineResult result) {

    MegatagPoseEstimate estimate = new MegatagPoseEstimate();
    Pose3d fieldPose3d = estimatedPose.estimatedPose;
    estimate.fieldPose = fieldPose3d.toPose2d();
    estimate.timestampSeconds = estimatedPose.timestampSeconds;
    estimate.latency = result.metadata.getLatencyMillis();
    estimate.tagCount = result.getTargets().size();

    // Extract fiducial IDs
    var targets = result.getTargets();
    estimate.fiducialIds = new int[targets.size()];
    double totalArea = 0;
    double totalDist = 0;
    for (int i = 0; i < targets.size(); i++) {
      estimate.fiducialIds[i] = targets.get(i).getFiducialId();
      totalArea += targets.get(i).getArea();
      var bestCamToTarget = targets.get(i).getBestCameraToTarget();
      totalDist += bestCamToTarget.getTranslation().getNorm();
    }
    estimate.avgTagArea = targets.isEmpty() ? 0 : totalArea / targets.size();
    estimate.avgTagDist = targets.isEmpty() ? 0 : totalDist / targets.size();

    return estimate;
  }
}
