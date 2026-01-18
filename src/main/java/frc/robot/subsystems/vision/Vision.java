package frc.robot.subsystems.vision;

import static frc.robot.Constants.BallVision.*;
import static frc.robot.Constants.Vision.LIMELIGHT_CAMERAS;
import static frc.robot.Constants.Vision.MULTI_TAG_STD_DEVS;
import static frc.robot.Constants.Vision.SINGLE_TAG_STD_DEVS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallVision;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.navigation.Ball;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private Matrix<N3, N1> curStdDevs = SINGLE_TAG_STD_DEVS;
  private final EstimateConsumer estConsumer;
  private final Supplier<Rotation2d> gyroYawSupplier;
  private final Supplier<Pose2d> robotPoseSupplier;
  private int visionLoopCounter = 0; // Throttle vision processing to reduce loop time

  // Ball detection
  private List<Ball> detectedBalls = new ArrayList<>();
  private String ballDetectionLimelight = null; // Name of limelight used for ball detection

  /**
   * @param estConsumer Lambda that will accept a pose estimate and pass it to your desired {@link
   *     edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
   * @param gyroYawSupplier Supplier that provides the current gyro yaw (not used by MegaTag1, kept
   *     for compatibility)
   * @param robotPoseSupplier Supplier that provides the current robot pose for ball position
   *     calculations
   */
  public Vision(
      EstimateConsumer estConsumer,
      Supplier<Rotation2d> gyroYawSupplier,
      Supplier<Pose2d> robotPoseSupplier) {
    this.estConsumer = estConsumer;
    this.gyroYawSupplier = gyroYawSupplier;
    this.robotPoseSupplier = robotPoseSupplier;
    // Limelight doesn't need complex initialization - uses NetworkTables
  }

  /**
   * Configure which Limelight to use for ball detection (optional)
   *
   * @param limelightName Name of the limelight (must match one in LIMELIGHT_CAMERAS)
   * @param ballDetectionPipeline Pipeline index for ball detection
   */
  public void enableBallDetection(String limelightName, int ballDetectionPipeline) {
    this.ballDetectionLimelight = limelightName;
    LimelightHelpers.setPipelineIndex(limelightName, ballDetectionPipeline);
  }

  /** Disable ball detection and switch back to AprilTag detection */
  public void disableBallDetection() {
    this.ballDetectionLimelight = null;
  }

  public void periodic() {
    // Throttle vision processing to every 3rd loop to reduce CPU load
    // Vision updates don't need to run every 20ms - 60ms is sufficient
    visionLoopCounter++;
    if (visionLoopCounter % 3 != 0) {
      return;
    }

    periodicLimelight();

    // Process ball detection if enabled
    if (ballDetectionLimelight != null) {
      periodicBallDetection();
    }
  }

  private void periodicLimelight() {
    // Cache alliance to avoid race condition
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

    // Process all configured Limelights
    for (var limelightCamera : LIMELIGHT_CAMERAS) {
      // Update robot orientation (not used by MegaTag1 but kept for future flexibility)
      double yaw = gyroYawSupplier.get().getDegrees();
      LimelightHelpers.SetRobotOrientation(limelightCamera.name, yaw, 0, 0, 0, 0, 0);

      // Get MegaTag1 pose estimate (alliance-aware, uses AprilTag field layout only)
      PoseEstimate limelightPose =
          isRed
              ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag1(limelightCamera.name)
              : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag1(limelightCamera.name);

      // Only process valid estimates
      if (LimelightHelpers.validPoseEstimate(limelightPose)) {
        // Early return if measurement quality is too poor
        if (!updateEstimationStdDevsLimelight(limelightPose)) {
          continue; // Skip this measurement
        }
        var estStdDevs = getEstimationStdDevs();
        estConsumer.accept(limelightPose.pose, limelightPose.timestampSeconds, estStdDevs);
      }
    }
  }

  /**
   * Calculates new standard deviations for Limelight MegaTag1 MegaTag1 uses only AprilTag positions
   * from the field layout, not gyro data
   *
   * @param poseEstimate The MegaTag1 pose estimate
   * @return true if the measurement should be used, false if it should be rejected
   */
  private boolean updateEstimationStdDevsLimelight(PoseEstimate poseEstimate) {
    if (poseEstimate == null || poseEstimate.tagCount == 0) {
      curStdDevs = SINGLE_TAG_STD_DEVS;
      return false;
    }

    var estStdDevs = SINGLE_TAG_STD_DEVS;
    int numTags = poseEstimate.tagCount;
    double avgDist = poseEstimate.avgTagDist;

    // Decrease std devs if multiple targets are visible (improves accuracy)
    if (numTags > 1) {
      estStdDevs = MULTI_TAG_STD_DEVS;
    }

    // Reject measurement if single tag is too far
    if (numTags == 1 && avgDist > 4) {
      curStdDevs = SINGLE_TAG_STD_DEVS;
      return false; // Reject this measurement
    }

    // Increase std devs based on distance
    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    curStdDevs = estStdDevs;
    return true;
  }

  /** Returns the latest standard deviations of the estimated pose. */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  /**
   * Process ball detections from configured Limelight Uses multi-target neural detector to find all
   * visible balls
   */
  private void periodicBallDetection() {
    if (!LimelightHelpers.hasTarget(ballDetectionLimelight)) {
      detectedBalls.clear();
      return;
    }

    Pose2d robotPose = robotPoseSupplier.get();
    double latency = LimelightHelpers.getLatency(ballDetectionLimelight);
    double timestamp = Timer.getFPGATimestamp() - (latency / 1000.0);

    // Find camera transform for ball detection limelight
    Transform3d cameraTransform = null;
    for (var camera : LIMELIGHT_CAMERAS) {
      if (camera.name.equals(ballDetectionLimelight)) {
        cameraTransform = camera.robotToCamera;
        break;
      }
    }

    if (cameraTransform == null) {
      detectedBalls.clear();
      return; // Camera not found in configuration
    }

    // Get all neural detector targets
    LimelightHelpers.NeuralTarget[] targets =
        LimelightHelpers.getNeuralTargets(ballDetectionLimelight);

    List<Ball> newBalls = new ArrayList<>();

    // Process each detected target
    for (LimelightHelpers.NeuralTarget target : targets) {
      Translation2d ballPosition =
          calculateBallPositionFromCamera(
              robotPose, cameraTransform, target.tx, target.ty, target.ta);

      if (ballPosition != null && isValidFieldPosition(ballPosition)) {
        Ball ball = new Ball(ballPosition, timestamp, target.confidence, target.ta);
        newBalls.add(ball);
      }
    }

    detectedBalls = newBalls;

    // Log ball detections
    Logger.recordOutput("Vision/BallDetection/Count", detectedBalls.size());
    if (!detectedBalls.isEmpty()) {
      double[] xPos = new double[detectedBalls.size()];
      double[] yPos = new double[detectedBalls.size()];
      for (int i = 0; i < detectedBalls.size(); i++) {
        xPos[i] = detectedBalls.get(i).getPosition().getX();
        yPos[i] = detectedBalls.get(i).getPosition().getY();
      }
      Logger.recordOutput("Vision/BallDetection/XPositions", xPos);
      Logger.recordOutput("Vision/BallDetection/YPositions", yPos);
    }
  }

  /**
   * Calculate ball field position from camera angles and area
   *
   * @param robotPose Current robot pose on field
   * @param cameraTransform Transform from robot center to camera
   * @param tx Horizontal offset (degrees)
   * @param ty Vertical offset (degrees)
   * @param ta Target area (0-100)
   * @return Ball position on field, or null if invalid
   */
  private Translation2d calculateBallPositionFromCamera(
      Pose2d robotPose, Transform3d cameraTransform, double tx, double ty, double ta) {
    if (ta <= 0) {
      return null;
    }

    // Convert angles to radians
    double txRad = Math.toRadians(tx);
    double tyRad = Math.toRadians(ty);

    // Estimate distance from area using inverse square law
    // distance = sqrt(calibration_constant / area)
    double distance = Math.sqrt(AREA_CALIBRATION_CONSTANT / ta);

    if (distance < 0 || distance > BallVision.MAX_BALL_DISTANCE) {
      return null;
    }

    // Calculate ball position relative to camera
    // Get camera mount angle from transform
    double mountAngle = Math.atan2(-cameraTransform.getRotation().getY(), 1.0);
    double angleToTarget = mountAngle + tyRad;
    double horizontalDistance = distance * Math.cos(angleToTarget);

    // Calculate position relative to robot center
    double xRelative = horizontalDistance * Math.cos(txRad) + cameraTransform.getX();
    double yRelative = horizontalDistance * Math.sin(txRad) + cameraTransform.getY();

    // Transform to field coordinates
    Translation2d relativePosition = new Translation2d(xRelative, yRelative);
    Translation2d fieldPosition =
        robotPose.getTranslation().plus(relativePosition.rotateBy(robotPose.getRotation()));

    return fieldPosition;
  }

  /** Validate that position is within field bounds */
  private boolean isValidFieldPosition(Translation2d position) {
    // 2024 field is 16.54m x 8.21m
    return position.getX() >= 0
        && position.getX() <= 16.54
        && position.getY() >= 0
        && position.getY() <= 8.21;
  }

  /**
   * Get all detected balls
   *
   * @return List of all currently detected balls
   */
  public List<Ball> getDetectedBalls() {
    return new ArrayList<>(detectedBalls);
  }

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }
}
