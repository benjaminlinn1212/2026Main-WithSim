package frc.robot.subsystems.vision;

import static frc.robot.Constants.Vision.LIMELIGHT_CAMERAS;
import static frc.robot.Constants.Vision.MULTI_TAG_STD_DEVS;
import static frc.robot.Constants.Vision.SINGLE_TAG_STD_DEVS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import java.util.function.Supplier;

public class Vision extends SubsystemBase {
  private Matrix<N3, N1> curStdDevs = SINGLE_TAG_STD_DEVS;
  private final EstimateConsumer estConsumer;
  private final Supplier<Rotation2d> gyroYawSupplier;
  private int visionLoopCounter = 0; // Throttle vision processing to reduce loop time

  /**
   * @param estConsumer Lambda that will accept a pose estimate and pass it to your desired {@link
   *     edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
   * @param gyroYawSupplier Supplier that provides the current gyro yaw (not used by MegaTag1, kept
   *     for compatibility)
   */
  public Vision(EstimateConsumer estConsumer, Supplier<Rotation2d> gyroYawSupplier) {
    this.estConsumer = estConsumer;
    this.gyroYawSupplier = gyroYawSupplier;
    // Limelight doesn't need complex initialization - uses NetworkTables
  }

  public void periodic() {
    // Throttle vision processing to every 3rd loop to reduce CPU load
    // Vision updates don't need to run every 20ms - 60ms is sufficient
    visionLoopCounter++;
    if (visionLoopCounter % 3 != 0) {
      return;
    }

    periodicLimelight();
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

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }
}
