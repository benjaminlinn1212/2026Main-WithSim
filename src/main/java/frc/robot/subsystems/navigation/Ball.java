package frc.robot.subsystems.navigation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Represents a detected ball on the field with position and metadata */
public class Ball {
  private final Translation2d position;
  private final double detectionTimestamp;
  private final double confidence;
  private final double areaPixels;

  public Ball(Translation2d position, double timestamp, double confidence, double areaPixels) {
    this.position = position;
    this.detectionTimestamp = timestamp;
    this.confidence = confidence;
    this.areaPixels = areaPixels;
  }

  public Translation2d getPosition() {
    return position;
  }

  public double getDetectionTimestamp() {
    return detectionTimestamp;
  }

  public double getConfidence() {
    return confidence;
  }

  public double getAreaPixels() {
    return areaPixels;
  }

  /** Calculate distance to another ball */
  public double distanceTo(Ball other) {
    return position.getDistance(other.position);
  }

  /** Calculate distance to a robot pose */
  public double distanceTo(Pose2d robotPose) {
    return position.getDistance(robotPose.getTranslation());
  }

  @Override
  public String toString() {
    return String.format(
        "Ball(pos=(%.2f, %.2f), conf=%.2f)", position.getX(), position.getY(), confidence);
  }
}
