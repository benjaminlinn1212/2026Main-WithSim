package frc.robot.subsystems.navigation;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

/**
 * Represents a cluster of balls that are close together Used for prioritizing groups of balls over
 * individual ones
 */
public class BallCluster {
  private final List<Ball> balls;
  private final Translation2d centroid;
  private final double averageConfidence;

  public BallCluster(List<Ball> balls) {
    this.balls = new ArrayList<>(balls);
    this.centroid = calculateCentroid();
    this.averageConfidence = calculateAverageConfidence();
  }

  private Translation2d calculateCentroid() {
    if (balls.isEmpty()) {
      return new Translation2d();
    }

    double sumX = 0;
    double sumY = 0;
    for (Ball ball : balls) {
      sumX += ball.getPosition().getX();
      sumY += ball.getPosition().getY();
    }

    return new Translation2d(sumX / balls.size(), sumY / balls.size());
  }

  private double calculateAverageConfidence() {
    if (balls.isEmpty()) {
      return 0.0;
    }

    double sum = 0;
    for (Ball ball : balls) {
      sum += ball.getConfidence();
    }

    return sum / balls.size();
  }

  public List<Ball> getBalls() {
    return new ArrayList<>(balls);
  }

  public Translation2d getCentroid() {
    return centroid;
  }

  public int getSize() {
    return balls.size();
  }

  public double getAverageConfidence() {
    return averageConfidence;
  }

  /** Calculate the maximum radius of the cluster from the centroid */
  public double getRadius() {
    double maxDist = 0;
    for (Ball ball : balls) {
      double dist = centroid.getDistance(ball.getPosition());
      if (dist > maxDist) {
        maxDist = dist;
      }
    }
    return maxDist;
  }

  @Override
  public String toString() {
    return String.format(
        "BallCluster(size=%d, centroid=(%.2f, %.2f), avgConf=%.2f)",
        balls.size(), centroid.getX(), centroid.getY(), averageConfidence);
  }
}
