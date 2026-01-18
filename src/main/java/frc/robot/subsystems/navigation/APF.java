package frc.robot.subsystems.navigation;

import static frc.robot.Constants.APFConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;

/**
 * Artificial Potential Field system for autonomous ball intake navigation Creates attractive fields
 * toward balls and repulsive fields from obstacles/walls Prioritizes balls based on distance,
 * clustering, and alliance color
 */
public class APF {

  /** Target ball with calculated priority */
  public static class TargetBall {
    public final Ball ball;
    public final double priority;
    public final BallCluster cluster; // null if not in a cluster

    public TargetBall(Ball ball, double priority, BallCluster cluster) {
      this.ball = ball;
      this.priority = priority;
      this.cluster = cluster;
    }
  }

  /**
   * Calculate the best target ball with weighted priority Priority factors: - Distance (closer =
   * higher priority) - Cluster size (more balls together = higher priority) - Alliance color (our
   * alliance = bonus priority) - Confidence (higher confidence = higher priority)
   */
  public static TargetBall calculateBestTarget(Pose2d robotPose, List<Ball> detectedBalls) {
    if (detectedBalls.isEmpty()) {
      return null;
    }

    // Detect clusters
    List<BallCluster> clusters = detectClusters(detectedBalls);

    TargetBall bestTarget = null;
    double bestPriority = Double.NEGATIVE_INFINITY;

    for (Ball ball : detectedBalls) {
      // Find if ball is in a cluster
      BallCluster cluster = findCluster(ball, clusters);

      // Calculate priority
      double priority = calculateBallPriority(robotPose, ball, cluster);

      if (priority > bestPriority) {
        bestPriority = priority;
        bestTarget = new TargetBall(ball, priority, cluster);
      }
    }

    return bestTarget;
  }

  /** Calculate priority score for a ball */
  private static double calculateBallPriority(Pose2d robotPose, Ball ball, BallCluster cluster) {
    double distance = ball.distanceTo(robotPose);

    // Base priority: inverse of distance (closer = better)
    // Use exponential decay to strongly favor closer balls
    double distancePriority = DISTANCE_WEIGHT * Math.exp(-distance / DISTANCE_DECAY_FACTOR);

    // Cluster bonus: more balls together = higher priority
    double clusterPriority = 0;
    if (cluster != null) {
      // Exponential bonus for cluster size
      clusterPriority = CLUSTER_WEIGHT * (cluster.getSize() - 1);
    }

    // Confidence factor: reduce priority for low confidence detections
    double confidenceFactor = Math.pow(ball.getConfidence(), CONFIDENCE_EXPONENT);

    // Distance penalty: heavily penalize very far balls
    double distancePenalty = 0;
    if (distance > FAR_BALL_THRESHOLD) {
      distancePenalty = -FAR_BALL_PENALTY * (distance - FAR_BALL_THRESHOLD);
    }

    // Total priority (removed alliance bonus since all balls are the same)
    return (distancePriority + clusterPriority + distancePenalty) * confidenceFactor;
  }

  /** Detect clusters of balls using distance-based grouping */
  private static List<BallCluster> detectClusters(List<Ball> balls) {
    List<BallCluster> clusters = new ArrayList<>();
    List<Ball> unclustered = new ArrayList<>(balls);

    while (!unclustered.isEmpty()) {
      Ball seed = unclustered.remove(0);
      List<Ball> clusterBalls = new ArrayList<>();
      clusterBalls.add(seed);

      // Find all balls within cluster radius of this seed
      for (int i = unclustered.size() - 1; i >= 0; i--) {
        Ball candidate = unclustered.get(i);
        if (seed.distanceTo(candidate) <= CLUSTER_RADIUS) {
          clusterBalls.add(candidate);
          unclustered.remove(i);
        }
      }

      // Only create cluster if it has multiple balls
      if (clusterBalls.size() >= MIN_CLUSTER_SIZE) {
        clusters.add(new BallCluster(clusterBalls));
      }
    }

    return clusters;
  }

  /** Find which cluster contains a ball */
  private static BallCluster findCluster(Ball ball, List<BallCluster> clusters) {
    for (BallCluster cluster : clusters) {
      for (Ball clusterBall : cluster.getBalls()) {
        if (clusterBall == ball) {
          return cluster;
        }
      }
    }
    return null;
  }

  /** Calculate attractive force vector toward target ball */
  public static Translation2d calculateAttractiveForce(
      Pose2d robotPose, Translation2d targetPosition) {
    Translation2d toTarget = targetPosition.minus(robotPose.getTranslation());
    double distance = toTarget.getNorm();

    if (distance < 0.01) {
      return new Translation2d();
    }

    // Attractive force increases linearly with distance up to a maximum
    double forceMagnitude = Math.min(distance * ATTRACTIVE_GAIN, MAX_ATTRACTIVE_FORCE);

    return toTarget.div(distance).times(forceMagnitude);
  }

  /** Calculate repulsive force from field boundaries */
  public static Translation2d calculateRepulsiveForce(Pose2d robotPose) {
    Translation2d pos = robotPose.getTranslation();
    Translation2d repulsive = new Translation2d();

    // Field boundaries (16.54m x 8.21m)
    double fieldWidth = 16.54;
    double fieldHeight = 8.21;

    // Repulsive force from each wall
    double distToLeftWall = pos.getX();
    double distToRightWall = fieldWidth - pos.getX();
    double distToBottomWall = pos.getY();
    double distToTopWall = fieldHeight - pos.getY();

    // Apply repulsive force if within threshold distance
    if (distToLeftWall < WALL_REPULSION_DISTANCE) {
      double force = WALL_REPULSION_GAIN / (distToLeftWall + 0.1);
      repulsive = repulsive.plus(new Translation2d(force, 0));
    }

    if (distToRightWall < WALL_REPULSION_DISTANCE) {
      double force = WALL_REPULSION_GAIN / (distToRightWall + 0.1);
      repulsive = repulsive.plus(new Translation2d(-force, 0));
    }

    if (distToBottomWall < WALL_REPULSION_DISTANCE) {
      double force = WALL_REPULSION_GAIN / (distToBottomWall + 0.1);
      repulsive = repulsive.plus(new Translation2d(0, force));
    }

    if (distToTopWall < WALL_REPULSION_DISTANCE) {
      double force = WALL_REPULSION_GAIN / (distToTopWall + 0.1);
      repulsive = repulsive.plus(new Translation2d(0, -force));
    }

    return repulsive;
  }

  /**
   * Calculate total force vector and convert to robot velocity command Uses multi-ball attraction:
   * primary target + weighted secondary attractions
   */
  public static Twist2d calculateVelocityCommand(Pose2d robotPose, Translation2d targetPosition) {
    // Calculate attractive and repulsive forces
    Translation2d attractive = calculateAttractiveForce(robotPose, targetPosition);
    Translation2d repulsive = calculateRepulsiveForce(robotPose);

    // Total force
    Translation2d totalForce = attractive.plus(repulsive);

    // Convert to robot-relative coordinates
    Translation2d robotRelative = totalForce.rotateBy(robotPose.getRotation().unaryMinus());

    // Calculate desired heading (toward target)
    Translation2d toTarget = targetPosition.minus(robotPose.getTranslation());
    double desiredHeading = Math.atan2(toTarget.getY(), toTarget.getX());
    double currentHeading = robotPose.getRotation().getRadians();
    double headingError = desiredHeading - currentHeading;

    // Normalize angle error to [-π, π]
    while (headingError > Math.PI) headingError -= 2 * Math.PI;
    while (headingError < -Math.PI) headingError += 2 * Math.PI;

    // Angular velocity proportional to heading error
    double omega = headingError * HEADING_GAIN;
    omega = Math.max(-MAX_ANGULAR_VELOCITY, Math.min(MAX_ANGULAR_VELOCITY, omega));

    // Limit linear velocity
    double vx = Math.max(-MAX_LINEAR_VELOCITY, Math.min(MAX_LINEAR_VELOCITY, robotRelative.getX()));
    double vy = Math.max(-MAX_LINEAR_VELOCITY, Math.min(MAX_LINEAR_VELOCITY, robotRelative.getY()));

    return new Twist2d(vx, vy, omega);
  }

  /**
   * Calculate velocity command with strategic clustering Primary target pulls strongly, nearby
   * balls add extra attraction Creates "flow" into dense ball regions
   *
   * @param robotPose Current robot pose
   * @param primaryTarget Main target ball position
   * @param allBalls All detected balls (for secondary attractions)
   * @return Velocity command with multi-ball influence
   */
  public static Twist2d calculateVelocityCommandWithClustering(
      Pose2d robotPose, Translation2d primaryTarget, List<Ball> allBalls) {

    // Primary attractive force (strongest)
    Translation2d primaryAttraction = calculateAttractiveForce(robotPose, primaryTarget);

    // Secondary attractions from nearby balls (weaker, adds "flow" to clusters)
    Translation2d secondaryAttraction = new Translation2d();
    for (Ball ball : allBalls) {
      Translation2d ballPos = ball.getPosition();

      // Skip if this is the primary target (within tolerance)
      if (ballPos.getDistance(primaryTarget) < Constants.NavigationConstants.SAME_BALL_TOLERANCE) {
        continue;
      }

      // Calculate weak attraction to this ball
      Translation2d toBall = ballPos.minus(robotPose.getTranslation());
      double distance = toBall.getNorm();

      if (distance < 0.01) continue;

      // Weaker force that decays rapidly with distance
      // Only influences path if ball is reasonably close
      double influenceRadius = 3.0; // meters
      if (distance < influenceRadius) {
        double forceMagnitude =
            SECONDARY_BALL_ATTRACTION * Math.exp(-distance / SECONDARY_BALL_DECAY);
        secondaryAttraction = secondaryAttraction.plus(toBall.div(distance).times(forceMagnitude));
      }
    }

    // Repulsive force from walls
    Translation2d repulsive = calculateRepulsiveForce(robotPose);

    // Combine all forces (primary strongest, secondary adds "gravity" from clusters)
    Translation2d totalForce =
        primaryAttraction
            .plus(
                secondaryAttraction.times(
                    Constants.NavigationConstants.SECONDARY_ATTRACTION_FACTOR))
            .plus(repulsive);

    // Convert to robot-relative coordinates
    Translation2d robotRelative = totalForce.rotateBy(robotPose.getRotation().unaryMinus());

    // Calculate desired heading (toward primary target, not cluster center)
    Translation2d toTarget = primaryTarget.minus(robotPose.getTranslation());
    double desiredHeading = Math.atan2(toTarget.getY(), toTarget.getX());
    double currentHeading = robotPose.getRotation().getRadians();
    double headingError = desiredHeading - currentHeading;

    // Normalize angle error to [-π, π]
    while (headingError > Math.PI) headingError -= 2 * Math.PI;
    while (headingError < -Math.PI) headingError += 2 * Math.PI;

    // Angular velocity proportional to heading error
    double omega = headingError * HEADING_GAIN;
    omega = Math.max(-MAX_ANGULAR_VELOCITY, Math.min(MAX_ANGULAR_VELOCITY, omega));

    // Limit linear velocity
    double vx = Math.max(-MAX_LINEAR_VELOCITY, Math.min(MAX_LINEAR_VELOCITY, robotRelative.getX()));
    double vy = Math.max(-MAX_LINEAR_VELOCITY, Math.min(MAX_LINEAR_VELOCITY, robotRelative.getY()));

    return new Twist2d(vx, vy, omega);
  }
}
