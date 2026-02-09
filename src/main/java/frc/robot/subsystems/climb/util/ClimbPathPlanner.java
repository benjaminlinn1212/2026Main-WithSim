package frc.robot.subsystems.climb.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ClimbConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

/** Path planning utilities for smooth climb motions. */
public class ClimbPathPlanner {

  /** Represents a planned path for one side of the climb mechanism. */
  public static class ClimbPath {
    private final List<Translation2d> waypoints;
    private final double totalDuration;
    private final PathType pathType;
    private final Trajectory trajectory; // WPILib trajectory with time-parameterization

    public ClimbPath(List<Translation2d> waypoints, double totalDuration, PathType pathType) {
      this(waypoints, totalDuration, pathType, null);
    }

    public ClimbPath(
        List<Translation2d> waypoints,
        double totalDuration,
        PathType pathType,
        Trajectory trajectory) {
      this.waypoints = waypoints;
      this.totalDuration = totalDuration;
      this.pathType = pathType;
      this.trajectory = trajectory;
    }

    public List<Translation2d> getWaypoints() {
      return waypoints;
    }

    public double getTotalDuration() {
      return totalDuration;
    }

    public PathType getPathType() {
      return pathType;
    }

    /** Get the number of waypoints in the path */
    public int getWaypointCount() {
      return waypoints.size();
    }

    /** Get the WPILib trajectory (if available) for time-based sampling */
    public Trajectory getTrajectory() {
      return trajectory;
    }

    /** Check if this path has trajectory data */
    public boolean hasTrajectory() {
      return trajectory != null;
    }
  }

  // ===========================================================================
  // Path Generation - Various interpolation methods
  // ===========================================================================

  /** Path interpolation types */
  public enum PathType {
    /** Linear interpolation - straight line between points */
    LINEAR,
    /** Smooth interpolation using cubic splines */
    SMOOTH,
    /** Trapezoidal velocity profile */
    TRAPEZOID
  }

  /**
   * Create a linear path between two positions.
   *
   * @param start Starting position
   * @param end Ending position
   * @param duration Time to complete the path (seconds)
   * @param numWaypoints Number of intermediate waypoints to generate
   * @return Planned path
   */
  public static ClimbPath createLinearPath(
      Translation2d start, Translation2d end, double duration, int numWaypoints) {
    List<Translation2d> waypoints = new ArrayList<>();

    for (int i = 0; i <= numWaypoints; i++) {
      double t = (double) i / numWaypoints;
      Translation2d point = start.interpolate(end, t);
      waypoints.add(point);
    }

    return new ClimbPath(waypoints, duration, PathType.LINEAR);
  }

  /**
   * Create a smooth path through multiple waypoints using spline-like interpolation.
   *
   * @param waypoints List of waypoints to pass through
   * @param duration Total time to complete the path (seconds)
   * @return Planned path with interpolated points
   */
  public static ClimbPath createSmoothPath(List<Translation2d> waypoints, double duration) {
    if (waypoints.size() < 2) {
      throw new IllegalArgumentException("Need at least 2 waypoints for a path");
    }

    List<Translation2d> smoothedPath = new ArrayList<>();
    int pointsPerSegment = 10; // Resolution between waypoints

    for (int i = 0; i < waypoints.size() - 1; i++) {
      Translation2d current = waypoints.get(i);
      Translation2d next = waypoints.get(i + 1);

      for (int j = 0; j < pointsPerSegment; j++) {
        double t = (double) j / pointsPerSegment;
        // Simple linear interpolation (can be upgraded to cubic later)
        Translation2d point = current.interpolate(next, smoothEaseInOut(t));
        smoothedPath.add(point);
      }
    }
    // Add final waypoint
    smoothedPath.add(waypoints.get(waypoints.size() - 1));

    return new ClimbPath(smoothedPath, duration, PathType.SMOOTH);
  }

  /**
   * Create a cubic Bezier curve path through waypoints with control points.
   *
   * <p>A Bezier curve provides smooth, continuous curves that pass through waypoints with
   * controllable curvature using control points.
   *
   * @param start Starting position
   * @param controlPoint1 First control point (influences curve near start)
   * @param controlPoint2 Second control point (influences curve near end)
   * @param end Ending position
   * @param duration Total time to complete the path (seconds)
   * @return Planned Bezier curve path
   */
  public static ClimbPath createBezierPath(
      Translation2d start,
      Translation2d controlPoint1,
      Translation2d controlPoint2,
      Translation2d end,
      double duration) {

    List<Translation2d> bezierPath = new ArrayList<>();
    int numSamples = 50; // Resolution of the curve

    for (int i = 0; i <= numSamples; i++) {
      double t = (double) i / numSamples;
      Translation2d point = calculateCubicBezier(start, controlPoint1, controlPoint2, end, t);
      bezierPath.add(point);
    }

    return new ClimbPath(bezierPath, duration, PathType.SMOOTH);
  }

  /**
   * Create a smooth trajectory through multiple waypoints using WPILib's trajectory generation.
   * Automatically generates smooth quintic splines with proper velocity/acceleration constraints.
   *
   * @param waypoints List of waypoints to pass through (in climb 2D space: x=forward, y=up)
   * @param tension Controls how tight the curves are - NOT USED, kept for API compatibility
   * @param duration Total time to complete the path (seconds) - NOT USED, calculated automatically
   * @param maintainEndVelocity If true, maintain velocity at end (for pulling); if false,
   *     decelerate to 0
   * @return Planned trajectory with time-based sampling
   */
  public static ClimbPath createMultiBezierPath(
      List<Translation2d> waypoints, double tension, double duration, boolean maintainEndVelocity) {
    if (waypoints.size() < 2) {
      throw new IllegalArgumentException("Need at least 2 waypoints for a path");
    }

    // Configure trajectory constraints
    TrajectoryConfig config =
        new TrajectoryConfig(
            ClimbConstants.PATH_MAX_VELOCITY_MPS, ClimbConstants.PATH_MAX_ACCELERATION_MPS2);

    // Set end velocity based on motion type
    if (maintainEndVelocity) {
      config.setEndVelocity(ClimbConstants.PATH_MAX_VELOCITY_MPS);
    } else {
      config.setEndVelocity(0.0); // Come to rest
    }

    // Convert waypoints to poses (heading = direction of travel)
    List<Pose2d> poses = new ArrayList<>();
    for (int i = 0; i < waypoints.size(); i++) {
      Translation2d wp = waypoints.get(i);

      // Calculate heading (direction of travel)
      Rotation2d heading;
      if (i == 0 && waypoints.size() > 1) {
        // First: point toward next waypoint
        Translation2d direction = waypoints.get(1).minus(wp);
        heading = new Rotation2d(direction.getX(), direction.getY());
      } else if (i == waypoints.size() - 1 && waypoints.size() > 1) {
        // Last: continue from previous direction
        Translation2d direction = wp.minus(waypoints.get(i - 1));
        heading = new Rotation2d(direction.getX(), direction.getY());
      } else if (waypoints.size() > 2) {
        // Middle: average of incoming/outgoing
        Translation2d incoming = wp.minus(waypoints.get(i - 1));
        Translation2d outgoing = waypoints.get(i + 1).minus(wp);
        Translation2d avg = incoming.plus(outgoing).times(0.5);
        heading = new Rotation2d(avg.getX(), avg.getY());
      } else {
        heading = new Rotation2d(); // Default if only 1 point
      }

      poses.add(new Pose2d(wp, heading));
    }

    // Generate trajectory using WPILib's quintic spline interpolation
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(poses, config);

    // Extract waypoints from trajectory for visualization
    List<Translation2d> trajectoryPoints =
        trajectory.getStates().stream()
            .map(state -> state.poseMeters.getTranslation())
            .collect(Collectors.toList());

    // Use trajectory's actual duration (may differ from requested)
    double actualDuration = trajectory.getTotalTimeSeconds();

    return new ClimbPath(trajectoryPoints, actualDuration, PathType.SMOOTH, trajectory);
  }

  /**
   * Calculate a point on a cubic Bezier curve. Formula: B(t) = (1-t)^3*P0 + 3(1-t)^2*t*P1 +
   * 3(1-t)*t^2*P2 + t^3*P3
   *
   * @param p0 Start point
   * @param p1 First control point
   * @param p2 Second control point
   * @param p3 End point
   * @param t Parameter [0, 1]
   * @return Point on the Bezier curve at parameter t
   */
  private static Translation2d calculateCubicBezier(
      Translation2d p0, Translation2d p1, Translation2d p2, Translation2d p3, double t) {
    double u = 1 - t;
    double tt = t * t;
    double uu = u * u;
    double uuu = uu * u;
    double ttt = tt * t;

    // Bezier formula: (1-t)^3*P0 + 3(1-t)^2*t*P1 + 3(1-t)*t^2*P2 + t^3*P3
    Translation2d point = p0.times(uuu);
    point = point.plus(p1.times(3 * uu * t));
    point = point.plus(p2.times(3 * u * tt));
    point = point.plus(p3.times(ttt));

    return point;
  }

  /**
   * Create a path with trapezoidal velocity profile for smooth acceleration/deceleration. Uses
   * WPILib's trajectory generation for proper time-parameterization.
   *
   * @param start Starting position
   * @param end Ending position
   * @param maxVelocity Maximum velocity (m/s)
   * @param maxAcceleration Maximum acceleration (m/s^2)
   * @return Planned path with trapezoidal profile
   */
  public static ClimbPath createTrapezoidPath(
      Translation2d start, Translation2d end, double maxVelocity, double maxAcceleration) {

    // Calculate heading from start to end
    Translation2d direction = end.minus(start);
    Rotation2d heading = new Rotation2d(direction.getX(), direction.getY());

    // Create trajectory config
    TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);

    // Generate trajectory (simple 2-point path)
    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(start, heading),
            List.of(), // No interior waypoints
            new Pose2d(end, heading),
            config);

    // Extract waypoints from trajectory
    List<Translation2d> waypoints =
        trajectory.getStates().stream()
            .map(state -> state.poseMeters.getTranslation())
            .collect(Collectors.toList());

    return new ClimbPath(
        waypoints, trajectory.getTotalTimeSeconds(), PathType.TRAPEZOID, trajectory);
  }

  /**
   * Get the target position at a specific time along the path.
   *
   * <p>Uses WPILib's trajectory time-based sampling with built-in velocity profiles.
   *
   * @param path The path to sample
   * @param elapsedTime Time since path started (seconds)
   * @return Target position at this time
   */
  public static Translation2d getPositionAtTime(ClimbPath path, double elapsedTime) {
    // If we have a trajectory, use it directly (best option)
    if (path.hasTrajectory()) {
      Trajectory trajectory = path.getTrajectory();

      // Clamp time to valid range
      double clampedTime = Math.max(0, Math.min(elapsedTime, trajectory.getTotalTimeSeconds()));

      // Sample trajectory at this time
      Trajectory.State state = trajectory.sample(clampedTime);
      return state.poseMeters.getTranslation();
    }

    // Fallback: linear interpolation through waypoints (for legacy paths without trajectory)
    List<Translation2d> waypoints = path.getWaypoints();

    if (elapsedTime <= 0) {
      return waypoints.get(0);
    }

    if (elapsedTime >= path.getTotalDuration()) {
      return waypoints.get(waypoints.size() - 1);
    }

    // Simple time-based interpolation
    double progress = elapsedTime / path.getTotalDuration();
    int waypointCount = waypoints.size();
    double floatIndex = progress * (waypointCount - 1);
    int index = (int) Math.floor(floatIndex);
    double t = floatIndex - index;

    if (index >= waypointCount - 1) {
      return waypoints.get(waypointCount - 1);
    }

    Translation2d current = waypoints.get(index);
    Translation2d next = waypoints.get(index + 1);
    return current.interpolate(next, t);
  }

  /**
   * Validate that all points in a path are within the workspace and reachable.
   *
   * @param path Path to validate
   * @return true if entire path is valid
   */
  public static boolean isPathValid(ClimbPath path) {
    for (Translation2d point : path.getWaypoints()) {
      if (!ClimbIK.isPositionReachable(point)) {
        return false;
      }
      if (!isWithinWorkspace(point)) {
        return false;
      }
    }
    return true;
  }

  /**
   * Check if a position is within the defined workspace limits.
   *
   * @param position Position to check
   * @return true if within workspace
   */
  public static boolean isWithinWorkspace(Translation2d position) {
    return position.getX() >= ClimbConstants.WORKSPACE_MIN_X_METERS
        && position.getX() <= ClimbConstants.WORKSPACE_MAX_X_METERS
        && position.getY() >= ClimbConstants.WORKSPACE_MIN_Y_METERS
        && position.getY() <= ClimbConstants.WORKSPACE_MAX_Y_METERS;
  }

  /**
   * Create a symmetric path for both sides of the climb.
   *
   * @param start Starting position
   * @param end Ending position
   * @param duration Path duration
   * @return Pair of identical paths [left, right]
   */
  public static ClimbPath[] createSymmetricPath(
      Translation2d start, Translation2d end, double duration) {
    ClimbPath path = createLinearPath(start, end, duration, 20);
    return new ClimbPath[] {path, path};
  }

  /**
   * Create independent paths for left and right sides.
   *
   * @param leftStart Left side starting position
   * @param leftEnd Left side ending position
   * @param rightStart Right side starting position
   * @param rightEnd Right side ending position
   * @param duration Path duration
   * @return Array of [leftPath, rightPath]
   */
  public static ClimbPath[] createIndependentPaths(
      Translation2d leftStart,
      Translation2d leftEnd,
      Translation2d rightStart,
      Translation2d rightEnd,
      double duration) {
    ClimbPath leftPath = createLinearPath(leftStart, leftEnd, duration, 20);
    ClimbPath rightPath = createLinearPath(rightStart, rightEnd, duration, 20);
    return new ClimbPath[] {leftPath, rightPath};
  }

  /**
   * Smooth ease-in-out function for more natural motion. Uses smoothstep function: 3t^2 - 2t^3
   *
   * @param t Input value [0, 1]
   * @return Smoothed value [0, 1]
   */
  private static double smoothEaseInOut(double t) {
    return t * t * (3.0 - 2.0 * t);
  }

  /**
   * Calculate the total distance of a path.
   *
   * @param path Path to measure
   * @return Total path length in meters
   */
  public static double calculatePathLength(ClimbPath path) {
    double totalLength = 0.0;
    List<Translation2d> waypoints = path.getWaypoints();

    for (int i = 0; i < waypoints.size() - 1; i++) {
      totalLength += waypoints.get(i).getDistance(waypoints.get(i + 1));
    }

    return totalLength;
  }

  /**
   * Executes a planned path in real-time.
   *
   * <p>Provides current position and velocity targets for Jacobian-based control. Used by
   * ClimbSubsystem.followWaypointPath()
   */
  public static class PathExecutor {
    private final ClimbPath leftPath;
    private final ClimbPath rightPath;
    private final Timer timer;
    private boolean isFinished;

    public PathExecutor(ClimbPath leftPath, ClimbPath rightPath) {
      this.leftPath = leftPath;
      this.rightPath = rightPath;
      this.timer = new Timer();
      this.isFinished = false;
    }

    /** Start executing the path */
    public void start() {
      timer.restart();
      isFinished = false;
    }

    /** Stop executing the path */
    public void stop() {
      timer.stop();
      isFinished = true;
    }

    /** Get current target positions - smoothly interpolates between waypoints based on time */
    public Translation2d[] getCurrentTargets() {
      double elapsed = timer.get();
      Translation2d leftTarget = getPositionAtTime(leftPath, elapsed);
      Translation2d rightTarget = getPositionAtTime(rightPath, elapsed);

      return new Translation2d[] {leftTarget, rightTarget};
    }

    /**
     * Get current velocities along the path (for velocity control). Uses trajectory's built-in
     * velocity if available, otherwise calculates as derivative.
     */
    public Translation2d[] getCurrentVelocities() {
      double elapsed = timer.get();

      // If we have trajectories, use their velocity data directly
      if (leftPath.hasTrajectory() && rightPath.hasTrajectory()) {
        double leftTime = Math.min(elapsed, leftPath.getTrajectory().getTotalTimeSeconds());
        double rightTime = Math.min(elapsed, rightPath.getTrajectory().getTotalTimeSeconds());

        Trajectory.State leftState = leftPath.getTrajectory().sample(leftTime);
        Trajectory.State rightState = rightPath.getTrajectory().sample(rightTime);

        // Convert velocity magnitude + heading to velocity vector
        double leftVelX =
            leftState.velocityMetersPerSecond * leftState.poseMeters.getRotation().getCos();
        double leftVelY =
            leftState.velocityMetersPerSecond * leftState.poseMeters.getRotation().getSin();
        double rightVelX =
            rightState.velocityMetersPerSecond * rightState.poseMeters.getRotation().getCos();
        double rightVelY =
            rightState.velocityMetersPerSecond * rightState.poseMeters.getRotation().getSin();

        return new Translation2d[] {
          new Translation2d(leftVelX, leftVelY), new Translation2d(rightVelX, rightVelY)
        };
      }

      // Fallback: numerical derivative (for legacy paths)
      double dt = 0.02; // 20ms lookahead
      double maxDuration = Math.max(leftPath.getTotalDuration(), rightPath.getTotalDuration());
      double clampedElapsed = Math.min(elapsed, maxDuration - dt);

      Translation2d leftCurrent = getPositionAtTime(leftPath, clampedElapsed);
      Translation2d leftFuture = getPositionAtTime(leftPath, clampedElapsed + dt);
      Translation2d rightCurrent = getPositionAtTime(rightPath, clampedElapsed);
      Translation2d rightFuture = getPositionAtTime(rightPath, clampedElapsed + dt);

      Translation2d leftVel = leftFuture.minus(leftCurrent).div(dt);
      Translation2d rightVel = rightFuture.minus(rightCurrent).div(dt);

      return new Translation2d[] {leftVel, rightVel};
    }

    /** Check if path execution is complete */
    public boolean isFinished() {
      return isFinished
          || timer.get() >= Math.max(leftPath.getTotalDuration(), rightPath.getTotalDuration());
    }

    /** Get current elapsed time */
    public double getElapsedTime() {
      return timer.get();
    }

    /** Get progress as a percentage (0.0 to 1.0) */
    public double getProgress() {
      return Math.min(
          1.0, timer.get() / Math.max(leftPath.getTotalDuration(), rightPath.getTotalDuration()));
    }
  }
}
