package frc.robot.subsystems.climb.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ClimbConstants;
import java.util.ArrayList;
import java.util.List;

/**
 * Path Planning - Interpolates between waypoints to create smooth paths Path planning utilities for
 * smooth climb motions.
 *
 * <p>Provides methods to: - Generate interpolated paths between waypoints - Create trapezoidal
 * velocity profiles - Calculate intermediate positions along a path - Validate paths are within
 * workspace
 */
public class ClimbPathPlanner {

  /** Represents a planned path for one side of the climb mechanism. */
  public static class ClimbPath {
    private final List<Translation2d> waypoints;
    private final double totalDuration;
    private final PathType pathType;

    public ClimbPath(List<Translation2d> waypoints, double totalDuration, PathType pathType) {
      this.waypoints = waypoints;
      this.totalDuration = totalDuration;
      this.pathType = pathType;
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
  }

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
   * Create a multi-segment Bezier path through multiple waypoints. Automatically generates control
   * points for smooth transitions.
   *
   * @param waypoints List of waypoints to pass through
   * @param tension Controls how tight the curves are (0.0 = tight, 1.0 = loose)
   * @param duration Total time to complete the path (seconds)
   * @return Planned multi-segment Bezier path
   */
  public static ClimbPath createMultiBezierPath(
      List<Translation2d> waypoints, double tension, double duration) {
    if (waypoints.size() < 2) {
      throw new IllegalArgumentException("Need at least 2 waypoints for a Bezier path");
    }

    List<Translation2d> bezierPath = new ArrayList<>();

    // For each segment between waypoints
    for (int i = 0; i < waypoints.size() - 1; i++) {
      Translation2d p0 = waypoints.get(i);
      Translation2d p3 = waypoints.get(i + 1);

      // Auto-generate control points based on waypoint tangents
      Translation2d direction = p3.minus(p0).times(tension);
      Translation2d p1 = p0.plus(direction.times(0.33));
      Translation2d p2 = p3.minus(direction.times(0.33));

      // Sample the Bezier segment
      int samplesPerSegment = 20;
      for (int j = 0; j < samplesPerSegment; j++) {
        double t = (double) j / samplesPerSegment;
        Translation2d point = calculateCubicBezier(p0, p1, p2, p3, t);
        bezierPath.add(point);
      }
    }

    // Add final waypoint
    bezierPath.add(waypoints.get(waypoints.size() - 1));

    return new ClimbPath(bezierPath, duration, PathType.SMOOTH);
  }

  /**
   * Calculate a point on a cubic Bezier curve. Formula: B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ +
   * t³P₃
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

    // Bezier formula: (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
    Translation2d point = p0.times(uuu);
    point = point.plus(p1.times(3 * uu * t));
    point = point.plus(p2.times(3 * u * tt));
    point = point.plus(p3.times(ttt));

    return point;
  }

  /**
   * Create a path with trapezoidal velocity profile for smooth acceleration/deceleration.
   *
   * @param start Starting position
   * @param end Ending position
   * @param maxVelocity Maximum velocity (m/s)
   * @param maxAcceleration Maximum acceleration (m/s^2)
   * @return Planned path with trapezoidal profile
   */
  public static ClimbPath createTrapezoidPath(
      Translation2d start, Translation2d end, double maxVelocity, double maxAcceleration) {

    double distance = start.getDistance(end);

    // Create a 1D trapezoidal profile for the distance
    TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    TrapezoidProfile.State initialState = new TrapezoidProfile.State(0, 0);
    TrapezoidProfile.State goalState = new TrapezoidProfile.State(distance, 0);
    TrapezoidProfile profile = new TrapezoidProfile(constraints);

    double duration = profile.totalTime();

    // Sample the profile to create waypoints
    List<Translation2d> waypoints = new ArrayList<>();
    int numSamples = (int) Math.ceil(duration * 50); // 50 Hz sampling
    numSamples = Math.max(numSamples, 10); // At least 10 points

    TrapezoidProfile.State currentState = initialState;
    for (int i = 0; i <= numSamples; i++) {
      double dt = i == 0 ? 0.0 : duration / numSamples;
      currentState = profile.calculate(dt, currentState, goalState);
      double progress = distance > 0 ? currentState.position / distance : 1.0;
      Translation2d point = start.interpolate(end, progress);
      waypoints.add(point);
    }

    return new ClimbPath(waypoints, duration, PathType.TRAPEZOID);
  }

  /**
   * Get the target position at a specific time along the path.
   *
   * @param path The path to sample
   * @param elapsedTime Time since path started (seconds)
   * @return Target position at this time
   */
  public static Translation2d getPositionAtTime(ClimbPath path, double elapsedTime) {
    if (elapsedTime <= 0) {
      return path.getWaypoints().get(0);
    }

    if (elapsedTime >= path.getTotalDuration()) {
      return path.getWaypoints().get(path.getWaypoints().size() - 1);
    }

    // Calculate which waypoint we should be at
    double progress = elapsedTime / path.getTotalDuration();
    int waypointCount = path.getWaypoints().size();
    double floatIndex = progress * (waypointCount - 1);
    int index = (int) Math.floor(floatIndex);
    double t = floatIndex - index;

    // Interpolate between waypoints
    if (index >= waypointCount - 1) {
      return path.getWaypoints().get(waypointCount - 1);
    }

    Translation2d current = path.getWaypoints().get(index);
    Translation2d next = path.getWaypoints().get(index + 1);
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

  /** Helper class to track path execution state. */
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

    /** Get current target positions for both sides */
    public Translation2d[] getCurrentTargets() {
      double elapsed = timer.get();
      Translation2d leftTarget = getPositionAtTime(leftPath, elapsed);
      Translation2d rightTarget = getPositionAtTime(rightPath, elapsed);
      return new Translation2d[] {leftTarget, rightTarget};
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
      double maxDuration = Math.max(leftPath.getTotalDuration(), rightPath.getTotalDuration());
      return Math.min(1.0, timer.get() / maxDuration);
    }
  }
}
