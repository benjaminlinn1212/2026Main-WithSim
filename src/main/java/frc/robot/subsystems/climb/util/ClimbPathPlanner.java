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

  /** Path interpolation types. */
  public enum PathType {
    /** Linear interpolation - straight line between points. */
    LINEAR,
    /** Smooth interpolation using quintic splines. */
    SMOOTH
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

    // For simple 2-point paths, use linear interpolation directly.
    // WPILib's trajectory generator is designed for drivetrain motions and can produce
    // curved/degenerate paths for purely vertical or horizontal lines in climb cartesian space.
    if (waypoints.size() == 2) {
      return createLinearPath(waypoints.get(0), waypoints.get(1), duration, 20);
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

    // Check for failed trajectory generation. WPILib returns kDoNothingTrajectory (1 state
    // at time=0, position=(0,0)) when quintic splines fail with MalformedSplineException.
    // This happens with close/collinear waypoints. Fall back to linear interpolation.
    if (trajectory.getStates().size() <= 1 || trajectory.getTotalTimeSeconds() <= 1e-6) {
      return createLinearPath(waypoints.get(0), waypoints.get(waypoints.size() - 1), duration, 20);
    }

    // If a specific duration is requested, time-scale the trajectory
    if (duration > 1e-6) {
      trajectory = scaleTrajectoryDuration(trajectory, duration);
    }

    // Extract waypoints from trajectory for visualization
    List<Translation2d> trajectoryPoints =
        trajectory.getStates().stream()
            .map(state -> state.poseMeters.getTranslation())
            .collect(Collectors.toList());

    // Use trajectory's actual duration (may differ from requested)
    double actualDuration = trajectory.getTotalTimeSeconds();

    return new ClimbPath(trajectoryPoints, actualDuration, PathType.SMOOTH, trajectory);
  }

  /** Scale trajectory timing to match the desired duration (keeps path geometry). */
  private static Trajectory scaleTrajectoryDuration(Trajectory trajectory, double desiredSeconds) {
    double actualSeconds = trajectory.getTotalTimeSeconds();
    if (actualSeconds <= 1e-6 || desiredSeconds <= 1e-6) {
      return trajectory;
    }

    double timeScale = desiredSeconds / actualSeconds;
    List<Trajectory.State> scaledStates = new ArrayList<>();
    for (Trajectory.State state : trajectory.getStates()) {
      Trajectory.State scaled = new Trajectory.State();
      scaled.timeSeconds = state.timeSeconds * timeScale;
      scaled.velocityMetersPerSecond = state.velocityMetersPerSecond / timeScale;
      scaled.accelerationMetersPerSecondSq =
          state.accelerationMetersPerSecondSq / (timeScale * timeScale);
      scaled.poseMeters = state.poseMeters;
      scaled.curvatureRadPerMeter = state.curvatureRadPerMeter;
      scaledStates.add(scaled);
    }
    return new Trajectory(scaledStates);
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
   * Executes a planned path in real-time.
   *
   * <p>Provides current position and velocity targets for Jacobian-based control. Used by
   * ClimbSubsystem.runPath()
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

      // If we have trajectories, compute velocity by sampling translation derivative
      if (leftPath.hasTrajectory() && rightPath.hasTrajectory()) {
        double leftTotal = leftPath.getTrajectory().getTotalTimeSeconds();
        double rightTotal = rightPath.getTrajectory().getTotalTimeSeconds();
        double dt = 0.01;

        double leftTime = Math.min(elapsed, leftTotal);
        double rightTime = Math.min(elapsed, rightTotal);
        double leftNext = Math.min(leftTime + dt, leftTotal);
        double rightNext = Math.min(rightTime + dt, rightTotal);

        Translation2d leftPos =
            leftPath.getTrajectory().sample(leftTime).poseMeters.getTranslation();
        Translation2d leftPosNext =
            leftPath.getTrajectory().sample(leftNext).poseMeters.getTranslation();
        Translation2d rightPos =
            rightPath.getTrajectory().sample(rightTime).poseMeters.getTranslation();
        Translation2d rightPosNext =
            rightPath.getTrajectory().sample(rightNext).poseMeters.getTranslation();

        Translation2d leftVel = leftPosNext.minus(leftPos).div(Math.max(1e-6, leftNext - leftTime));
        Translation2d rightVel =
            rightPosNext.minus(rightPos).div(Math.max(1e-6, rightNext - rightTime));

        return new Translation2d[] {leftVel, rightVel};
      }

      // Fallback: numerical derivative (for legacy paths)
      double dt = 0.02; // 20ms lookahead
      double maxDuration = Math.max(leftPath.getTotalDuration(), rightPath.getTotalDuration());
      if (maxDuration <= dt) {
        return new Translation2d[] {new Translation2d(), new Translation2d()};
      }
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
