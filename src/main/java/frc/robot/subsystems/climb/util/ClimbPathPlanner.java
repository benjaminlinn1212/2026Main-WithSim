package frc.robot.subsystems.climb.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
   * Create a linear path between two positions. Duration is calculated automatically from distance
   * and the max velocity/acceleration constraints to ensure physically achievable motion.
   *
   * @param start Starting position
   * @param end Ending position
   * @param numWaypoints Number of intermediate waypoints to generate
   * @return Planned path
   */
  public static ClimbPath createLinearPath(
      Translation2d start, Translation2d end, int numWaypoints) {
    List<Translation2d> waypoints = new ArrayList<>();

    for (int i = 0; i <= numWaypoints; i++) {
      double t = (double) i / numWaypoints;
      Translation2d point = start.interpolate(end, t);
      waypoints.add(point);
    }

    // Calculate natural duration: use a trapezoidal motion profile
    // Time = distance / max_velocity, with extra time for acceleration/deceleration
    double distance = end.minus(start).getNorm();
    double maxVel = ClimbConstants.PATH_MAX_VELOCITY_MPS;
    double maxAccel = ClimbConstants.PATH_MAX_ACCELERATION_MPS2;
    double accelTime = maxVel / maxAccel; // time to reach max velocity
    double accelDistance = 0.5 * maxAccel * accelTime * accelTime; // distance during accel

    double duration;
    if (distance < 2.0 * accelDistance) {
      // Short path: never reaches max velocity (triangular profile)
      duration = 2.0 * Math.sqrt(distance / maxAccel);
    } else {
      // Long enough for trapezoidal profile
      double cruiseDistance = distance - 2.0 * accelDistance;
      duration = 2.0 * accelTime + cruiseDistance / maxVel;
    }

    return new ClimbPath(waypoints, duration, PathType.LINEAR);
  }

  /**
   * Create a smooth trajectory through multiple waypoints using WPILib's trajectory generation.
   * Automatically generates smooth quintic splines with proper velocity/acceleration constraints.
   * Duration is determined naturally by the trajectory constraints — no time-scaling is applied.
   *
   * @param waypoints List of waypoints to pass through (in climb 2D space: x=forward, y=up)
   * @param tension Controls how tight the curves are - NOT USED, kept for API compatibility
   * @param maintainEndVelocity If true, maintain velocity at end (for pulling); if false,
   *     decelerate to 0
   * @return Planned trajectory with time-based sampling
   */
  public static ClimbPath createMultiBezierPath(
      List<Translation2d> waypoints, double tension, boolean maintainEndVelocity) {
    if (waypoints.size() < 2) {
      throw new IllegalArgumentException("Need at least 2 waypoints for a path");
    }

    // For simple 2-point paths, use linear interpolation directly.
    // WPILib's trajectory generator is designed for drivetrain motions and can produce
    // curved/degenerate paths for purely vertical or horizontal lines in climb cartesian space.
    if (waypoints.size() == 2) {
      return createLinearPath(waypoints.get(0), waypoints.get(1), 20);
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
      return createLinearPath(waypoints.get(0), waypoints.get(waypoints.size() - 1), 20);
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

    // Fallback: trapezoidal position profile through waypoints (for linear paths without
    // trajectory). Uses the same accel/cruise/decel phases as the velocity envelope so that
    // position targets and velocity feedforward are perfectly consistent.
    List<Translation2d> waypoints = path.getWaypoints();

    if (elapsedTime <= 0) {
      return waypoints.get(0);
    }

    if (elapsedTime >= path.getTotalDuration()) {
      return waypoints.get(waypoints.size() - 1);
    }

    // Compute progress (0→1) using WPILib TrapezoidProfile. The profile plans accel/cruise/decel
    // over the straight-line distance, and we sample it at elapsedTime to get how far along the
    // path we should be. This keeps position targets consistent with the velocity derivative.
    double totalDistance = waypoints.get(0).getDistance(waypoints.get(waypoints.size() - 1));
    double progress = trapezoidalProgress(elapsedTime, totalDistance);

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
   * Compute normalized progress (0→1) along a trapezoidal velocity profile using WPILib's {@link
   * TrapezoidProfile}. The profile accelerates at maxAccel up to maxVel, cruises, then decelerates
   * to zero at the goal distance. Returns the fraction of total distance traveled at the given
   * time.
   *
   * @param elapsed Time since path start (seconds)
   * @param totalDistance Total path distance (meters)
   * @return Fraction of total path distance completed (0.0 to 1.0)
   */
  private static double trapezoidalProgress(double elapsed, double totalDistance) {
    if (totalDistance <= 0 || elapsed <= 0) return 0.0;

    TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(
            ClimbConstants.PATH_MAX_VELOCITY_MPS, ClimbConstants.PATH_MAX_ACCELERATION_MPS2);
    TrapezoidProfile profile = new TrapezoidProfile(constraints);

    TrapezoidProfile.State initial = new TrapezoidProfile.State(0.0, 0.0);
    TrapezoidProfile.State goal = new TrapezoidProfile.State(totalDistance, 0.0);

    TrapezoidProfile.State state = profile.calculate(elapsed, initial, goal);
    return Math.max(0.0, Math.min(1.0, state.position / totalDistance));
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

        // At or past the end of the trajectory, command zero velocity to avoid overshoot.
        double maxTotal = Math.max(leftTotal, rightTotal);
        if (elapsed >= maxTotal - dt) {
          return new Translation2d[] {new Translation2d(), new Translation2d()};
        }

        Translation2d leftPos =
            leftPath.getTrajectory().sample(elapsed).poseMeters.getTranslation();
        Translation2d leftPosNext =
            leftPath.getTrajectory().sample(elapsed + dt).poseMeters.getTranslation();
        Translation2d rightPos =
            rightPath.getTrajectory().sample(elapsed).poseMeters.getTranslation();
        Translation2d rightPosNext =
            rightPath.getTrajectory().sample(elapsed + dt).poseMeters.getTranslation();

        Translation2d leftVel = leftPosNext.minus(leftPos).div(dt);
        Translation2d rightVel = rightPosNext.minus(rightPos).div(dt);

        return new Translation2d[] {leftVel, rightVel};
      }

      // Fallback: numerical derivative of the trapezoidal position profile.
      // getPositionAtTime() already uses trapezoidalProgress() for linear paths, so the
      // derivative naturally produces a trapezoidal velocity shape (accel → cruise → decel).
      // No explicit envelope is needed — applying one would double-ramp the velocity.
      double dt = 0.02; // 20ms lookahead
      double maxDuration = Math.max(leftPath.getTotalDuration(), rightPath.getTotalDuration());

      // At or past the end of the path, command zero velocity so the arm doesn't overshoot
      // into the MotionMagic hold transition. Previously, clamping to maxDuration-dt always
      // produced a small residual velocity from the last finite-difference sample.
      if (elapsed >= maxDuration - dt || maxDuration <= dt) {
        return new Translation2d[] {new Translation2d(), new Translation2d()};
      }

      Translation2d leftCurrent = getPositionAtTime(leftPath, elapsed);
      Translation2d leftFuture = getPositionAtTime(leftPath, elapsed + dt);
      Translation2d rightCurrent = getPositionAtTime(rightPath, elapsed);
      Translation2d rightFuture = getPositionAtTime(rightPath, elapsed + dt);

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
