// Copyright (c) 2026 FRC Team 10922 (Amped)
// Dashboard-driven autonomous system — Action types

package frc.robot.auto.dashboard;

import static frc.robot.auto.dashboard.AutoTuning.*;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto.dashboard.FieldConstants.ClimbLevel;
import frc.robot.auto.dashboard.FieldConstants.ClimbPose;
import frc.robot.auto.dashboard.FieldConstants.IntakeLocation;
import frc.robot.auto.dashboard.FieldConstants.ScoringWaypoint;

/**
 * Represents a single step in a planned autonomous sequence. The {@link AutoPlanner} produces a
 * list of these; the {@link AutoCommandBuilder} converts each into a runnable Command.
 *
 * <p>Each subclass represents a specific action type. Use {@link #getType()} and instanceof checks
 * to dispatch. (Java 17 compatible — no sealed/pattern-matching.)
 */
public abstract class AutoAction {

  public enum Type {
    SET_START_POSE,
    SCORE_AT,
    SCORE_PRELOAD,
    INTAKE_AT,
    DRIVE_TO,
    CLIMB
  }

  private final Type type;

  protected AutoAction(Type type) {
    this.type = type;
  }

  public Type getType() {
    return type;
  }

  /** Human-readable description for dashboard preview. */
  public abstract String describe();

  /** Estimated duration of this action in seconds (for time-budgeting). */
  public abstract double estimatedDuration();

  /** Get the target pose of this action, or null if it doesn't involve movement. */
  public abstract Pose2d getTargetPose();

  // ===== Concrete Action Types =====

  /**
   * Set the robot's initial pose (odometry reset). Always the first action if the planner knows the
   * start pose.
   */
  public static final class SetStartPose extends AutoAction {
    private final Pose2d pose;

    public SetStartPose(Pose2d pose) {
      super(Type.SET_START_POSE);
      this.pose = pose;
    }

    public Pose2d getPose() {
      return pose;
    }

    @Override
    public String describe() {
      return String.format(
          "Reset pose to (%.1f, %.1f, %.0f°)",
          pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    @Override
    public double estimatedDuration() {
      return 0.0;
    }

    @Override
    public Pose2d getTargetPose() {
      return pose;
    }
  }

  /**
   * Score FUEL. Two modes:
   *
   * <ul>
   *   <li><b>Stop-and-shoot:</b> Drive to the HUB shooting waypoint ({@code location}), stop, aim,
   *       shoot. The scoring waypoint is the target pose.
   *   <li><b>Shoot-while-driving (SWD):</b> The robot does NOT detour to a scoring waypoint.
   *       Instead, it drives directly to {@code swdDestination} (typically the next intake
   *       location) while the turret tracks and shoots the HUB in real-time mid-transit.
   * </ul>
   */
  public static final class ScoreAt extends AutoAction {
    private final ScoringWaypoint location;
    private final boolean shootWhileMoving;
    /**
     * The pose the robot actually drives toward during SWD (e.g., the next intake). Null when
     * stop-and-shoot is used — in that case the robot drives to {@code location.toPose()}.
     */
    private final Pose2d swdDestination;

    /** Stop-and-shoot constructor. */
    public ScoreAt(ScoringWaypoint location) {
      this(location, false, null);
    }

    /** SWD constructor — destination is where the robot drives while shooting. */
    public ScoreAt(ScoringWaypoint location, boolean shootWhileMoving, Pose2d swdDestination) {
      super(Type.SCORE_AT);
      this.location = location;
      this.shootWhileMoving = shootWhileMoving;
      this.swdDestination = swdDestination;
    }

    public ScoringWaypoint getLocation() {
      return location;
    }

    public boolean isShootWhileMoving() {
      return shootWhileMoving;
    }

    /**
     * The actual drive destination during SWD (next intake pose). Returns null for stop-and-shoot
     * cycles.
     */
    public Pose2d getSwdDestination() {
      return swdDestination;
    }

    @Override
    public String describe() {
      return "Score FUEL at "
          + location.name()
          + (shootWhileMoving ? " (shoot-while-driving)" : "");
    }

    @Override
    public double estimatedDuration() {
      return shootWhileMoving ? SWD_SCORE_DURATION : STOP_AND_SHOOT_DURATION;
    }

    @Override
    public Pose2d getTargetPose() {
      // SWD: the robot ends at the SWD destination (next intake), not the scoring waypoint
      return shootWhileMoving && swdDestination != null ? swdDestination : location.toPose();
    }
  }

  /**
   * Drive from the current pose to an intake location and pick up FUEL.
   *
   * <p>{@code visitNumber} tracks how many times this specific intake location has been visited
   * (1-based). On the first visit (visitNumber=1), the robot drives to the nominal intake pose. On
   * subsequent visits (visitNumber≥2), the robot drives deeper past the original waypoint to reach
   * FUEL that wasn't collected on previous passes. The deeper offset is computed at runtime by
   * {@link AutoCommandBuilder#buildIntakeAt}.
   */
  public static final class IntakeAt extends AutoAction {
    private final IntakeLocation location;
    private final int visitNumber;

    public IntakeAt(IntakeLocation location) {
      this(location, 1);
    }

    public IntakeAt(IntakeLocation location, int visitNumber) {
      super(Type.INTAKE_AT);
      this.location = location;
      this.visitNumber = visitNumber;
    }

    public IntakeLocation getLocation() {
      return location;
    }

    /** How many times this intake location has been visited (1-based). */
    public int getVisitNumber() {
      return visitNumber;
    }

    @Override
    public String describe() {
      if (visitNumber > 1) {
        return "Intake FUEL at " + location.name() + " (visit #" + visitNumber + ", deeper)";
      }
      return "Intake FUEL at " + location.name();
    }

    @Override
    public double estimatedDuration() {
      return 0.0; // No stop — robot drives through at full speed
    }

    @Override
    public Pose2d getTargetPose() {
      return location.getPose();
    }
  }

  /**
   * Drive from the current pose to an arbitrary target pose. Used for lane-constrained traversal
   * via waypoints, or navigating to a climb position.
   */
  public static final class DriveTo extends AutoAction {
    private final Pose2d target;
    private final String label;

    public DriveTo(Pose2d target, String label) {
      super(Type.DRIVE_TO);
      this.target = target;
      this.label = label;
    }

    public Pose2d getTarget() {
      return target;
    }

    public String getLabel() {
      return label;
    }

    @Override
    public String describe() {
      return String.format("Drive to %s (%.1f, %.1f)", label, target.getX(), target.getY());
    }

    @Override
    public double estimatedDuration() {
      return 1.0;
    }

    @Override
    public Pose2d getTargetPose() {
      return target;
    }
  }

  /** Execute the TOWER climb sequence. Usually the last action. */
  public static final class Climb extends AutoAction {
    private final ClimbLevel climbLevel;
    private final ClimbPose climbPose;

    public Climb(ClimbLevel climbLevel, ClimbPose climbPose) {
      super(Type.CLIMB);
      this.climbLevel = climbLevel;
      this.climbPose = climbPose;
    }

    public ClimbLevel getClimbLevel() {
      return climbLevel;
    }

    public ClimbPose getClimbPose() {
      return climbPose;
    }

    @Override
    public String describe() {
      return "Climb TOWER " + climbLevel.name() + " (" + climbLevel.teleopPoints + " pts)";
    }

    @Override
    public double estimatedDuration() {
      return climbLevel.estimatedClimbDuration;
    }

    @Override
    public Pose2d getTargetPose() {
      return climbPose.getPose();
    }
  }

  /**
   * Score preloaded FUEL. Two modes:
   *
   * <ul>
   *   <li><b>Stop-and-shoot:</b> Drive to scoring waypoint, stop, aim, fire.
   *   <li><b>Shoot-while-driving (SWD):</b> Drive directly to {@code swdDestination} (the first
   *       intake location) while dumping preloaded FUEL mid-transit through the HUB zone.
   * </ul>
   */
  public static final class ScorePreload extends AutoAction {
    private final ScoringWaypoint location;
    private final boolean shootWhileMoving;
    private final Pose2d swdDestination;

    /** Stop-and-shoot constructor. */
    public ScorePreload(ScoringWaypoint location) {
      this(location, false, null);
    }

    /** SWD constructor — destination is where the robot drives while shooting preload. */
    public ScorePreload(ScoringWaypoint location, boolean shootWhileMoving, Pose2d swdDestination) {
      super(Type.SCORE_PRELOAD);
      this.location = location;
      this.shootWhileMoving = shootWhileMoving;
      this.swdDestination = swdDestination;
    }

    public ScoringWaypoint getLocation() {
      return location;
    }

    public boolean isShootWhileMoving() {
      return shootWhileMoving;
    }

    public Pose2d getSwdDestination() {
      return swdDestination;
    }

    @Override
    public String describe() {
      return "Score preloaded FUEL at "
          + location.name()
          + (shootWhileMoving ? " (shoot-while-driving)" : "");
    }

    @Override
    public double estimatedDuration() {
      // Preload has fewer FUEL so it's slightly faster than a full cycle score
      return (shootWhileMoving ? SWD_SCORE_DURATION : STOP_AND_SHOOT_DURATION) * 0.7;
    }

    @Override
    public Pose2d getTargetPose() {
      return shootWhileMoving && swdDestination != null ? swdDestination : location.toPose();
    }
  }
}
