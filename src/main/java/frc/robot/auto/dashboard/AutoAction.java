// Copyright (c) 2026 FRC Team 0 (Amped)
// Dashboard-driven autonomous system — Action types

package frc.robot.auto.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto.dashboard.FieldConstants.ClimbLevel;
import frc.robot.auto.dashboard.FieldConstants.IntakeLocation;
import frc.robot.auto.dashboard.FieldConstants.ScoringLocation;

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
    CLIMB,
    WAIT
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
   * Drive from the current pose to a HUB shooting position, then execute the scoring sequence (aim
   * + shoot FUEL).
   */
  public static final class ScoreAt extends AutoAction {
    private final ScoringLocation location;
    private final boolean shootWhileMoving;

    public ScoreAt(ScoringLocation location, boolean shootWhileMoving) {
      super(Type.SCORE_AT);
      this.location = location;
      this.shootWhileMoving = shootWhileMoving;
    }

    public ScoringLocation getLocation() {
      return location;
    }

    public boolean isShootWhileMoving() {
      return shootWhileMoving;
    }

    @Override
    public String describe() {
      return "Score FUEL at "
          + location.name()
          + (shootWhileMoving ? " (shoot-while-driving)" : "");
    }

    @Override
    public double estimatedDuration() {
      return FieldConstants.SCORE_DURATION;
    }

    @Override
    public Pose2d getTargetPose() {
      return location.getPose();
    }
  }

  /** Drive from the current pose to an intake location and pick up FUEL. */
  public static final class IntakeAt extends AutoAction {
    private final IntakeLocation location;

    public IntakeAt(IntakeLocation location) {
      super(Type.INTAKE_AT);
      this.location = location;
    }

    public IntakeLocation getLocation() {
      return location;
    }

    @Override
    public String describe() {
      return "Intake FUEL at " + location.name();
    }

    @Override
    public double estimatedDuration() {
      return FieldConstants.INTAKE_DURATION;
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

    public Climb(ClimbLevel climbLevel) {
      super(Type.CLIMB);
      this.climbLevel = climbLevel;
    }

    public ClimbLevel getClimbLevel() {
      return climbLevel;
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
      return climbLevel.getPose();
    }
  }

  /** Wait a fixed duration (e.g., for strategy or to let a partner pass). */
  public static final class Wait extends AutoAction {
    private final double seconds;
    private final String reason;

    public Wait(double seconds, String reason) {
      super(Type.WAIT);
      this.seconds = seconds;
      this.reason = reason;
    }

    public double getSeconds() {
      return seconds;
    }

    public String getReason() {
      return reason;
    }

    @Override
    public String describe() {
      return String.format("Wait %.1fs (%s)", seconds, reason);
    }

    @Override
    public double estimatedDuration() {
      return seconds;
    }

    @Override
    public Pose2d getTargetPose() {
      return null;
    }
  }

  /**
   * Score preloaded FUEL at the first shooting position. Similar to ScoreAt but the robot is
   * assumed to already be positioned.
   */
  public static final class ScorePreload extends AutoAction {
    private final ScoringLocation location;

    public ScorePreload(ScoringLocation location) {
      super(Type.SCORE_PRELOAD);
      this.location = location;
    }

    public ScoringLocation getLocation() {
      return location;
    }

    @Override
    public String describe() {
      return "Score preloaded FUEL at " + location.name();
    }

    @Override
    public double estimatedDuration() {
      return FieldConstants.SCORE_DURATION * 0.7;
    }

    @Override
    public Pose2d getTargetPose() {
      return location.getPose();
    }
  }
}
