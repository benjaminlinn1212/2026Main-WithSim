package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * CLIMB STATES - Pre-planned waypoints for each climb phase
 *
 * <p>Edit waypoints below. Translation2d(x, y) in meters. Left side automatically mirrors right
 * side.
 */
public enum ClimbState {

  // STOWED - Starting position
  STOWED("Stowed", new Translation2d(0.0, 0.0), null, 1.0),

  // REACH_L1 - Reach up to L1 bar (initial reach from stowed)
  REACH_L1(
      "Reach L1",
      new Translation2d(0.15, 0.7),
      List.of(new Translation2d(0.0, 0.0), new Translation2d(0.15, 0.7)),
      2.0),

  // PULL_L1_AUTO - Pull up on L1 for autonomous
  PULL_L1_AUTO(
      "Pull L1 Auto",
      new Translation2d(0.12, 0.55),
      List.of(new Translation2d(0.15, 0.7), new Translation2d(0.12, 0.55)),
      2.0),

  // DROP_L1_AUTO - Drop from L1 for autonomous transition
  DROP_L1_AUTO(
      "Drop L1 Auto",
      new Translation2d(0.15, 0.65),
      List.of(new Translation2d(0.12, 0.55), new Translation2d(0.15, 0.65)),
      1.5),

  // PULL_L1 - Full pull on L1 (teleop)
  PULL_L1(
      "Pull L1",
      new Translation2d(0.08, 0.45),
      List.of(new Translation2d(0.15, 0.7), new Translation2d(0.08, 0.45)),
      2.5),

  // REACH_L2 - Reach L2 from L1 (shared position for L2/L3)
  REACH_L2(
      "Reach L2",
      new Translation2d(0.2, 0.8),
      List.of(new Translation2d(0.08, 0.45), new Translation2d(0.2, 0.8)),
      2.5),

  // PULL_L2 - Pull up on L2 (shared position for L2/L3)
  PULL_L2(
      "Pull L2",
      new Translation2d(0.1, 0.5),
      List.of(new Translation2d(0.2, 0.8), new Translation2d(0.1, 0.5)),
      2.5),

  // REACH_L3 - Reach L3 from L2 (shared position for L2/L3)
  REACH_L3(
      "Reach L3",
      new Translation2d(0.2, 0.8),
      List.of(new Translation2d(0.1, 0.5), new Translation2d(0.2, 0.8)),
      2.5),

  // PULL_L3 - Pull up on L3 (shared position for L2/L3, final)
  PULL_L3(
      "Pull L3",
      new Translation2d(0.1, 0.5),
      List.of(new Translation2d(0.2, 0.8), new Translation2d(0.1, 0.5)),
      3.0),

  // MANUAL & EMERGENCY_STOP - No paths
  MANUAL("Manual", new Translation2d(0.0, 0.0), null, 0.0),
  EMERGENCY_STOP("Emergency Stop", new Translation2d(0.0, 0.0), null, 0.0);

  // State data
  private final String name;
  private final Translation2d targetPosition;
  private final List<Translation2d> prePlannedWaypoints;
  private final double defaultDuration;

  ClimbState(String name, Translation2d target, List<Translation2d> waypoints, double duration) {
    this.name = name;
    this.targetPosition = target;
    this.prePlannedWaypoints = waypoints;
    this.defaultDuration = duration;
  }

  public String getName() {
    return name;
  }

  /** Get target position for left side (mirrors right) */
  public Translation2d getLeftTargetPosition() {
    return targetPosition; // Same as right (symmetric)
  }

  /** Get target position for right side */
  public Translation2d getRightTargetPosition() {
    return targetPosition;
  }

  /** Check if this state has a pre-planned path */
  public boolean hasPrePlannedPath() {
    return prePlannedWaypoints != null && !prePlannedWaypoints.isEmpty();
  }

  /** Get pre-planned waypoints (right side, left mirrors) */
  public List<Translation2d> getPrePlannedWaypoints() {
    return prePlannedWaypoints;
  }

  /** Get default duration for this state's path */
  public double getDefaultDuration() {
    return defaultDuration;
  }

  /** Get next state in climb sequence */
  public ClimbState getNextState() {
    switch (this) {
      case STOWED:
        return REACH_L1;
      case REACH_L1:
        return PULL_L1_AUTO;
      case PULL_L1_AUTO:
        return DROP_L1_AUTO;
      case DROP_L1_AUTO:
        return PULL_L1;
      case PULL_L1:
        return REACH_L2;
      case REACH_L2:
        return PULL_L2;
      case PULL_L2:
        return REACH_L3;
      case REACH_L3:
        return PULL_L3;
      default:
        return null; // Terminal states
    }
  }

  /** Get previous state in climb sequence */
  public ClimbState getPreviousState() {
    switch (this) {
      case REACH_L1:
        return STOWED;
      case PULL_L1_AUTO:
        return REACH_L1;
      case DROP_L1_AUTO:
        return PULL_L1_AUTO;
      case PULL_L1:
        return DROP_L1_AUTO;
      case REACH_L2:
        return PULL_L1;
      case PULL_L2:
        return REACH_L2;
      case REACH_L3:
        return PULL_L2;
      case PULL_L3:
        return REACH_L3;
      default:
        return null;
    }
  }

  @Override
  public String toString() {
    return name;
  }
}
