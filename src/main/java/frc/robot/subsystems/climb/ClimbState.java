package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ClimbConstants;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * CLIMB STATES - Pre-planned waypoints for each climb phase.
 *
 * <p>Naming convention: VERB_LEVEL (e.g., EXTEND_L1 = extending arm to L1 bar)
 *
 * <p>Teleop sequence: STOWED -> EXTEND_L1 -> RETRACT_L1 -> EXTEND_L2 -> RETRACT_L2 -> EXTEND_L3 ->
 * RETRACT_L3
 *
 * <p>Edit waypoints below. Translation2d(x, y) in meters.
 */
public enum ClimbState {

  // ── Starting position ──
  STOWED(
      "Stowed",
      new Translation2d(
          ClimbConstants.START_POSITION_X_METERS, ClimbConstants.START_POSITION_Y_METERS),
      null,
      1.0,
      false),

  // ── L1 sequence ──

  /** Extend arm up to L1 bar - curved path */
  EXTEND_L1(
      "Extending to L1",
      new Translation2d(0.40, 0.55),
      List.of(
          new Translation2d(
              ClimbConstants.START_POSITION_X_METERS, ClimbConstants.START_POSITION_Y_METERS),
          new Translation2d(0.395, 0.49),
          new Translation2d(0.40, 0.55)),
      2.0,
      false),

  /** Retract cables to pull robot up on L1 bar (teleop) */
  RETRACT_L1(
      "Retracting on L1",
      new Translation2d(0.38, 0.50),
      List.of(new Translation2d(0.40, 0.55), new Translation2d(0.38, 0.50)),
      2.5,
      true),

  // ── L2 sequence ──

  /** Extend arm from L1 up to L2 bar - curved path */
  EXTEND_L2(
      "Extending to L2",
      new Translation2d(0.42, 0.58),
      List.of(
          new Translation2d(0.38, 0.50),
          new Translation2d(0.40, 0.54),
          new Translation2d(0.42, 0.58)),
      2.5,
      false),

  /** Retract cables to pull robot up on L2 bar */
  RETRACT_L2(
      "Retracting on L2",
      new Translation2d(0.39, 0.52),
      List.of(new Translation2d(0.42, 0.58), new Translation2d(0.39, 0.52)),
      2.5,
      true),

  // ── L3 sequence ──

  /** Extend arm from L2 up to L3 bar - curved path */
  EXTEND_L3(
      "Extending to L3",
      new Translation2d(0.43, 0.60),
      List.of(
          new Translation2d(0.39, 0.52),
          new Translation2d(0.41, 0.56),
          new Translation2d(0.43, 0.60)),
      2.5,
      false),

  /** Retract cables to pull robot up on L3 bar (final climb position) */
  RETRACT_L3(
      "Retracting on L3",
      new Translation2d(0.40, 0.54),
      List.of(new Translation2d(0.43, 0.60), new Translation2d(0.40, 0.54)),
      3.0,
      true),

  // ── Special states ──
  MANUAL("Manual", new Translation2d(0.0, 0.0), null, 0.0, false),
  EMERGENCY_STOP("E-Stop", new Translation2d(0.0, 0.0), null, 0.0, false);

  // State data
  private final String name;
  private final Translation2d targetPosition;
  private final List<Translation2d> prePlannedWaypoints;
  private final double defaultDuration;
  private final boolean isPulling;

  ClimbState(
      String name,
      Translation2d target,
      List<Translation2d> waypoints,
      double duration,
      boolean isPulling) {
    this.name = name;
    this.targetPosition = target;
    this.prePlannedWaypoints = waypoints;
    this.defaultDuration = duration;
    this.isPulling = isPulling;
  }

  public String getName() {
    return name;
  }

  /** Get target end-effector position (symmetric for both sides). */
  public Translation2d getTargetPosition() {
    return targetPosition;
  }

  /** Check if this state has a pre-planned path */
  public boolean hasPrePlannedPath() {
    return prePlannedWaypoints != null && !prePlannedWaypoints.isEmpty();
  }

  /** Get pre-planned waypoints for this state's path (symmetric for both sides). */
  public List<Translation2d> getPrePlannedWaypoints() {
    return prePlannedWaypoints;
  }

  /** Get reversed waypoints for going backwards along the path. */
  public List<Translation2d> getReversedWaypoints() {
    if (prePlannedWaypoints == null || prePlannedWaypoints.isEmpty()) {
      return null;
    }
    List<Translation2d> reversed = new ArrayList<>(prePlannedWaypoints);
    Collections.reverse(reversed);
    return reversed;
  }

  /** Get default duration for this state's path */
  public double getDefaultDuration() {
    return defaultDuration;
  }

  /** Check if this is a pulling path (needs extra feedforward for load) */
  public boolean isPulling() {
    return isPulling;
  }

  /** Get next state in climb sequence. Returns null if at terminal state. */
  public ClimbState getNextState() {
    switch (this) {
      case STOWED:
        return EXTEND_L1;
      case EXTEND_L1:
        return RETRACT_L1;
      case RETRACT_L1:
        return EXTEND_L2;
      case EXTEND_L2:
        return RETRACT_L2;
      case RETRACT_L2:
        return EXTEND_L3;
      case EXTEND_L3:
        return RETRACT_L3;
      default:
        return null; // Terminal states (RETRACT_L3, MANUAL, EMERGENCY_STOP)
    }
  }

  /** Get previous state in climb sequence. Returns null if at initial state. */
  public ClimbState getPreviousState() {
    switch (this) {
      case EXTEND_L1:
        return STOWED;
      case RETRACT_L1:
        return EXTEND_L1;
      case EXTEND_L2:
        return RETRACT_L1;
      case RETRACT_L2:
        return EXTEND_L2;
      case EXTEND_L3:
        return RETRACT_L2;
      case RETRACT_L3:
        return EXTEND_L3;
      default:
        return null;
    }
  }

  @Override
  public String toString() {
    return name;
  }
}
