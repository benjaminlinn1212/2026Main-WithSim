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
 * <p>Teleop sequence (one POV-Right press per step): STOWED -> EXTEND_L1 -> RETRACT_L1 ->
 * RELEASE_ANGLE_L1 -> RELEASE_HARDSTOP_L1 -> EXTEND_L2 -> RETRACT_L2 -> STOW_SERVOS_L2 ->
 * RELEASE_ANGLE_L2 -> RELEASE_HARDSTOP_L2 -> EXTEND_L3 -> RETRACT_L3
 *
 * <p>Edit waypoints below. Translation2d(x, y) in meters.
 */
public enum ClimbState {

  // ── Starting position ──
  STOWED(
      "Stowed",
      new Translation2d(
          ClimbConstants.START_POSITION_X_METERS, ClimbConstants.START_POSITION_Y_METERS),
      false),

  // ── L1 sequence ──

  /** Extend arm up to L1 bar - curved path */
  EXTEND_L1(
      "Extending to L1",
      List.of(
          new Translation2d(
              ClimbConstants.START_POSITION_X_METERS, ClimbConstants.START_POSITION_Y_METERS),
          new Translation2d(0.6, 0.63)),
      false),

  /** Retract cables to pull robot up on L1 bar (teleop) */
  RETRACT_L1(
      "Retracting on L1",
      List.of(
          new Translation2d(0.6, 0.63),
          new Translation2d(0.63, 0.60),
          new Translation2d(0.65, 0.46),
          new Translation2d(0.56, 0.42),
          new Translation2d(0.491, 0.146)),
      true),

  /** Release angle servos after L1 retract (servo-only, no path) */
  RELEASE_ANGLE_L1("Release Angle L1", new Translation2d(0.491, 0.146), false),

  /** Release hardstop servos after angle released (servo-only, no path) */
  RELEASE_HARDSTOP_L1("Release Hardstop L1", new Translation2d(0.491, 0.146), false),

  // ── L2 sequence ──

  /** Extend arm from L1 up to L2 bar - curved path */
  EXTEND_L2(
      "Extending to L2",
      List.of(
          new Translation2d(0.38, 0.50),
          new Translation2d(0.40, 0.54),
          new Translation2d(0.42, 0.58)),
      false),

  /** Retract cables to pull robot up on L2 bar */
  RETRACT_L2(
      "Retracting on L2",
      List.of(new Translation2d(0.42, 0.58), new Translation2d(0.39, 0.52)),
      true),

  /** Stow servos (hardstop then angle) after L2 retract (servo-only, no path) */
  STOW_SERVOS_L2("Stow Servos L2", new Translation2d(0.39, 0.52), false),

  /** Release angle servos after L2 stow (servo-only, no path) */
  RELEASE_ANGLE_L2("Release Angle L2", new Translation2d(0.39, 0.52), false),

  /** Release hardstop servos after L2 angle released (servo-only, no path) */
  RELEASE_HARDSTOP_L2("Release Hardstop L2", new Translation2d(0.39, 0.52), false),

  // ── L3 sequence ──

  /** Extend arm from L2 up to L3 bar - curved path */
  EXTEND_L3(
      "Extending to L3",
      List.of(
          new Translation2d(0.39, 0.52),
          new Translation2d(0.41, 0.56),
          new Translation2d(0.43, 0.60)),
      false),

  /** Retract cables to pull robot up on L3 bar (final climb position) */
  RETRACT_L3(
      "Retracting on L3",
      List.of(new Translation2d(0.43, 0.60), new Translation2d(0.40, 0.54)),
      true),

  // ── Auto-only L1 sequence (NOT part of teleop state cycle) ──

  /** Extend arm up to L1 bar for auto climb only */
  EXTEND_L1_AUTO(
      "Auto Extending to L1",
      List.of(
          new Translation2d(
              ClimbConstants.START_POSITION_X_METERS, ClimbConstants.START_POSITION_Y_METERS),
          new Translation2d(0.6, 0.63)),
      false),

  /** Retract cables on L1 bar for auto climb only */
  RETRACT_L1_AUTO(
      "Auto Retracting on L1",
      List.of(
          new Translation2d(0.6, 0.63),
          new Translation2d(0.63, 0.60),
          new Translation2d(0.65, 0.46),
          new Translation2d(0.56, 0.42)),
      true),

  // ── Special states ──
  MANUAL("Manual", new Translation2d(0.0, 0.0), false),
  EMERGENCY_STOP("E-Stop", new Translation2d(0.0, 0.0), false);

  // State data
  private final String name;
  private final Translation2d targetPosition;
  private final List<Translation2d> prePlannedWaypoints;
  private final boolean isPulling;

  /** Constructor for states WITH a path — target is derived from the last waypoint. */
  ClimbState(String name, List<Translation2d> waypoints, boolean isPulling) {
    this(name, waypoints.get(waypoints.size() - 1), waypoints, isPulling);
  }

  /** Constructor for states WITHOUT a path (STOWED, MANUAL, EMERGENCY_STOP). */
  ClimbState(String name, Translation2d target, boolean isPulling) {
    this(name, target, null, isPulling);
  }

  /** Private canonical constructor. */
  private ClimbState(
      String name, Translation2d target, List<Translation2d> waypoints, boolean isPulling) {
    this.name = name;
    this.targetPosition = target;
    this.prePlannedWaypoints = waypoints;
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
        return RELEASE_ANGLE_L1;
      case RELEASE_ANGLE_L1:
        return RELEASE_HARDSTOP_L1;
      case RELEASE_HARDSTOP_L1:
        return EXTEND_L2;
      case EXTEND_L2:
        return RETRACT_L2;
      case RETRACT_L2:
        return STOW_SERVOS_L2;
      case STOW_SERVOS_L2:
        return RELEASE_ANGLE_L2;
      case RELEASE_ANGLE_L2:
        return RELEASE_HARDSTOP_L2;
      case RELEASE_HARDSTOP_L2:
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
      case RELEASE_ANGLE_L1:
        return RETRACT_L1;
      case RELEASE_HARDSTOP_L1:
        return RELEASE_ANGLE_L1;
      case EXTEND_L2:
        return RELEASE_HARDSTOP_L1;
      case RETRACT_L2:
        return EXTEND_L2;
      case STOW_SERVOS_L2:
        return RETRACT_L2;
      case RELEASE_ANGLE_L2:
        return STOW_SERVOS_L2;
      case RELEASE_HARDSTOP_L2:
        return RELEASE_ANGLE_L2;
      case EXTEND_L3:
        return RELEASE_HARDSTOP_L2;
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
