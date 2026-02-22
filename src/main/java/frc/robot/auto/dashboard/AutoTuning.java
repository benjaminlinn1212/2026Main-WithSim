// Copyright (c) 2026 FRC Team 0 (Amped)
// Dashboard-driven autonomous system — Behavioral tuning constants

package frc.robot.auto.dashboard;

/**
 * Tuning constants for the dashboard auto command builder. These are the "knobs" that control
 * shoot-while-driving gating, current-based FUEL detection, nudge recovery, and sim fallbacks.
 *
 * <p>Separated from {@link FieldConstants} (field geometry) and {@link
 * frc.robot.Constants.AutoConstants} (path-following PID / drive constraints) so they're easy to
 * find and tweak during practice.
 *
 * <p><b>TUNE THIS FILE</b> — values marked "TUNE THIS" should be adjusted based on real-robot
 * testing. Start with the defaults and adjust based on log data from {@code
 * DashboardAuto/IntakeDetect/*} and {@code DashboardAuto/ShooterDetect/*}.
 */
public final class AutoTuning {

  private AutoTuning() {}

  // ===== Shoot-While-Driving Gating =====

  /**
   * Maximum linear acceleration (m/s²) at which the robot is allowed to start feeding FUEL to the
   * shooter. Shooting at high acceleration degrades FUEL accuracy, so we wait until the robot has
   * decelerated (or is at cruise) before commanding the conveyor/indexer.
   */
  public static final double MAX_FEED_ACCELERATION = 3.5;

  /**
   * Hysteresis band for the acceleration gate. Once feeding starts (accel ≤ MAX_FEED_ACCELERATION),
   * it continues until accel exceeds MAX_FEED_ACCELERATION + this value. Prevents stutter when the
   * acceleration oscillates near the threshold.
   */
  public static final double FEED_ACCEL_HYSTERESIS = 1.0;

  // ===== Intake Current-Based FUEL Pickup Detection =====

  /**
   * TUNE THIS: Stator current threshold (amps) for the upper intake roller that indicates FUEL has
   * contacted the rollers. When the roller current exceeds this value at any point during the drive
   * to the intake pose, the pickup latch is set. Typical free-spinning current is ~2-5A; with FUEL
   * contact expect ~15-30A. Start high and lower until it reliably triggers.
   */
  public static final double INTAKE_CURRENT_THRESHOLD_AMPS = 15.0;

  /**
   * How far (meters) to nudge the robot toward the field center Y-axis when no FUEL pickup was
   * detected during the drive. The robot drives further into the FUEL scatter zone to find nearby
   * FUEL.
   */
  public static final double INTAKE_NUDGE_DISTANCE_METERS = 1.0;

  /**
   * Maximum time (seconds) to wait for a FUEL pickup after the nudge drive completes. If still no
   * current spike, give up and continue the auto sequence without FUEL.
   */
  public static final double INTAKE_NUDGE_TIMEOUT_SECONDS = 0.5;

  // ===== Shooter Current-Based Shot Completion Detection =====

  /**
   * TUNE THIS: Stator current threshold (amps) for the shooter motor that indicates FUEL is
   * currently being launched. The shooter current rises when FUEL contacts the flywheel, then drops
   * when the FUEL exits. We detect "all balls done" when current stays BELOW this for {@link
   * #SHOOTER_NO_BALL_TIME_LIMIT} seconds.
   */
  public static final double SHOOTER_CURRENT_THRESHOLD_AMPS = 20.0;

  /**
   * TUNE THIS: How long (seconds) the shooter current must stay below {@link
   * #SHOOTER_CURRENT_THRESHOLD_AMPS} before we declare "all FUEL has been fired". A single ball
   * takes ~0.05-0.1s to pass through; set this to ~0.15-0.25s to confirm no more are queued. Too
   * short → false positive (pause between balls). Too long → wasted time.
   */
  public static final double SHOOTER_NO_BALL_TIME_LIMIT = 0.20;

  /**
   * Maximum time (seconds) to wait for the shooter to finish firing all FUEL. Safety fallback in
   * case the current detection logic gets stuck (e.g., current never drops, sensor noise). After
   * this timeout, the robot moves on regardless.
   */
  public static final double SHOOTER_DETECT_TIMEOUT_SECONDS = 1.5;

  // ===== Simulation Fallbacks =====
  // Sim IO doesn't model FUEL load, so current-based detection is bypassed.
  // Intake detection: latch is set true immediately upon arrival (no delay).
  // Shooter detection: uses a fixed time delay instead.

  /** In simulation, use a fixed time delay instead of current detection for shooter completion. */
  public static final double SIM_SHOOTER_DONE_SECONDS = 1.5;

  // ===== Time-Check Budgeting =====
  // These values are used by the runtime time-check methods in AutoCommandBuilder to decide
  // whether there's enough time to continue cycling or to abort to climb.

  /**
   * Estimated time (seconds) for a stop-and-shoot action: pathfind to scoring waypoint, come to a
   * stop, aim, fire all FUEL, and resume. Includes aim convergence + shooter spin-up + firing all
   * loaded FUEL. On a real robot this is typically 1.0-1.5s; in sim it uses {@link
   * #SIM_SHOOTER_DONE_SECONDS}. Use the worst-case estimate for safe time budgeting.
   */
  public static final double STOP_AND_SHOOT_DURATION = 1.5;

  /**
   * Estimated time (seconds) spent at an intake location picking up FUEL — deceleration, dwell for
   * current detection or sim latch, and any nudge recovery. On a real robot: ~0.5-1.0s. In sim:
   * nearly instant but the pathfinder still decelerates.
   */
  public static final double INTAKE_DWELL_ESTIMATE = 1.0;

  /**
   * Estimated time (seconds) for a shoot-while-driving (SWD) pass through the HUB zone. The robot
   * drives ~4m through the scoring zone at ~2 m/s while the turret tracks and fires. Used by D/O
   * time checks and the planner for SWD cycle budgeting.
   */
  public static final double SWD_SCORE_DURATION = 2.0;

  // ===== Runtime Drive Time Correction =====

  /**
   * Multiplier applied to straight-line drive time estimates in the builder's runtime time checks.
   * PathPlanner AD* paths are significantly longer than straight-line because of:
   *
   * <ul>
   *   <li>Obstacle avoidance (field elements, BARRIERS, HUB)
   *   <li>Acceleration/deceleration phases at start and end
   *   <li>Curved paths through the navmesh
   * </ul>
   *
   * <p>The planner uses raw straight-line estimates (generous planning). The builder uses this
   * multiplier so runtime time checks reflect actual PathPlanner drive times.
   *
   * <p>TUNE THIS: Start at 1.5 and adjust based on {@code DashboardAuto/ULScoreBreakdown} logs. If
   * the robot still runs out of time before climbing, increase. If it aborts to climb too early,
   * decrease.
   */
  public static final double RUNTIME_DRIVE_TIME_MULTIPLIER = 1.5;

  // ===== Zone-Aware Aiming Threshold =====

  /**
   * X-coordinate threshold (blue-origin meters) separating the aiming zone from the no-aim zone.
   * Everything with blue-origin X ≤ this value is inside the aiming zone (ALLIANCE_ZONE +
   * HUB_ZONE). Above this — neutral zone, opponent side — the robot stows the turret and uses
   * ONLY_INTAKE.
   *
   * <p>Derived from {@link FieldConstants.Zone#NEUTRAL_ZONE} minX. The turret starts aiming as soon
   * as the robot leaves the neutral zone (enters the HUB zone), giving extra time for the
   * turret/shooter to settle before reaching optimal scoring range.
   *
   * <p>Note: the hood is still stowed independently by the Superstructure's trench detection — even
   * though the turret can aim in the HUB zone, the hood stays stowed until clear of the trench.
   */
  public static final double AIMING_ZONE_MAX_X = FieldConstants.Zone.NEUTRAL_ZONE.minX; // 5.50m
}
