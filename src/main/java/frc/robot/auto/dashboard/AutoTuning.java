// Copyright (c) 2026 FRC Team 10922 (Amped)
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

  // ===== Intake Current-Based FUEL Presence Detection =====
  // New logic: monitor the LOWER intake roller current. Because there's motor accel time,
  // the rollers always draw some current when spinning. If the lower roller current drops
  // below the threshold for a sustained period, it means no FUEL is in the intake path.
  // Conversely, while FUEL is present the lower roller is loaded and stays above threshold.

  /**
   * TUNE THIS: Stator current threshold (amps) for the lower intake roller below which we consider
   * "no FUEL present". When the rollers are spinning with FUEL, the lower roller current stays
   * above this. When the rollers are spinning without FUEL (free-spinning), current drops below.
   * The detection triggers after the current stays below this for {@link
   * #INTAKE_NO_FUEL_TIME_SECONDS}.
   */
  public static final double INTAKE_NO_FUEL_CURRENT_THRESHOLD_AMPS = 9.0;

  /**
   * TUNE THIS: How long (seconds) the lower intake roller current must stay below {@link
   * #INTAKE_NO_FUEL_CURRENT_THRESHOLD_AMPS} before we declare "no FUEL in intake". Because the
   * motor has accel time, we need a sustained low-current window to avoid false triggers during
   * transients. 0.5s accounts for motor spin-up and brief current dips between fuel contacts.
   */
  public static final double INTAKE_NO_FUEL_TIME_SECONDS = 0.5;

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

  // ===== Neutral Zone Repeat Visit Depth =====

  /**
   * TUNE THIS: How much deeper (meters) to drive past the nominal neutral zone intake waypoint on
   * each subsequent visit. On the first visit (visitNumber=1), the robot drives to the nominal
   * pose. On the second visit (visitNumber=2), it drives {@code 1 * NEUTRAL_ZONE_DEEPER_PER_VISIT}
   * meters deeper. On the third visit, {@code 2 * NEUTRAL_ZONE_DEEPER_PER_VISIT}, etc.
   *
   * <p>The "deeper" direction is along the intake heading (the direction the robot faces when
   * intaking). For NEUTRAL_ZONE_UPPER (heading=90°), deeper means further positive-Y. For
   * NEUTRAL_ZONE_LOWER (heading=-90°), deeper means further negative-Y. This pushes the robot
   * further into the FUEL scatter zone where un-collected FUEL remains after previous passes.
   */
  public static final double NEUTRAL_ZONE_DEEPER_PER_VISIT = 1.5;

  // ===== Shooter Current-Based Shot Completion Detection =====
  // New logic: monitor the CONVEYOR motor current instead of the shooter motor. Because there's
  // motor accel time, the conveyor always draws current when feeding. If conveyor current drops
  // below the threshold for a sustained period, it means no FUEL is being pushed through —
  // all FUEL has been fired.

  /**
   * TUNE THIS: Stator current threshold (amps) for the conveyor motor below which we consider "no
   * FUEL being fed". When FUEL is being pushed into the shooter, the conveyor is loaded and current
   * stays above this. When all FUEL has exited and the conveyor is spinning freely, current drops
   * below. The detection triggers after the current stays below this for {@link
   * #SHOOTER_DONE_TIME_SECONDS}.
   */
  public static final double CONVEYOR_NO_FUEL_CURRENT_THRESHOLD_AMPS = 4.0;

  /**
   * TUNE THIS: How long (seconds) the conveyor current must stay below {@link
   * #CONVEYOR_NO_FUEL_CURRENT_THRESHOLD_AMPS} before we declare "all FUEL has been fired". Because
   * the motor has accel time, we need a sustained low-current window to confirm there are no more
   * FUEL queued. 0.5s accounts for gaps between consecutive FUEL and motor transients.
   */
  public static final double SHOOTER_DONE_TIME_SECONDS = 0.5;

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
   * When true, the time estimation for intake dwell and shooting includes the current-detection
   * timing constants ({@link #INTAKE_NO_FUEL_TIME_SECONDS}, {@link #SHOOTER_DONE_TIME_SECONDS},
   * {@link #SHOOTER_DETECT_TIMEOUT_SECONDS}). This gives a more conservative (realistic) estimate
   * that accounts for the time the robot waits for current-based FUEL presence/absence detection.
   *
   * <p>When false, the fixed estimates ({@link #STOP_AND_SHOOT_DURATION}, {@link
   * #INTAKE_DWELL_ESTIMATE}) are used as-is — simpler but may underestimate actual cycle time.
   *
   * <p>TUNE THIS: Enable once current detection is tuned and you want tighter time budgets.
   */
  public static final boolean USE_CURRENT_DETECTION_TIMING = false;

  /**
   * TUNE THIS: Estimated time (seconds) for aim convergence + shooter spin-up before the first FUEL
   * exits. Used only when {@link #USE_CURRENT_DETECTION_TIMING} is true.
   */
  public static final double AIM_CONVERGENCE_SECONDS = 0.3;

  /**
   * TUNE THIS: Estimated time (seconds) for the robot to decelerate to a stop at an intake
   * location. Used only when {@link #USE_CURRENT_DETECTION_TIMING} is true.
   */
  public static final double INTAKE_DECEL_SECONDS = 0.3;

  /**
   * Estimated time (seconds) for a stop-and-shoot action: pathfind to scoring waypoint, come to a
   * stop, aim, fire all FUEL, and resume. Includes aim convergence + shooter spin-up + firing all
   * loaded FUEL. On a real robot this is typically 1.0-1.5s; in sim it uses {@link
   * #SIM_SHOOTER_DONE_SECONDS}. Use the worst-case estimate for safe time budgeting.
   *
   * <p>When {@link #USE_CURRENT_DETECTION_TIMING} is true, uses {@link #AIM_CONVERGENCE_SECONDS} +
   * {@link #SHOOTER_DETECT_TIMEOUT_SECONDS} (worst-case current detection wait) instead of the
   * fixed estimate.
   */
  public static final double STOP_AND_SHOOT_DURATION =
      USE_CURRENT_DETECTION_TIMING ? AIM_CONVERGENCE_SECONDS + SHOOTER_DETECT_TIMEOUT_SECONDS : 1.5;

  /**
   * Estimated time (seconds) spent at an intake location picking up FUEL — deceleration, dwell for
   * current detection or sim latch, and any nudge recovery. On a real robot: ~0.5-1.0s. In sim:
   * nearly instant but the pathfinder still decelerates.
   *
   * <p>When {@link #USE_CURRENT_DETECTION_TIMING} is true, uses {@link #INTAKE_DECEL_SECONDS} +
   * {@link #INTAKE_NO_FUEL_TIME_SECONDS} + {@link #INTAKE_NUDGE_TIMEOUT_SECONDS} (worst-case: no
   * fuel, nudge, retry) instead of the fixed estimate.
   */
  public static final double INTAKE_DWELL_ESTIMATE =
      USE_CURRENT_DETECTION_TIMING
          ? INTAKE_DECEL_SECONDS + INTAKE_NO_FUEL_TIME_SECONDS + INTAKE_NUDGE_TIMEOUT_SECONDS
          : 1.0;

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
