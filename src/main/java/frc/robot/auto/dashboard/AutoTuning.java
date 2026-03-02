// Copyright (c) 2026 FRC Team 10922 (Amped)
// Dashboard-driven autonomous system — Behavioral tuning constants

package frc.robot.auto.dashboard;

/**
 * Tuning constants for the dashboard auto command builder. These are the "knobs" that control
 * shoot-while-driving gating, sim fallbacks, and time budgeting.
 *
 * <p>Separated from {@link FieldConstants} (field geometry) and {@link
 * frc.robot.Constants.AutoConstants} (path-following PID / drive constraints) so they're easy to
 * find and tweak during practice.
 *
 * <p><b>TUNE THIS FILE</b> — values marked "TUNE THIS" should be adjusted based on real-robot
 * testing.
 */
public final class AutoTuning {

  private AutoTuning() {}

  // ===== Shoot-While-Driving Gating =====
  // Feeding is gated only by hood readiness (atSetpoint). Once the hood reaches its setpoint,
  // feeding latches on and stays on until the robot leaves the aiming zone.

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
  // Sim IO doesn't model FUEL load — shooter completion uses a fixed time delay.

  /** In simulation, use a fixed time delay for shooter completion. */
  public static final double SIM_SHOOTER_DONE_SECONDS = 1.5;

  // ===== Time-Check Budgeting =====
  // These values are used by the runtime time-check methods in AutoCommandBuilder to decide
  // whether there's enough time to continue cycling or to abort to climb.

  /**
   * TUNE THIS: Estimated time (seconds) for a stop-and-shoot action: pathfind to scoring waypoint,
   * come to a stop, aim, fire all FUEL, and resume. Includes aim convergence + shooter spin-up +
   * firing all loaded FUEL. On a real robot this is typically 1.0-1.5s; in sim it uses {@link
   * #SIM_SHOOTER_DONE_SECONDS}. Use the worst-case estimate for safe time budgeting.
   */
  public static final double STOP_AND_SHOOT_DURATION = 2.0;

  /**
   * TUNE THIS: Estimated time (seconds) spent at an intake location picking up FUEL — deceleration,
   * dwell, and resume. On a real robot: ~0.5-1.0s. In sim: nearly instant but the pathfinder still
   * decelerates.
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
   * <p>TUNE THIS: Start at 1.25 and adjust based on {@code DashboardAuto/ULScoreBreakdown} logs. If
   * the robot still runs out of time before climbing, increase. If it aborts to climb too early,
   * decrease. Note: {@link FieldConstants#estimateDriveTime} already inflates by {@code
   * 1/PATH_DISTANCE_DERATING} (≈1.11×), so the combined inflation is {@code multiplier × 1.11}.
   */
  public static final double RUNTIME_DRIVE_TIME_MULTIPLIER = 1.1;

  // ===== Stop-and-Shoot Velocity Guard =====

  /**
   * Maximum linear speed (m/s) at which the robot is considered "stopped" for stop-and-shoot
   * feeding. If the robot's speed exceeds this threshold, feeding pauses (AIMING_WHILE_INTAKING) to
   * avoid wasting FUEL while moving. Feeding resumes once the robot slows below this threshold.
   *
   * <p>TUNE THIS: 0.1 m/s allows for minor drivetrain settling vibration while still catching any
   * real movement (bumped, still decelerating, etc.).
   */
  public static final double STOP_AND_SHOOT_MAX_SPEED_MPS = 0.1;

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
