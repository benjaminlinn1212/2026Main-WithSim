// Copyright (c) 2026 FRC Team 10922 (Amped)
// Dashboard-driven autonomous system — Behavioral tuning constants

package frc.robot.auto.dashboard;

/**
 * Tuning constants for the dashboard auto builder: SWD gating, sim fallbacks, time budgeting.
 * Separated from FieldConstants (geometry) and AutoConstants (PID/constraints). Values marked "TUNE
 * THIS" should be adjusted on the real robot.
 */
public final class AutoTuning {

  private AutoTuning() {}

  // ===== Shoot-While-Driving Gating =====
  // Feeding is gated only by hood readiness (atSetpoint). Once the hood reaches its setpoint,
  // feeding latches on and stays on until the robot leaves the aiming zone.

  // ===== Neutral Zone Repeat Visit Depth =====

  /**
   * TUNE THIS: Extra depth (meters) per repeat neutral zone visit along the intake heading. Visit 1
   * = nominal, visit 2 = +1x deeper, visit 3 = +2x deeper, etc.
   */
  public static final double NEUTRAL_ZONE_DEEPER_PER_VISIT = 1.5;

  // ===== Shooter Current-Based Shot Completion Detection =====
  // Monitor conveyor current: sustained low = all FUEL fired. Falls back to timeout if disabled.

  /** Master toggle for current-based shot completion detection. False = fixed time delay. */
  public static final boolean USE_CURRENT_DETECTION = false;

  /** TUNE THIS: Conveyor "no FUEL" current threshold (amps). Below this = spinning freely. */
  public static final double CONVEYOR_NO_FUEL_CURRENT_THRESHOLD_AMPS = 4.0;

  /** TUNE THIS: Duration (sec) conveyor must stay below threshold to confirm all FUEL fired. */
  public static final double SHOOTER_DONE_TIME_SECONDS = 0.5;

  /** Max wait (sec) for shooter completion. Safety fallback if current detection gets stuck. */
  public static final double SHOOTER_DETECT_TIMEOUT_SECONDS = 1.5;

  // ===== Simulation Fallbacks =====
  // Sim IO doesn't model FUEL load — shooter completion uses a fixed time delay.

  /** In simulation, use a fixed time delay for shooter completion. */
  public static final double SIM_SHOOTER_DONE_SECONDS = 1.5;

  // ===== Time-Check Budgeting =====
  // These values are used by the runtime time-check methods in AutoCommandBuilder to decide
  // whether there's enough time to continue cycling or to abort to climb.

  /**
   * TUNE THIS: Estimated time (sec) for stop-and-shoot (pathfind + stop + aim + fire). Typical
   * 1.0-1.5s real robot. Use worst-case for safe budgeting.
   */
  public static final double STOP_AND_SHOOT_DURATION = 2.0;

  /** TUNE THIS: Estimated time (sec) at intake location (decel + dwell + resume). */
  public static final double INTAKE_DWELL_ESTIMATE = 1.0;

  /** Estimated time (sec) for SWD pass through scoring zone (~4m at ~2m/s). */
  public static final double SWD_SCORE_DURATION = 2.0;

  // ===== Runtime Drive Time Correction =====

  /**
   * TUNE THIS: Multiplier on straight-line drive time estimates (AD* paths are longer due to
   * obstacles, accel/decel, curves). Start at 1.25, adjust per DashboardAuto/ULScoreBreakdown logs.
   */
  public static final double RUNTIME_DRIVE_TIME_MULTIPLIER = 1.1;

  // ===== Stop-and-Shoot Velocity Guard =====

  /** TUNE THIS: Max speed (m/s) to be considered "stopped" for stop-and-shoot feeding. */
  public static final double STOP_AND_SHOOT_MAX_SPEED_MPS = 0.1;

  // ===== Zone-Aware Aiming Threshold =====

  /**
   * X threshold (blue-origin meters) for aiming zone boundary. Below = aim turret, above = stow.
   * Derived from NEUTRAL_ZONE.minX.
   */
  public static final double AIMING_ZONE_MAX_X = FieldConstants.Zone.NEUTRAL_ZONE.minX; // 5.50m
}
