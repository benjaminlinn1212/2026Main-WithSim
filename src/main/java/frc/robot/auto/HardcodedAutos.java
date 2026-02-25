// Copyright (c) 2026 FRC Team 0 (Amped)
// Hardcoded fallback autonomous routines — one per lane (UPPER, CENTER, LOWER)

package frc.robot.auto;

import static frc.robot.auto.dashboard.AutoTuning.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.dashboard.DashboardAutoManager;
import frc.robot.auto.dashboard.FieldConstants;
import frc.robot.auto.dashboard.FieldConstants.IntakeLocation;
import frc.robot.auto.dashboard.FieldConstants.ScoringWaypoint;
import frc.robot.auto.dashboard.FieldConstants.StartPose;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * Hardcoded fallback autonomous routines for when the dashboard auto system isn't configured or as
 * a reliable alternative. Reads the Start Pose from the dashboard auto settings (same dropdown used
 * by Dashboard Auto) and maps it to a lane: UPPER, CENTER, or LOWER. Each lane follows a simple
 * pattern: score preload → (intake → score) cycle.
 *
 * <p>Easy to configure: adjust the per-lane constants (scoring waypoint, intake location, cycle
 * count) at the top of the class.
 *
 * <p>No separate lane chooser — just set the "Auto Settings/Start Pose" dropdown on the dashboard
 * and select "Hardcoded Auto" in the main auto chooser. The start pose pre-seeding in Robot.java
 * works correctly since both auto modes read from the same setting.
 */
public class HardcodedAutos {

  // ===================================================================
  // CONFIGURATION — Edit these to change what each hardcoded auto does.
  // Each lane defines: scoring waypoint, intake location, and how many
  // intake→score cycles to attempt after the preload.
  // The start pose comes from the dashboard auto settings.
  // ===================================================================

  // --- UPPER LANE ---
  private static final ScoringWaypoint UPPER_SCORE = ScoringWaypoint.HUB_UPPER;
  private static final IntakeLocation UPPER_INTAKE = IntakeLocation.NEUTRAL_ZONE_UPPER;
  private static final int UPPER_CYCLES = 1;

  // --- CENTER LANE ---
  private static final ScoringWaypoint CENTER_SCORE = ScoringWaypoint.HUB_CENTER;
  private static final IntakeLocation CENTER_INTAKE = IntakeLocation.NEUTRAL_ZONE_UPPER;
  private static final int CENTER_CYCLES = 1;

  // --- LOWER LANE ---
  private static final ScoringWaypoint LOWER_SCORE = ScoringWaypoint.HUB_LOWER;
  private static final IntakeLocation LOWER_INTAKE = IntakeLocation.NEUTRAL_ZONE_LOWER;
  private static final int LOWER_CYCLES = 1;

  // ===================================================================

  private final DriveSwerveDrivetrain drive;
  private final Superstructure superstructure;
  private final DashboardAutoManager dashboardAutoManager;

  public HardcodedAutos(
      DriveSwerveDrivetrain drive,
      Superstructure superstructure,
      DashboardAutoManager dashboardAutoManager) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.dashboardAutoManager = dashboardAutoManager;
  }

  /**
   * Get the deferred command that reads the dashboard Start Pose setting at auto init time and
   * returns the corresponding hardcoded auto command for that lane.
   */
  public Command getCommand() {
    return Commands.defer(
        () -> {
          StartPose startPose = dashboardAutoManager.getSettings().getStartPose();
          switch (startPose) {
            case UPPER:
              return buildLaneAuto("Upper", startPose, UPPER_SCORE, UPPER_INTAKE, UPPER_CYCLES);
            case LOWER:
              return buildLaneAuto("Lower", startPose, LOWER_SCORE, LOWER_INTAKE, LOWER_CYCLES);
            case CENTER:
            default:
              return buildLaneAuto("Center", startPose, CENTER_SCORE, CENTER_INTAKE, CENTER_CYCLES);
          }
        },
        Set.of(drive));
  }

  // ===== Auto Builder =====

  /**
   * Build a complete lane auto: score preload → N × (intake → score).
   *
   * <p>The start pose is NOT set here — it was already pre-seeded during disabled by Robot.java
   * using the same dashboard Start Pose setting that selected this lane.
   *
   * <p>Mirrors the patterns from {@link frc.robot.auto.dashboard.AutoCommandBuilder}:
   *
   * <ul>
   *   <li>Zone-aware state management during drives (aimingWhileIntaking in alliance zone,
   *       onlyIntake in neutral zone)
   *   <li>Fixed-time waits for intake dwell and shooting (simple and predictable for a fallback
   *       auto)
   * </ul>
   *
   * @param label Human-readable lane name for logging
   * @param start The start pose (for logging only — pose was pre-seeded during disabled)
   * @param scoreWaypoint Where to shoot FUEL
   * @param intakeLocation Where to collect FUEL
   * @param cycles Number of intake→score cycles after the preload
   */
  private Command buildLaneAuto(
      String label,
      StartPose start,
      ScoringWaypoint scoreWaypoint,
      IntakeLocation intakeLocation,
      int cycles) {

    Command auto =
        Commands.sequence(
            // Reset superstructure state (in case previous auto ended in EMERGENCY)
            Commands.runOnce(() -> superstructure.forceIdleState()),

            // Log start (pose was already pre-seeded during disabled)
            Commands.print(
                "[HardcodedAuto] Running "
                    + label
                    + " lane auto (start pose: "
                    + start.name()
                    + ")"),

            // 1. Score preload — zone-aware drive to scoring waypoint, then stop-and-shoot
            Commands.print("[HardcodedAuto] Scoring preload"),
            Commands.deadline(pathfindTo(scoreWaypoint.toPose()), zoneAwareIntake()),
            buildStopAndShootSequence());

    // 3. Intake→Score cycles
    for (int i = 0; i < cycles; i++) {
      final int cycle = i + 1;
      auto =
          auto.andThen(
              Commands.print("[HardcodedAuto] Cycle " + cycle + "/" + cycles + " — intaking"),
              // Drive to intake with zone-aware state
              Commands.deadline(pathfindTo(intakeLocation.getPose()), zoneAwareIntake()),
              // Dwell at intake location to collect FUEL
              Commands.waitSeconds(INTAKE_DWELL_SECONDS),
              Commands.print("[HardcodedAuto] Cycle " + cycle + "/" + cycles + " — scoring"),
              // Drive to scoring waypoint with zone-aware intake, then stop-and-shoot
              Commands.deadline(pathfindTo(scoreWaypoint.toPose()), zoneAwareIntake()),
              buildStopAndShootSequence());
    }

    // 4. End in idle
    auto = auto.andThen(superstructure.idle(), Commands.print("[HardcodedAuto] Complete!"));

    return auto.withName("HardcodedAuto_" + label);
  }

  // ===== Shared Helpers =====

  /**
   * Stop-and-shoot: fire FUEL at the HUB while stationary.
   *
   * <p>The turret should already be tracking the HUB via zone-aware intake during the drive. We go
   * straight to SHOOTING_WHILE_INTAKING (not AIMING first) to avoid a visible state flicker.
   *
   * <p>Uses fixed wait times for simplicity — this is a fallback auto, not the optimized dashboard
   * auto. The dashboard auto uses current-based detection; hardcoded autos keep it simple.
   */
  private Command buildStopAndShootSequence() {
    return Commands.sequence(
        // Fire immediately — turret was already tracking via zoneAwareIntake during the drive
        superstructure.shootingWhileIntaking(),
        // Wait for FUEL to exit the shooter
        Commands.waitSeconds(SHOOT_DURATION_SECONDS),
        // Return to zone-aware default
        zoneAwareDefaultState());
  }

  /**
   * Return an instant command that sets the superstructure to the correct default state based on
   * the robot's current field zone. In the aiming zone (alliance + HUB), the default is
   * AIMING_WHILE_INTAKING so the turret always tracks the HUB. In the neutral/opponent zone, the
   * default is ONLY_INTAKE (turret stowed).
   *
   * <p>Mirrors {@link frc.robot.auto.dashboard.AutoCommandBuilder#zoneAwareDefaultState()}.
   */
  private Command zoneAwareDefaultState() {
    return Commands.runOnce(
            () -> {
              if (isOutsideAimingZone()) {
                superstructure.forceWantedState(Superstructure.SuperstructureState.ONLY_INTAKE);
              } else {
                superstructure.forceWantedState(
                    Superstructure.SuperstructureState.AIMING_WHILE_INTAKING);
              }
            })
        .withName("ZoneAwareDefault");
  }

  /**
   * Continuous zone-aware intake command. Runs every cycle and sets the superstructure state based
   * on whether the robot is in the aiming zone. Use as a parallel with a deadline (pathfindTo).
   *
   * <p>Simplified version of {@link frc.robot.auto.dashboard.AutoCommandBuilder#zoneAwareIntake()}
   * — no acceleration-gated feeding (hardcoded autos do stop-and-shoot only, never SWD).
   */
  private Command zoneAwareIntake() {
    return Commands.run(
            () -> {
              if (isOutsideAimingZone()) {
                superstructure.forceWantedState(Superstructure.SuperstructureState.ONLY_INTAKE);
              } else {
                superstructure.forceWantedState(
                    Superstructure.SuperstructureState.AIMING_WHILE_INTAKING);
              }
              Logger.recordOutput(
                  "HardcodedAuto/ZoneAware/OutsideAimingZone", isOutsideAimingZone());
            },
            superstructure)
        .withName("ZoneAwareIntake");
  }

  /**
   * Check if the robot is currently outside the aiming zone (neutral zone or beyond). When outside,
   * the turret should be stowed instead of tracking the HUB. Uses the same X-threshold approach as
   * {@link frc.robot.auto.dashboard.AutoCommandBuilder#isOutsideAimingZone()}.
   */
  private boolean isOutsideAimingZone() {
    var translation = drive.getPose().getTranslation();
    // Convert alliance coords to blue-origin for the X check
    double blueX =
        FieldConstants.isRedAlliance()
            ? FieldConstants.FIELD_LENGTH - translation.getX()
            : translation.getX();
    return blueX > AIMING_ZONE_MAX_X;
  }

  /**
   * Deferred pathfind command with trench-aware heading snapping. Mirrors the same logic from
   * {@link frc.robot.auto.dashboard.AutoCommandBuilder}.
   */
  private Command pathfindTo(Pose2d target) {
    return Commands.defer(
        () -> AutoBuilder.pathfindToPose(trenchAwarePose(target), drive.getPathConstraints(), 0.0),
        Set.of(drive));
  }

  /** Snap heading to cardinal if near a trench. */
  private static Pose2d trenchAwarePose(Pose2d pose) {
    if (FieldConstants.isNearTrench(pose.getTranslation())) {
      Rotation2d snapped = FieldConstants.snapToCardinal(pose.getRotation());
      return new Pose2d(pose.getTranslation(), snapped);
    }
    return pose;
  }

  // ===== Hardcoded Auto Tuning Constants =====

  /** How long to dwell at the intake location to collect FUEL (seconds). */
  private static final double INTAKE_DWELL_SECONDS = 0.5;

  /** How long to feed FUEL into the shooter before moving on (seconds). */
  private static final double SHOOT_DURATION_SECONDS = 0.75;
}
