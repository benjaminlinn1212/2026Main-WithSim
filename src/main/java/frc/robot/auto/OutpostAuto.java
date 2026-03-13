// Copyright (c) 2026 FRC Team 10922 (Amped)
// Hardcoded outpost autonomous — see class Javadoc for the full sequence.

package frc.robot.auto;

import static frc.robot.auto.dashboard.AutoTuning.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.dashboard.FieldConstants;
import frc.robot.auto.dashboard.FieldConstants.StartPose;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * Hardcoded outpost auto using pre-drawn PathPlanner paths for trench traversal. Sequence: seed
 * pose -> start-to-hub -> (hub-to-neutral intake, neutral-to-hub shoot) -> SWD to outpost ->
 * dwell/jiggle -> final shoot -> idle.
 */
public class OutpostAuto {

  // ==================== Path Names (must match files in deploy/pathplanner/paths/) ==============

  /** Path: HUB_LOWER → NEUTRAL_ZONE_LOWER (outbound intake leg). */
  private static final String PATH_HUB_TO_NEUTRAL = "Outpost Hub To Neutral 1";

  /** Path: NEUTRAL_ZONE_LOWER → OUTPOST (inbound through trench, final leg). */
  private static final String PATH_NEUTRAL_TO_OUTPOST = "Outpost Neutral To Outpost";

  // ==================== Route Constants ====================

  /** Starting pose for this auto. Also used by RobotContainer.getAutoStartingPose(). */
  public static final StartPose START_POSE = StartPose.LOWER;

  // ==================== Timing Constants ====================

  /** Dwell at the intake location to collect FUEL before leaving (seconds). */
  private static final double INTAKE_DWELL_SECONDS = 0.0;

  /**
   * Feed duration for the outpost stop-and-shoot (seconds). Much longer than a normal cycle because
   * we may accumulate extra FUEL from the SWD leg and the human player continues feeding FUEL
   * through the CHUTE.
   */
  private static final double OUTPOST_SHOOT_DURATION_SECONDS = 10.0;

  /** Dwell at outpost position before activating the jiggle (seconds). */
  private static final double OUTPOST_DWELL_BEFORE_JIGGLE_SECONDS = 1.0;

  // ==================== Dependencies ====================

  private final DriveSwerveDrivetrain drive;
  private final Superstructure superstructure;

  // ==================== Runtime State ====================

  /**
   * FPGA timestamp captured at the very start of the auto command. Used to compute remaining auto
   * time as {@code AUTO_DURATION - (now - autoStartTimestamp)}. Works in sim, practice, and
   * competition (unlike {@code Timer.getMatchTime()} which returns -1 without FMS).
   */
  private double autoStartTimestamp = 0.0;

  // ==================== Constructor ====================

  public OutpostAuto(DriveSwerveDrivetrain drive, Superstructure superstructure) {
    this.drive = drive;
    this.superstructure = superstructure;
  }

  // ==================== Public Entry Points ====================

  /** Deferred command — safe for auto choosers. Builds a fresh command graph each invocation. */
  public Command getCommand() {
    return Commands.defer(this::buildCommand, Set.of(drive));
  }

  /**
   * Build the full auto command graph. Public so RobotContainer can call it directly for a fresh
   * instance each auto run (avoids the "already composed" error).
   */
  public Command buildCommand() {
    return Commands.deadline(
            Commands.sequence(
                buildInit(), buildIntakeThenSWDToOutpost(), buildOutpostSequence(), buildCleanup()),
            // Continuous timer logging — runs every 20ms for the entire auto so AdvantageScope
            // never sees gaps in the time-remaining signal.
            Commands.run(
                () -> Logger.recordOutput("OutpostAuto/AutoTimeRemaining", getAutoTimeRemaining())))
        .withName("OutpostAuto");
  }

  // ==================== Phase Builders ====================

  /** Phase 0: Seed pose, reset superstructure, and capture auto start time. */
  private Command buildInit() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              autoStartTimestamp = Timer.getFPGATimestamp();
              drive.setPose(START_POSE.getPose());
            }),
        Commands.runOnce(superstructure::forceIdleState),
        Commands.print("[OutpostAuto] Starting — Lower start, outpost finish"));
  }

  /**
   * Phase 1: Follow path to the neutral zone to intake FUEL, then follow drawn path directly to the
   * outpost with shoot-while-driving (no return to HUB for a stop-and-shoot). Feeds only while
   * inside the alliance zone.
   */
  private Command buildIntakeThenSWDToOutpost() {
    return Commands.sequence(
            // Intake leg — follow Hub To Neutral 1 path. ONLY_INTAKE the entire way out
            // (no aiming on the outbound leg — shooter should not aim until coming back).
            Commands.print("[OutpostAuto] Intaking at NEUTRAL_ZONE_LOWER"),
            Commands.deadline(
                followPath(PATH_HUB_TO_NEUTRAL),
                Commands.run(
                    () -> superstructure.forceWantedState(SuperstructureState.ONLY_INTAKE),
                    superstructure)),
            // Enable outpost pivot pose for the return leg (no aiming yet — stays ONLY_INTAKE
            // until the robot reaches the alliance zone).
            Commands.runOnce(
                () -> {
                  superstructure.setIntakeOutpostMode(true);
                }),
            Commands.waitSeconds(INTAKE_DWELL_SECONDS),
            // SWD leg — drive to outpost, but only start aiming + feeding once inside the
            // alliance zone. Stay in ONLY_INTAKE while still in the neutral/aiming zone so
            // the shooter doesn't spin up prematurely on the return from neutral intake.
            Commands.print("[OutpostAuto] Cycle 2 — SWD to outpost"),
            Commands.deadline(
                followPath(PATH_NEUTRAL_TO_OUTPOST),
                Commands.run(
                    () -> {
                      boolean inAllianceZone = isInAllianceZone();
                      if (inAllianceZone) {
                        superstructure.forceWantedState(SuperstructureState.AIMING_WHILE_INTAKING);
                        superstructure.setFeedingRequested(true);
                      } else {
                        superstructure.forceWantedState(SuperstructureState.ONLY_INTAKE);
                        superstructure.setFeedingRequested(false);
                      }
                      Logger.recordOutput("OutpostAuto/SWD/Active", true);
                      Logger.recordOutput("OutpostAuto/SWD/Feeding", inAllianceZone);
                    },
                    superstructure)),
            Commands.runOnce(
                () -> {
                  superstructure.setFeedingRequested(false);
                  Logger.recordOutput("OutpostAuto/SWD/Active", false);
                }))
        .withName("IntakeThenSWD_ToOutpost");
  }

  /**
   * Phase 5: At the outpost — enable outpost intake mode, dwell briefly so FUEL falls from the
   * CHUTE, then activate the intake pivot jiggle to dislodge stuck pieces while stop-and-shooting
   * with the extended outpost duration.
   */
  private Command buildOutpostSequence() {
    return Commands.sequence(
            Commands.print("[OutpostAuto] At outpost — enabling outpost intake mode"),
            Commands.runOnce(() -> superstructure.setIntakeOutpostMode(true)),
            Commands.waitSeconds(OUTPOST_DWELL_BEFORE_JIGGLE_SECONDS),
            Commands.print("[OutpostAuto] Activating outpost jiggle + stop-and-shoot"),
            Commands.runOnce(() -> superstructure.setIntakeOutpostJiggleMode(true)),
            buildStopAndShootSequence(OUTPOST_SHOOT_DURATION_SECONDS),
            Commands.runOnce(
                () -> {
                  superstructure.setIntakeOutpostJiggleMode(false);
                  superstructure.setIntakeOutpostMode(false);
                }))
        .withName("OutpostShoot");
  }

  /** Phase 6: Return to idle. */
  private Command buildCleanup() {
    return Commands.sequence(superstructure.idle(), Commands.print("[OutpostAuto] Complete!"));
  }

  // ==================== Reusable Command Helpers ====================

  /**
   * Stop-and-shoot: aim at the HUB, feed for a fixed duration, then return to zone-aware default
   * state. Uses a simple timed feed rather than current-based detection for reliability in a
   * hardcoded auto.
   *
   * @param durationSeconds How long to feed FUEL into the shooter
   */
  private Command buildStopAndShootSequence(double durationSeconds) {
    return Commands.sequence(
        superstructure.aimingWhileIntaking(),
        Commands.runOnce(() -> superstructure.setFeedingRequested(true)),
        Commands.waitSeconds(durationSeconds),
        Commands.runOnce(() -> superstructure.setFeedingRequested(false)),
        zoneAwareDefaultState());
  }

  /** Instant zone-aware state set (used after stop-and-shoot to pick the correct idle state). */
  private Command zoneAwareDefaultState() {
    return Commands.runOnce(
            () ->
                superstructure.forceWantedState(
                    isOutsideAimingZone()
                        ? SuperstructureState.ONLY_INTAKE
                        : SuperstructureState.AIMING_WHILE_INTAKING))
        .withName("ZoneAwareDefault");
  }

  // ==================== Field Zone Helpers ====================

  /** Convert the current robot pose to blue-alliance X coordinate. */
  private double getBlueX() {
    double rawX = drive.getPose().getX();
    return FieldConstants.isRedAlliance() ? FieldConstants.FIELD_LENGTH - rawX : rawX;
  }

  /** True when the robot is past the aiming zone boundary (in the neutral zone or beyond). */
  private boolean isOutsideAimingZone() {
    return getBlueX() > AIMING_ZONE_MAX_X;
  }

  /** True when the robot is inside the alliance zone (safe to feed during SWD). */
  private boolean isInAllianceZone() {
    return getBlueX() <= FieldConstants.Zone.ALLIANCE_ZONE.maxX;
  }

  // ==================== Time Management Helpers ====================

  /**
   * Get seconds remaining in the auto period. Uses our own FPGA-based countdown so it works in sim,
   * practice, and competition (unlike {@code Timer.getMatchTime()} which returns -1 without FMS).
   *
   * <p>If the auto start timestamp hasn't been captured yet, returns {@code AUTO_DURATION} as a
   * safe fallback.
   */
  private double getAutoTimeRemaining() {
    if (autoStartTimestamp <= 0.0) {
      return FieldConstants.AUTO_DURATION;
    }
    double elapsed = Timer.getFPGATimestamp() - autoStartTimestamp;
    return Math.max(0.0, FieldConstants.AUTO_DURATION - elapsed);
  }

  // ==================== Path Following Helpers ====================

  /**
   * Load a pre-drawn PathPlanner path by name and return a deferred follow command. PathPlanner
   * handles alliance flipping automatically based on the {@code shouldFlipPath} supplier configured
   * in {@code AutoBuilder.configure()}.
   *
   * @param pathName The path file name (without extension) in deploy/pathplanner/paths/
   * @return A deferred command that follows the path
   */
  private Command followPath(String pathName) {
    return Commands.defer(
        () -> {
          try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.followPath(path);
          } catch (Exception e) {
            System.err.println("[OutpostAuto] Failed to load path: " + pathName);
            e.printStackTrace();
            return Commands.print("[OutpostAuto] ERROR: Path not found: " + pathName);
          }
        },
        Set.of(drive));
  }
}
