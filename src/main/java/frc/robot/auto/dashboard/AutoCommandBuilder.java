// Copyright (c) 2026 FRC Team 0 (Amped)
// Dashboard-driven autonomous system — Command builder

package frc.robot.auto.dashboard;

import static frc.robot.auto.dashboard.AutoTuning.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.auto.dashboard.FieldConstants.ClimbLevel;
import frc.robot.auto.dashboard.FieldConstants.ScoringWaypoint;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climb.ClimbState;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import frc.robot.util.ChezySequenceCommandGroup;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * Converts a list of {@link AutoAction}s from the {@link AutoPlanner} into a single runnable WPILib
 * {@link Command}.
 *
 * <p><b>Runtime cycle budgeting:</b> Instead of blindly executing every planned cycle, this builder
 * checks remaining auto time (via an FPGA-based countdown timer) at runtime before starting each
 * intake→score cycle. If the remaining time is insufficient for the cycle plus any climb reserve,
 * it skips remaining cycles and proceeds directly to climb.
 *
 * <p><b>Shoot-while-driving (254-style):</b> When enabled, the robot does NOT stop at a scoring
 * waypoint. Instead, it drives directly to its next destination (typically the next intake) at full
 * speed. The turret/hood/shooter track the HUB in real-time via {@code ShooterSetpoint} (which uses
 * the robot's live pose), and the conveyor/indexer feed FUEL while the robot is still moving. This
 * saves ~1-2s per cycle by eliminating the scoring waypoint detour entirely.
 *
 * <p>Each action type maps to a composition of:
 *
 * <ul>
 *   <li>PathPlanner pathfinding commands (for navigation)
 *   <li>Superstructure commands (for intaking, scoring, climbing)
 *   <li>Drive commands (for pose resets)
 * </ul>
 *
 * <p>Inspired by 254's approach: the command builder is stateless and produces a fresh command tree
 * each time. It uses {@link ChezySequenceCommandGroup} for faster execution.
 */
public class AutoCommandBuilder {

  // All tuning constants (SWD gating, current detection, sim fallbacks) live in AutoTuning.java.

  private final DriveSwerveDrivetrain drive;
  private final Superstructure superstructure;
  private final ClimbSubsystem climb;

  /**
   * Hysteresis latch for the zone-aware acceleration gate. When true, the robot is currently
   * feeding (SHOOTING_WHILE_INTAKING). It stays true until acceleration exceeds
   * MAX_FEED_ACCELERATION + FEED_ACCEL_HYSTERESIS. When false, it stays false until acceleration
   * drops below MAX_FEED_ACCELERATION. Prevents state stutter when accel oscillates near threshold.
   */
  private boolean zoneAwareFeeding = false;

  /**
   * Runtime flag set when a cycle's SWD score is merged with the post-cycle DriveTo+Climb at
   * runtime. When true, the post-cycle loop skips the DriveTo+Climb pair since it was already
   * handled by the merged command. Reset at the start of each {@link #buildAutoCommand} call.
   */
  private boolean climbMergedAtRuntime = false;

  /**
   * FPGA timestamp captured at the very start of the auto command. Used to compute remaining auto
   * time as {@code AUTO_DURATION - (now - autoStartTimestamp)}. This works everywhere — sim,
   * practice, and competition — unlike {@link Timer#getMatchTime()} which returns -1 without FMS.
   */
  private double autoStartTimestamp = 0.0;

  public AutoCommandBuilder(
      DriveSwerveDrivetrain drive, Superstructure superstructure, ClimbSubsystem climb) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.climb = climb;
  }

  /**
   * Build a single Command that executes the entire auto plan with <b>runtime time checks</b>.
   *
   * <p>The planner generates a priority-ordered list of actions, but this builder doesn't blindly
   * execute all of them. Instead, it splits actions into phases:
   *
   * <ol>
   *   <li><b>Pre-cycle</b>: SetStartPose, ScorePreload — always executed.
   *   <li><b>Cycles</b>: IntakeAt + ScoreAt pairs with <b>zone-based</b> scoring and <b>at-intake
   *       runtime time checks</b>:
   *       <ul>
   *         <li><b>U/L (neutral zone) intakes:</b> After arriving at the intake, check time. If
   *             enough → stop-and-shoot at scoring waypoint, continue. If not → SWD directly to
   *             tower with climb arms extending mid-transit.
   *         <li><b>D/O (alliance zone) intakes:</b> Always SWD — the robot naturally passes through
   *             the HUB zone on the way to the next destination, dumping FUEL mid-transit. Before
   *             starting the <i>next</i> cycle, check time. If not enough for the next cycle → the
   *             current SWD merges with climb (drive to tower while shooting).
   *       </ul>
   *   <li><b>Post-cycle</b>: DriveTo (TOWER) + Climb — executed when all cycles complete or
   *       time-triggered climb already ran.
   * </ol>
   *
   * @param actions Ordered list of actions from the planner
   * @param settings The auto settings (for climb config and time margins)
   * @return A command that executes the full sequence with runtime time management
   */
  public Command buildAutoCommand(List<AutoAction> actions, AutoSettings settings) {
    // Reset mutable state from any previous auto run
    zoneAwareFeeding = false;
    climbMergedAtRuntime = false;
    autoStartTimestamp = 0.0;

    if (actions.isEmpty()) {
      return Commands.print("[DashboardAuto] Empty action list — doing nothing.")
          .withName("DashboardAuto_Empty");
    }

    // Split actions into phases
    List<AutoAction> preCycleActions = new ArrayList<>();
    List<AutoAction> cycleActions = new ArrayList<>(); // IntakeAt + ScoreAt pairs
    List<AutoAction> postCycleActions = new ArrayList<>(); // DriveTo + Climb

    boolean inCycles = false;
    boolean pastCycles = false;

    for (AutoAction action : actions) {
      if (pastCycles) {
        postCycleActions.add(action);
      } else if (action.getType() == AutoAction.Type.INTAKE_AT
          || action.getType() == AutoAction.Type.SCORE_AT) {
        inCycles = true;
        cycleActions.add(action);
      } else if (inCycles) {
        // First non-cycle action after cycles started = post-cycle phase
        pastCycles = true;
        postCycleActions.add(action);
      } else {
        preCycleActions.add(action);
      }
    }

    ChezySequenceCommandGroup sequence = new ChezySequenceCommandGroup();

    // Capture start time for our own countdown timer (works in sim + practice + comp)
    sequence.addCommands(Commands.runOnce(() -> autoStartTimestamp = Timer.getFPGATimestamp()));

    // Start intaking immediately — and aim if in alliance zone
    sequence.addCommands(Commands.runOnce(() -> superstructure.forceIdleState()));
    sequence.addCommands(zoneAwareDefaultState());
    sequence.addCommands(
        Commands.print("[DashboardAuto] Starting auto with " + actions.size() + " actions"));

    // ===== Phase 1: Pre-cycle (always run) =====
    for (AutoAction action : preCycleActions) {
      sequence.addCommands(logStep(action));
      Command cmd = buildActionCommand(action);
      if (cmd != null) sequence.addCommands(cmd);
    }

    // ===== Phase 2: Cycles with zone-based scoring + runtime time checks =====
    //
    // Time checks happen AT THE INTAKE POSE (after arriving), not before starting the cycle.
    // This gives the most accurate time estimate since we know exactly where the robot is.
    //
    // U/L cycles: IntakeAt → [time check] → either stop-and-shoot OR SWD-to-climb
    // D/O cycles: IntakeAt → always SWD → [time check before NEXT cycle]
    // NOTE: U/L intakes whose NEXT target is alliance-zone become SWD (not stop-and-shoot),
    // because the robot drives back through the HUB zone and can shoot mid-transit.

    final AutoSettings settingsRef = settings;

    // CRITICAL: Derive climb target from SETTINGS, not just the planner's action list.
    // The planner may fail to fit climb (time budget exhausted by cycles), but the builder
    // must ALWAYS have climb awareness when the user configured a climb. Without this,
    // climbTarget would be null and ALL runtime time gates would be disabled — the robot
    // would blindly execute every cycle with no climb attempt.
    final boolean shouldClimb = settings.shouldAttemptClimb();
    final Pose2d climbTarget = shouldClimb ? settings.getClimbPose().getPose() : null;
    final AutoAction.Climb climbAction =
        shouldClimb
            ? new AutoAction.Climb(settings.getClimbLevel(), settings.getClimbPose())
            : null;

    for (int i = 0; i < cycleActions.size(); i += 2) {
      AutoAction intakeAction = cycleActions.get(i);
      AutoAction scoreAction = (i + 1 < cycleActions.size()) ? cycleActions.get(i + 1) : null;

      final int cycleNum = (i / 2) + 1;
      final boolean isLastPlannedCycle = (i + 2 >= cycleActions.size());

      // Determine the scoring strategy from the planner's action.
      // SWD = the robot drives to the next destination while shooting mid-transit.
      // Stop-and-shoot = the robot drives to the scoring waypoint, stops, aims, fires.
      boolean isSWDScore =
          (scoreAction instanceof AutoAction.ScoreAt)
              && ((AutoAction.ScoreAt) scoreAction).isShootWhileMoving();

      // Build the intake command (always runs — no time gate before intake)
      Command intakeCmd =
          Commands.sequence(logStep(intakeAction), buildActionCommand(intakeAction));

      if (scoreAction == null) {
        // Odd trailing intake with no score — just run it (gated by climb merge)
        sequence.addCommands(
            Commands.either(
                Commands.print(
                    "[DashboardAuto] Cycle " + cycleNum + " skipped (climb already merged)"),
                intakeCmd,
                () -> climbMergedAtRuntime));
        continue;
      }

      // Build the full cycle command (intake + score/climb logic).
      // This will be gated below — if a previous cycle already merged with climb,
      // the entire cycle is skipped.
      Command cycleCmd;

      if (!isSWDScore) {
        // ===== STOP-AND-SHOOT CYCLE: Time check AFTER arriving at intake =====
        // After intake, check: do we have time to drive to scoring waypoint, shoot,
        // and still reach the tower to climb?
        //
        // If YES: stop-and-shoot at the scoring waypoint, then continue to next cycle.
        // If NO:  skip the score, SWD directly to the tower while extending climb arms.
        //
        // EXCEPTION: last planned cycle before climb always merges with climb (SWD to
        // tower). No point stopping to shoot then driving to tower separately — SWD
        // from neutral to tower lets us score mid-transit and saves time.

        final AutoAction.ScoreAt scoreAt = (AutoAction.ScoreAt) scoreAction;
        final Pose2d scorePose = scoreAt.getLocation().toPose();

        Command normalScoreCmd =
            Commands.sequence(logStep(scoreAction), buildActionCommand(scoreAction));

        if (isLastPlannedCycle && climbTarget != null) {
          // Last stop-and-shoot cycle before climb — always merge: SWD to tower
          Command climbMerged = buildDriveAndClimb(climbTarget, climbAction);
          cycleCmd =
              Commands.sequence(
                  intakeCmd,
                  Commands.runOnce(
                      () -> {
                        climbMergedAtRuntime = true;
                        Logger.recordOutput(
                            "DashboardAuto/CycleTimeCheck",
                            String.format(
                                "StopShoot Cycle %d (last): merging with climb (%.1fs remaining)",
                                cycleNum, getAutoTimeRemaining()));
                      }),
                  climbMerged);
        } else if (climbTarget != null) {
          // Build the climb fallback (SWD to tower + extend + retract)
          Command climbMerged = buildDriveAndClimb(climbTarget, climbAction);

          cycleCmd =
              Commands.sequence(
                  intakeCmd,
                  Commands.either(
                      // Enough time → normal stop-and-shoot
                      Commands.sequence(
                          Commands.runOnce(
                              () ->
                                  Logger.recordOutput(
                                      "DashboardAuto/CycleTimeCheck",
                                      String.format(
                                          "StopShoot Cycle %d: scoring (%.1fs remaining)",
                                          cycleNum, getAutoTimeRemaining()))),
                          normalScoreCmd),
                      // Not enough time → SWD to tower + climb
                      Commands.sequence(
                          Commands.runOnce(
                              () -> {
                                climbMergedAtRuntime = true;
                                Logger.recordOutput(
                                    "DashboardAuto/CycleTimeCheck",
                                    String.format(
                                        "StopShoot Cycle %d: skip → SWD to climb (%.1fs remaining)",
                                        cycleNum, getAutoTimeRemaining()));
                              }),
                          climbMerged),
                      // Condition: enough time for THIS score + climb?
                      () -> hasTimeForULScore(drive.getPose(), scorePose, settingsRef)));
        } else {
          // No climb configured — always score if there's any time at all
          cycleCmd = Commands.sequence(intakeCmd, normalScoreCmd);
        }

      } else {
        // ===== SWD CYCLE: Shoot-while-driving, time check before NEXT cycle =====
        // The robot SWDs to its next destination while shooting mid-transit.
        // This covers D/O intakes (always SWD) and U/L intakes whose next target
        // is in the alliance zone (SWD back through the HUB zone). The time check
        // determines whether to start another cycle or merge the SWD with climb.

        Command normalScoreCmd =
            Commands.sequence(logStep(scoreAction), buildActionCommand(scoreAction));

        if (isLastPlannedCycle && climbTarget != null) {
          // Last planned SWD cycle before climb — always merge: SWD to tower.
          // There's no next cycle to save time for, so merging is always optimal.
          // The robot drives directly to the tower while shooting mid-transit,
          // instead of detouring to the HUB scoring waypoint first.
          // (Mirrors the stop-and-shoot last-cycle logic above.)
          Command climbMerged = buildDriveAndClimb(climbTarget, climbAction);
          cycleCmd =
              Commands.sequence(
                  intakeCmd,
                  Commands.runOnce(
                      () -> {
                        climbMergedAtRuntime = true;
                        Logger.recordOutput(
                            "DashboardAuto/CycleTimeCheck",
                            String.format(
                                "SWD Cycle %d (last): merging with climb (%.1fs remaining)",
                                cycleNum, getAutoTimeRemaining()));
                      }),
                  climbMerged);
        } else if (climbTarget != null) {
          // Not the last planned cycle — check at intake whether there's time for THIS SWD + climb
          Command mergedCmd = buildDriveAndClimb(climbTarget, climbAction);
          cycleCmd =
              Commands.sequence(
                  intakeCmd,
                  Commands.either(
                      // Enough time for this SWD + climb → normal SWD score
                      Commands.sequence(
                          Commands.runOnce(
                              () ->
                                  Logger.recordOutput(
                                      "DashboardAuto/CycleTimeCheck",
                                      String.format(
                                          "SWD Cycle %d: normal SWD (%.1fs remaining)",
                                          cycleNum, getAutoTimeRemaining()))),
                          normalScoreCmd),
                      // Not enough → merge this SWD with climb
                      Commands.sequence(
                          Commands.runOnce(
                              () -> {
                                climbMergedAtRuntime = true;
                                Logger.recordOutput(
                                    "DashboardAuto/CycleTimeCheck",
                                    String.format(
                                        "SWD Cycle %d: merging SWD with climb (%.1fs remaining)",
                                        cycleNum, getAutoTimeRemaining()));
                              }),
                          mergedCmd),
                      // Condition: enough time for THIS SWD score + climb?
                      () -> hasTimeForNextCycle(drive.getPose(), settingsRef)));
        } else {
          // No climb configured — always SWD
          cycleCmd = Commands.sequence(intakeCmd, normalScoreCmd);
        }
      }

      // Gate the entire cycle: if a previous cycle already merged with climb at runtime,
      // skip this cycle entirely. This prevents the robot from continuing to intake/score
      // after it has already started climbing.
      sequence.addCommands(
          Commands.either(
              Commands.print(
                  "[DashboardAuto] Cycle " + cycleNum + " skipped (climb already merged)"),
              cycleCmd,
              () -> climbMergedAtRuntime));
    }

    // ===== Phase 3: Post-cycle (DriveTo TOWER + Climb) =====
    // When a cycle's score was merged with climb at runtime (climbMergedAtRuntime), the
    // DriveTo+Climb pair was already handled — skip it here.
    boolean climbHandledInPostCycle = false;

    for (int i = 0; i < postCycleActions.size(); i++) {
      AutoAction action = postCycleActions.get(i);

      // If this is a DriveTo followed by a Climb, merge them: shoot on the way,
      // enter climb mode, extend arms while driving, retract when arrived.
      if (action.getType() == AutoAction.Type.DRIVE_TO
          && i + 1 < postCycleActions.size()
          && postCycleActions.get(i + 1).getType() == AutoAction.Type.CLIMB) {
        climbHandledInPostCycle = true;
        // Use Commands.either to check at runtime whether climb was already consumed
        AutoAction.Climb postClimbAction = (AutoAction.Climb) postCycleActions.get(i + 1);
        Pose2d postClimbTarget = action.getTargetPose();
        Command driveAndClimbCmd = buildDriveAndClimb(postClimbTarget, postClimbAction);

        sequence.addCommands(
            Commands.either(
                // Climb was already merged into the last cycle's score — skip
                Commands.print("[DashboardAuto] DriveTo+Climb already consumed by cycle merge"),
                // Normal path: build merged drive+climb command
                Commands.sequence(logStep(action), driveAndClimbCmd),
                // Condition: was climb already merged at runtime?
                () -> climbMergedAtRuntime));
        // Skip the Climb action — handled by either branch
        i++;
      } else {
        sequence.addCommands(logStep(action));
        Command cmd = buildActionCommand(action);
        if (cmd != null) sequence.addCommands(cmd);
      }
    }

    // ===== Fallback climb: settings say climb, but planner didn't include it =====
    // This catches the case where the planner ran out of time budget for climb.
    // The builder always ensures a climb attempt when the user configured one.
    if (shouldClimb && !climbHandledInPostCycle) {
      Command fallbackClimb = buildDriveAndClimb(climbTarget, climbAction);
      sequence.addCommands(
          Commands.either(
              Commands.print("[DashboardAuto] Fallback climb skipped (already merged by cycle)"),
              Commands.sequence(
                  Commands.runOnce(
                      () ->
                          Logger.recordOutput(
                              "DashboardAuto/CurrentStep",
                              "FALLBACK_CLIMB (planner didn't plan climb)")),
                  fallbackClimb),
              () -> climbMergedAtRuntime));
    }

    sequence.addCommands(
        Commands.runOnce(() -> Logger.recordOutput("DashboardAuto/CurrentStep", "COMPLETE")));
    // Stop intake at the end of auto
    sequence.addCommands(superstructure.idle());
    sequence.addCommands(Commands.print("[DashboardAuto] Auto sequence complete!"));

    sequence.setName("DashboardAuto");

    // Wrap the sequence with a background command that continuously logs the auto timer
    // every robot cycle (20ms). The deadline ends when the sequence finishes.
    return Commands.deadline(
            sequence,
            Commands.run(
                () ->
                    Logger.recordOutput("DashboardAuto/AutoTimeRemaining", getAutoTimeRemaining())))
        .withName("DashboardAuto");
  }

  // ===== Runtime Time Management =====
  //
  // All time checks happen AT THE INTAKE POSE (after arriving), not before starting
  // the cycle. This gives the most accurate time estimate since we know exactly where
  // the robot is.
  //
  // We track time ourselves using Timer.getFPGATimestamp() captured at auto start,
  // so time checks work everywhere: sim, practice, and competition.

  /**
   * Get seconds remaining in the auto period. Uses our own FPGA-based countdown so it works in sim,
   * practice, and competition (unlike Timer.getMatchTime() which returns -1 without FMS).
   *
   * <p>If the auto start timestamp hasn't been captured yet (shouldn't happen), returns
   * AUTO_DURATION as a safe fallback.
   */
  private double getAutoTimeRemaining() {
    if (autoStartTimestamp <= 0.0) {
      return FieldConstants.AUTO_DURATION; // fallback: assume full time
    }
    double elapsed = Timer.getFPGATimestamp() - autoStartTimestamp;
    return Math.max(0.0, FieldConstants.AUTO_DURATION - elapsed);
  }

  /**
   * Estimate the time needed to drive to the TOWER from a given pose and climb. Drive time is
   * scaled by {@link AutoTuning#RUNTIME_DRIVE_TIME_MULTIPLIER} to account for PathPlanner AD* paths
   * being longer than straight-line.
   *
   * @param fromPose The pose from which the robot will drive to the tower
   * @param settings The auto settings with climb configuration
   * @return Estimated seconds for drive + climb + safety, or 0 if no climb planned
   */
  private static double estimateClimbReserve(Pose2d fromPose, AutoSettings settings) {
    if (!settings.shouldAttemptClimb()) {
      return 0.0;
    }
    ClimbLevel cl = settings.getClimbLevel();
    double driveToTower =
        FieldConstants.estimateDriveTime(fromPose, settings.getClimbPose().getPose())
            * RUNTIME_DRIVE_TIME_MULTIPLIER;
    // Drive time (corrected for PathPlanner pathing) + climb duration + 0.5s safety margin
    return driveToTower + cl.estimatedClimbDuration + 0.5;
  }

  /**
   * <b>Stop-and-shoot decision</b>: After arriving at an intake, check whether there's enough time
   * to drive to the scoring waypoint, stop-and-shoot, and still reach the TOWER to climb.
   *
   * <p>This method only checks the CURRENT score — it does NOT look ahead at the next cycle. Each
   * cycle makes its own time decision independently when it reaches its intake pose.
   *
   * <p>Time budget: driveToScore * RUNTIME_DRIVE_TIME_MULTIPLIER + STOP_AND_SHOOT_DURATION +
   * climbReserve
   *
   * @param currentPose Robot's pose at the intake location
   * @param scorePose The stop-and-shoot scoring waypoint
   * @param settings Auto settings for climb config
   * @return true if there's enough time to score and still climb
   */
  private boolean hasTimeForULScore(Pose2d currentPose, Pose2d scorePose, AutoSettings settings) {
    double remaining = getAutoTimeRemaining();

    double driveToScore =
        FieldConstants.estimateDriveTime(currentPose, scorePose) * RUNTIME_DRIVE_TIME_MULTIPLIER;
    double shootDuration = STOP_AND_SHOOT_DURATION;
    // Climb reserve estimated from the scoring pose (where we'll be after shooting)
    double climbReserve = estimateClimbReserve(scorePose, settings);

    double needed = driveToScore + shootDuration + climbReserve;

    Logger.recordOutput("DashboardAuto/AutoTimeRemaining", remaining);
    Logger.recordOutput("DashboardAuto/ULScoreEstimate", needed);
    Logger.recordOutput(
        "DashboardAuto/ULScoreBreakdown",
        String.format(
            "drive=%.1f shoot=%.1f climb=%.1f", driveToScore, shootDuration, climbReserve));

    return remaining > needed;
  }

  /**
   * <b>SWD decision</b>: After arriving at an intake, check whether there's enough time for a
   * normal SWD score and still reach the TOWER to climb. This method only checks the CURRENT SWD —
   * it does NOT look ahead at the next cycle. Each cycle makes its own time decision.
   *
   * <p>Time budget: SWD_SCORE_DURATION + climbReserve
   *
   * @param currentPose Robot's pose at the intake location
   * @param settings Auto settings for climb config
   * @return true if there's enough time for a normal SWD + separate climb
   */
  private boolean hasTimeForNextCycle(Pose2d currentPose, AutoSettings settings) {
    double remaining = getAutoTimeRemaining();

    double swdScoreTime = SWD_SCORE_DURATION;
    double climbReserve = estimateClimbReserve(currentPose, settings);

    double needed = swdScoreTime + climbReserve;

    Logger.recordOutput("DashboardAuto/AutoTimeRemaining", remaining);
    Logger.recordOutput("DashboardAuto/DOLastCycleEstimate", needed);

    return remaining > needed;
  }

  // ===== FUEL Detection Commands =====
  // Detection logic lives in Superstructure (runs every periodic cycle).
  // These commands just reset the detection state and wait on the boolean getters.

  /**
   * Build a command that waits until the Superstructure detects FUEL in the intake path (lower
   * roller current above threshold), or handles the case where no FUEL was picked up.
   *
   * <p><b>Logic:</b> The Superstructure continuously monitors the lower intake roller current.
   * While FUEL is present, the roller is loaded and current stays high. If the current drops below
   * the threshold for 0.5s, "no fuel" is declared. The detection is reset before each intake drive.
   *
   * <p>Upon arrival at the intake pose:
   *
   * <ul>
   *   <li><b>FUEL detected (intakeHasFuel):</b> Continue immediately — no waiting.
   *   <li><b>No FUEL detected:</b> Nudge 1m toward field center Y (deeper into the scatter zone),
   *       then wait briefly for a pickup with timeout.
   * </ul>
   *
   * <p><b>In simulation</b>, detection is bypassed — instant success.
   *
   * @param intakePose The nominal intake pose (used to compute nudge direction)
   * @return Command that checks FUEL presence and nudges if empty
   */
  private Command waitForIntakePickup(Pose2d intakePose) {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return Commands.runOnce(
              () -> Logger.recordOutput("DashboardAuto/IntakeDetect", "SIM: instant success"))
          .withName("IntakeDetect_Sim");
    }

    // Real robot: check Superstructure detection upon arrival
    return Commands.either(
            // FUEL detected during the drive — continue immediately
            Commands.runOnce(
                () ->
                    Logger.recordOutput(
                        "DashboardAuto/IntakeDetect", "FUEL detected during drive — continuing")),
            // No FUEL detected — nudge + retry
            Commands.sequence(
                Commands.runOnce(
                    () ->
                        Logger.recordOutput(
                            "DashboardAuto/IntakeDetect",
                            "No FUEL during drive — nudging 1m toward center Y")),
                // Drive 1m toward field center Y with intake still running
                pathfindTo(computeNudgePose(intakePose)),
                // After nudge: check again
                Commands.either(
                    Commands.runOnce(
                        () ->
                            Logger.recordOutput(
                                "DashboardAuto/IntakeDetect",
                                "FUEL detected during nudge — continuing")),
                    // Still nothing — wait briefly with timeout, then give up
                    Commands.sequence(
                        Commands.race(
                            Commands.waitUntil(() -> superstructure.intakeHasFuel()),
                            Commands.waitSeconds(INTAKE_NUDGE_TIMEOUT_SECONDS)),
                        Commands.runOnce(
                            () ->
                                Logger.recordOutput(
                                    "DashboardAuto/IntakeDetect",
                                    superstructure.intakeHasFuel()
                                        ? "FUEL detected after nudge wait"
                                        : "No FUEL after nudge — giving up"))),
                    () -> superstructure.intakeHasFuel())),
            // Condition: does the Superstructure detect FUEL?
            () -> superstructure.intakeHasFuel())
        .withName("IntakeDetect");
  }

  /**
   * Compute a pose 1m toward the field center Y from the given intake pose. The X coordinate stays
   * the same; the Y moves toward {@code FIELD_WIDTH / 2.0}. Heading preserved from original pose.
   *
   * <p>This works in alliance-corrected coordinates (the intake pose is already alliance-flipped
   * via {@link FieldConstants.IntakeLocation#getPose()}).
   */
  private static Pose2d computeNudgePose(Pose2d intakePose) {
    double centerY = FieldConstants.FIELD_WIDTH / 2.0;
    double currentY = intakePose.getY();
    double deltaY = centerY - currentY;
    // Normalize direction and scale to INTAKE_NUDGE_DISTANCE_METERS
    double nudgeY;
    if (Math.abs(deltaY) < 0.01) {
      nudgeY = 0; // Already at center — no nudge
    } else {
      nudgeY = Math.signum(deltaY) * INTAKE_NUDGE_DISTANCE_METERS;
    }
    return new Pose2d(intakePose.getX(), currentY + nudgeY, intakePose.getRotation());
  }

  /**
   * Build a command that waits until the Superstructure detects all FUEL has been fired, based on
   * conveyor motor current dropping below threshold for a sustained period.
   *
   * <p><b>Logic:</b> The Superstructure continuously monitors the conveyor current. While FUEL is
   * being pushed into the shooter, the conveyor is loaded and current stays high. When all FUEL has
   * exited and the conveyor is spinning freely, current drops. After the current stays below the
   * threshold for 0.5s, "shooter finished" is declared.
   *
   * <p><b>In simulation</b>, uses a fixed time delay since sim IO doesn't model FUEL load.
   *
   * <p>The command finishes when either:
   *
   * <ol>
   *   <li>Superstructure reports all FUEL fired ({@link Superstructure#isShooterFinishedFiring()}).
   *   <li>Timeout reached (safety fallback — move on regardless).
   * </ol>
   *
   * @return Command that waits for shooter to finish firing
   */
  private Command waitForShooterEmpty() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return Commands.sequence(
              Commands.waitSeconds(SIM_SHOOTER_DONE_SECONDS),
              Commands.runOnce(
                  () -> Logger.recordOutput("DashboardAuto/ShooterDetect", "SIM: fixed delay")))
          .withName("ShooterDetect_Sim");
    }

    // Real robot: reset detection, then wait for Superstructure to declare "done"
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  superstructure.resetShooterDetection();
                  Logger.recordOutput("DashboardAuto/ShooterDetect", "Waiting for FUEL to exit");
                }),
            Commands.race(
                Commands.waitUntil(() -> superstructure.isShooterFinishedFiring()),
                Commands.waitSeconds(SHOOTER_DETECT_TIMEOUT_SECONDS)))
        .finallyDo(
            () ->
                Logger.recordOutput(
                    "DashboardAuto/ShooterDetect",
                    superstructure.isShooterFinishedFiring()
                        ? "All FUEL fired"
                        : "Timeout — moving on"))
        .withName("ShooterDetect");
  }

  // ===== Per-Action Command Builders =====

  /** Build a log command for an action step. */
  private Command logStep(AutoAction action) {
    return Commands.runOnce(
        () ->
            Logger.recordOutput(
                "DashboardAuto/CurrentStep", action.getType() + ": " + action.describe()));
  }

  private Command buildActionCommand(AutoAction action) {
    switch (action.getType()) {
      case SET_START_POSE:
        return buildSetStartPose((AutoAction.SetStartPose) action);
      case SCORE_PRELOAD:
        return buildScorePreload((AutoAction.ScorePreload) action);
      case SCORE_AT:
        return buildScoreAt((AutoAction.ScoreAt) action);
      case INTAKE_AT:
        return buildIntakeAt((AutoAction.IntakeAt) action);
      case DRIVE_TO:
        return buildDriveTo((AutoAction.DriveTo) action);
      case CLIMB:
        return buildClimb((AutoAction.Climb) action);
      default:
        return Commands.print("[DashboardAuto] Unknown action type: " + action.getType());
    }
  }

  /**
   * 254-style: The robot's pose was already pre-seeded during disabled and refined by vision.
   * Instead of hard-resetting odometry (which would discard vision corrections), we just log the
   * expected starting pose for verification. The pose estimator should already be close to this
   * pose thanks to the disabled pre-seeding strategy.
   */
  private Command buildSetStartPose(AutoAction.SetStartPose action) {
    return Commands.runOnce(
            () -> {
              Logger.recordOutput("DashboardAuto/ExpectedStartPose", action.getPose());
              Logger.recordOutput("DashboardAuto/ActualStartPose", drive.getPose());
              double poseError =
                  drive.getPose().getTranslation().getDistance(action.getPose().getTranslation());
              Logger.recordOutput("DashboardAuto/StartPoseErrorMeters", poseError);
              if (poseError > 1.0) {
                System.out.println(
                    "[DashboardAuto] WARNING: Start pose error is "
                        + String.format("%.2f", poseError)
                        + "m — was the robot placed correctly?");
              }
            })
        .withName("VerifyStartPose");
  }

  /**
   * Score the preloaded FUEL. Two modes:
   *
   * <ul>
   *   <li><b>Stop-and-shoot:</b> Drive to scoring waypoint, stop, aim, fire.
   *   <li><b>SWD:</b> Drive directly to the first intake location while dumping preloaded FUEL
   *       mid-transit. The turret tracks the HUB in real-time as the robot passes through.
   * </ul>
   */
  private Command buildScorePreload(AutoAction.ScorePreload action) {
    if (action.isShootWhileMoving()) {
      Pose2d swdTarget =
          action.getSwdDestination() != null
              ? action.getSwdDestination()
              : action.getLocation().toPose();
      return buildShootWhileDriving(swdTarget, action.getLocation())
          .withName("ScorePreload_SWD_" + action.getLocation().name());
    } else {
      Pose2d target = action.getLocation().toPose();
      return Commands.sequence(
              Commands.print("[DashboardAuto] Scoring preload at " + action.getLocation().name()),
              // Drive to scoring waypoint with zone-aware intake (aims when in alliance/hub zone)
              Commands.deadline(pathfindTo(target), zoneAwareIntake()),
              buildStopAndShootSequence())
          .withName("ScorePreload_" + action.getLocation().name());
    }
  }

  /**
   * Drive to a HUB shooting position and score FUEL.
   *
   * <p><b>Two modes:</b>
   *
   * <ul>
   *   <li><b>Stop-and-shoot (default):</b> Drive to scoring waypoint → stop → aim → shoot → idle.
   *       Most accurate, but costs ~0.5s extra per cycle.
   *   <li><b>Shoot-while-driving:</b> The robot does NOT detour to the scoring waypoint. Instead,
   *       it drives directly to the next destination ({@code swdDestination}, typically the next
   *       intake location) at full speed while the turret tracks and shoots the HUB in real-time
   *       mid-transit. The robot passes through the alliance/HUB zone naturally, firing FUEL as it
   *       goes. Saves ~1-2s per cycle by eliminating the scoring waypoint detour entirely.
   * </ul>
   */
  private Command buildScoreAt(AutoAction.ScoreAt action) {
    if (action.isShootWhileMoving()) {
      // SWD: drive to the actual destination (next intake) while shooting mid-transit
      Pose2d swdTarget =
          action.getSwdDestination() != null
              ? action.getSwdDestination()
              : action.getLocation().toPose(); // fallback for last cycle
      return buildShootWhileDriving(swdTarget, action.getLocation());
    } else {
      // Stop-and-shoot: drive to scoring waypoint, stop, aim, fire
      return buildStopAndScore(action.getLocation().toPose(), action.getLocation());
    }
  }

  /**
   * Traditional stop-and-shoot: drive to pose, stop, aim, fire, resume intake.
   *
   * <p>Zone-aware: while driving, uses ONLY_INTAKE in the neutral zone and AIMING_WHILE_INTAKING in
   * the alliance/HUB zone. By the time the robot arrives at the scoring waypoint (always in
   * alliance/HUB zone), the turret is already tracking the HUB.
   */
  private Command buildStopAndScore(Pose2d target, ScoringWaypoint location) {
    return Commands.sequence(
            Commands.print("[DashboardAuto] Stop-and-shoot at " + location.name()),
            // Drive to scoring waypoint with zone-aware intake (aims when in alliance/hub zone)
            Commands.deadline(pathfindTo(target), zoneAwareIntake()),
            buildStopAndShootSequence())
        .withName("ScoreAt_" + location.name());
  }

  /**
   * SWD wrapper for mid-cycle scoring. Delegates to {@link #buildShootWhileDrivingCore}.
   *
   * @param destination Where the robot is actually driving to (next intake location)
   * @param location The scoring waypoint (for logging/naming only — robot doesn't drive here)
   */
  private Command buildShootWhileDriving(Pose2d destination, ScoringWaypoint location) {
    return buildShootWhileDrivingCore(destination, location.name())
        .withName("ScoreAt_SWD_" + location.name());
  }

  /**
   * Core shoot-while-driving logic shared by both mid-cycle SWD and climb-transit SWD. The robot
   * drives to {@code destination} at full speed while shooting the HUB mid-transit. The turret
   * tracks in real-time via ShooterSetpoint.
   *
   * <p>Architecture: a single {@link Commands#deadline} with the pathfind as backbone and a
   * continuous {@link Commands#run} loop that checks zone, hood readiness, and acceleration every
   * cycle:
   *
   * <ul>
   *   <li><b>Neutral/opponent zone:</b> ONLY_INTAKE (turret stowed, just collect FUEL)
   *   <li><b>Aiming zone, hood not at setpoint:</b> AIMING_WHILE_INTAKING (turret tracks, hood
   *       deploying, no feed)
   *   <li><b>Aiming zone, hood ready, high accel:</b> AIMING_WHILE_INTAKING (turret tracks, no
   *       feed)
   *   <li><b>Aiming zone, hood ready, low accel:</b> SHOOTING_WHILE_INTAKING (turret tracks + feed)
   * </ul>
   *
   * <p>Acceleration gating uses hysteresis ({@link #FEED_ACCEL_HYSTERESIS}) to prevent stutter. The
   * robot shoots whenever acceleration is low AND hood is settled — not just once — and pauses
   * feeding when accelerating hard or hood leaves setpoint. When the path ends, a zone-aware
   * default state is applied.
   *
   * @param destination Where the robot is actually driving to
   * @param logLabel Label for logging (scoring waypoint name or "TOWER_TRANSIT")
   */
  private Command buildShootWhileDrivingCore(Pose2d destination, String logLabel) {
    return Commands.sequence(
        Commands.print("[DashboardAuto] Shoot-while-driving (" + logLabel + ")"),
        // Drive at full speed to destination while continuously shooting when possible
        Commands.deadline(
            // Deadline: the path finishing is what ends this group
            pathfindTo(destination),
            // Continuous zone-aware shooting: every cycle, check zone + hood + accel and
            // toggle between AIMING_WHILE_INTAKING and SHOOTING_WHILE_INTAKING. This
            // ensures the robot shoots whenever accel is low and hood is settled (not just
            // once) and stops shooting when accel spikes, hood leaves setpoint, or when
            // in neutral/opponent zone.
            Commands.run(
                () -> {
                  updateZoneAwareFeeding(
                      Superstructure.SuperstructureState.SHOOTING_WHILE_INTAKING,
                      Superstructure.SuperstructureState.AIMING_WHILE_INTAKING,
                      Superstructure.SuperstructureState.ONLY_INTAKE);
                  Logger.recordOutput("DashboardAuto/SWD/Label", logLabel);
                  Logger.recordOutput("DashboardAuto/SWD/Feeding", zoneAwareFeeding);
                },
                superstructure))
        // No zoneAwareDefaultState here — the next command (IntakeAt, StopAndScore, etc.)
        // will set the appropriate state. Avoiding an intermediate AIMING state prevents
        // a visible SHOOTING→AIMING→SHOOTING flicker at command transitions.
        );
  }

  /**
   * Drive to a FUEL intake location (OUTPOST, DEPOT, or NEUTRAL ZONE) and collect FUEL.
   *
   * <p>All intake approaches use {@link #zoneAwareIntake()} as a deadline-parallel with the path.
   * The zone-aware command continuously manages the superstructure state based on the robot's field
   * position and acceleration:
   *
   * <ul>
   *   <li><b>Alliance / HUB zone, low accel:</b> SHOOTING_WHILE_INTAKING — opportunistically dumps
   *       any loaded FUEL into the HUB while driving through.
   *   <li><b>Alliance / HUB zone, high accel:</b> AIMING_WHILE_INTAKING — turret tracks HUB but
   *       doesn't feed (shots inaccurate while accelerating).
   *   <li><b>Neutral zone:</b> ONLY_INTAKE — turret stowed, just collect FUEL.
   * </ul>
   *
   * <p>This replaces the previous approach of using {@code buildShootWhileDrivingCore} for
   * alliance-zone intakes (OUTPOST/DEPOT). The one-shot SWD core's settle+shoot sequence was
   * getting killed by the deadline on short alliance-zone paths before it could complete. {@code
   * zoneAwareIntake()} is continuous (polled every cycle) so it works regardless of path length.
   *
   * <p><b>Current-based pickup detection:</b> The Superstructure continuously monitors the lower
   * intake roller current. The detection is reset before the drive starts. If the lower roller
   * current stays above the threshold during the drive (FUEL present), {@code intakeHasFuel()}
   * returns true. Upon arrival, the detection is checked — if FUEL detected, continue; if not,
   * nudge 1m toward center Y and retry briefly.
   */
  private Command buildIntakeAt(AutoAction.IntakeAt action) {
    FieldConstants.IntakeLocation loc = action.getLocation();
    Pose2d intakePose = loc.getPose();

    // All intakes use the same logic: zone-aware intake during drive, then check
    // Superstructure's intake detection on arrival. Detection runs in Superstructure's
    // periodic() — no separate monitor command needed.
    return Commands.sequence(
            Commands.print("[DashboardAuto] Intaking at " + loc.name()),
            // Reset the intake detection before starting the drive
            Commands.runOnce(() -> superstructure.resetIntakeDetection()),
            // Drive with zone-aware intake (detection runs automatically in Superstructure)
            Commands.deadline(pathfindTo(intakePose), zoneAwareIntake()),
            // Upon arrival: check detection — nudge if no FUEL detected during drive
            waitForIntakePickup(intakePose))
        .withName("IntakeAt_" + loc.name());
  }

  /** Drive to an arbitrary pose. */
  private Command buildDriveTo(AutoAction.DriveTo action) {
    Pose2d target = action.getTarget();
    return Commands.sequence(
            Commands.print("[DashboardAuto] Driving to " + action.getLabel()), pathfindTo(target))
        .withName("DriveTo_" + action.getLabel());
  }

  /**
   * Combined DriveTo + Climb: drive toward the TOWER while overlapping shooting, intake stow, climb
   * extend, and scoring idle — then retract once both drive and extend are complete.
   *
   * <p>Uses {@link Commands#parallel} (not deadline) so the climb extend can finish its full motion
   * path even if the drivetrain arrives first. Two groups run simultaneously:
   *
   * <ol>
   *   <li><b>Drive + shoot group (deadline):</b> Pathfind to climb pose while continuously shooting
   *       with accel gating. Intake is stowed (ONLY_AIMING / ONLY_SHOOTING) since we're heading to
   *       climb. When within 1m, idle all scoring. Ends when drive arrives.
   *   <li><b>Climb extend:</b> Waits for intake to physically reach stow, then runs the full extend
   *       motion path. NOT inside a deadline, so the motion completes even if the drive arrives
   *       first.
   * </ol>
   *
   * <p>Both groups must finish before retract begins (parallel semantics).
   *
   * @param climbTarget The TOWER climb pose to pathfind to
   * @param climbAction The Climb action (for level and logging)
   */
  private Command buildDriveAndClimb(Pose2d climbTarget, AutoAction.Climb climbAction) {
    return Commands.sequence(
            Commands.print(
                "[DashboardAuto] Driving to TOWER & climbing "
                    + climbAction.getClimbLevel().name()),
            // Parallel: both drive+shoot AND climb extend must finish before retract
            Commands.parallel(
                // Group 1: Drive to climb pose with continuous accel-gated shooting
                Commands.deadline(
                    pathfindThenDriveStraight(climbTarget),
                    // Continuous zone-aware shooting loop: ONLY_AIMING when accel high,
                    // ONLY_SHOOTING when accel low. No intake (stowing for climb).
                    // IDLE when outside alliance zone (turret stowed — can't score from
                    // neutral zone) or within 1m of climb pose (final approach).
                    Commands.run(
                        () -> {
                          double distToTarget =
                              drive
                                  .getPose()
                                  .getTranslation()
                                  .getDistance(climbTarget.getTranslation());
                          if (distToTarget <= 1.0) {
                            // Within 1m of climb pose — stow everything for final approach
                            zoneAwareFeeding = false;
                            superstructure.forceWantedState(
                                Superstructure.SuperstructureState.IDLE);
                          } else {
                            updateZoneAwareFeeding(
                                Superstructure.SuperstructureState.ONLY_SHOOTING,
                                Superstructure.SuperstructureState.ONLY_AIMING,
                                Superstructure.SuperstructureState.IDLE);
                          }
                          Logger.recordOutput(
                              "DashboardAuto/ClimbTransit/Feeding", zoneAwareFeeding);
                        },
                        superstructure)),
                // Group 2: Climb extend — waits for stow, then runs full motion path
                Commands.sequence(
                    Commands.waitUntil(() -> superstructure.isIntakeStowed()),
                    Commands.print("[DashboardAuto] Intake stowed — extending climb"),
                    climb.setStateCommand(ClimbState.EXTEND_L1_AUTO))),
            // Both drive and extend complete — retract to pull robot up
            Commands.print("[DashboardAuto] Reached climb pose & extended — retracting"),
            climb.setStateCommand(ClimbState.RETRACT_L1_AUTO))
        .withName("DriveAndClimb_" + climbAction.getClimbLevel().name());
  }

  /**
   * Standalone climb (fallback when no DriveTo precedes the Climb action). Idles the superstructure
   * and executes extend → retract at the current position. Climb is independent — same system as
   * teleop.
   */
  private Command buildClimb(AutoAction.Climb action) {
    return Commands.sequence(
            Commands.print("[DashboardAuto] Climbing TOWER " + action.getClimbLevel().name()),
            // Idle superstructure so intake stows (clearance for arms)
            superstructure.idle(),
            Commands.waitUntil(() -> superstructure.isIntakeStowed()),
            // Extend arms then retract — same ClimbSubsystem API as teleop
            climb.setStateCommand(ClimbState.EXTEND_L1_AUTO),
            Commands.print("[DashboardAuto] Retracting"),
            climb.setStateCommand(ClimbState.RETRACT_L1_AUTO))
        .withName("Climb_" + action.getClimbLevel().name());
  }

  // ===== Shared Helpers =====

  /**
   * Build the stop-and-shoot sequence: shoot immediately → wait for all FUEL to exit → resume.
   *
   * <p>By the time this runs, the turret/hood/shooter have been continuously tracking the HUB via
   * {@link #zoneAwareIntake()} during the entire drive approach. The robot is already stationary
   * (stop-and-shoot pattern) and the mechanisms are already stabilized — no additional aim+settle
   * is needed. Going straight to SHOOTING avoids a SHOOTING→AIMING→SHOOTING flicker that was
   * visible when the robot was already feeding via zoneAwareIntake before the pathfind ended.
   *
   * <p><b>Current-based shot detection:</b> Instead of a fixed 0.15s wait, we now use {@link
   * #waitForShooterEmpty()} which delegates to the Superstructure's conveyor current monitoring.
   * When the conveyor current drops below the threshold for a sustained period, all FUEL has been
   * fired. A timeout fallback prevents getting stuck.
   *
   * @return Command sequence to fire FUEL at the HUB while stationary
   */
  private Command buildStopAndShootSequence() {
    return Commands.sequence(
        // Fire immediately — turret was already tracking via zoneAwareIntake during the drive
        superstructure.shootingWhileIntaking(),
        // Wait for all FUEL to exit (current-based detection with timeout fallback)
        waitForShooterEmpty(),
        // Return to zone-aware default (aim in alliance zone, intake-only outside)
        zoneAwareDefaultState());
  }

  /**
   * Get the path constraints for pathfinding. Delegates to the drive subsystem's single source of
   * truth (which reads from Constants.AutoConstants).
   */
  private PathConstraints getPathConstraints() {
    return drive.getPathConstraints();
  }

  /**
   * Build a deferred pathfind command to a target pose, automatically snapping the heading to a
   * cardinal direction (0°/90°/180°/270°) if the target is near a TRENCH. This ensures the robot
   * arrives at the trench already aligned to fit through the 22.25in tunnel.
   *
   * @param target The desired target pose
   * @return A deferred pathfinding command with trench-aware heading
   */
  private Command pathfindTo(Pose2d target) {
    return Commands.defer(
        () -> AutoBuilder.pathfindToPose(trenchAwarePose(target), getPathConstraints(), 0.0),
        Set.of(drive));
  }

  /**
   * Build a command that pathfinds to a standoff point near the target, then drives in a perfectly
   * straight line toward the alliance wall to reach the final target. This ensures the climb hook
   * approaches the tower dead-straight in the alliance direction for a clean latch.
   *
   * <p>The standoff point is {@link Constants.AutoConstants#CLIMB_APPROACH_DISTANCE_M} meters in
   * front of the target (opposite to the target's heading). The robot pathfinds there, then uses a
   * 2910-style PID drive-to-pose controller to approach the target with feedback control:
   *
   * <ul>
   *   <li>PID on linear distance computes a scalar velocity toward the target
   *   <li>Static friction feedforward overcomes drivetrain friction at small distances
   *   <li>Heading PID holds the target rotation throughout the approach
   *   <li>Velocity is clamped to a max approach speed for controlled docking
   * </ul>
   *
   * @param target The final climb pose (alliance-corrected, facing the alliance wall)
   * @return A command sequence: pathfind to standoff → PID drive-to-pose → stop
   */
  private Command pathfindThenDriveStraight(Pose2d target) {
    // Compute the standoff pose: offset away from the wall by CLIMB_APPROACH_DISTANCE_M
    // For a 180° target (facing -X), the standoff is in the +X direction (into the field)
    double approachDist = Constants.AutoConstants.CLIMB_APPROACH_DISTANCE_M;
    Translation2d standoffOffset =
        new Translation2d(approachDist, target.getRotation().plus(Rotation2d.k180deg));
    Pose2d standoffPose =
        new Pose2d(target.getTranslation().plus(standoffOffset), target.getRotation());

    double maxVel = Constants.AutoConstants.CLIMB_APPROACH_MAX_VELOCITY_MPS;
    double tolerance = Constants.AutoConstants.CLIMB_APPROACH_TOLERANCE_M;
    double thetaTolerance = Constants.AutoConstants.CLIMB_APPROACH_THETA_TOLERANCE_RAD;

    return Commands.sequence(
        // Step 1: Pathfind to the standoff point (stop there)
        Commands.defer(
            () -> AutoBuilder.pathfindToPose(standoffPose, getPathConstraints(), 0.0),
            Set.of(drive)),
        // Step 2: PID drive-to-pose (2910-style)
        buildDriveToPose(target, maxVel, tolerance, thetaTolerance),
        // Step 3: Stop
        Commands.runOnce(() -> drive.stop(), drive));
  }

  /**
   * Build a 2910-style PID drive-to-pose command. Uses a PID controller on linear distance for
   * translation velocity and a separate PID for heading hold. The translation PID output is
   * projected along the direction-of-travel vector to produce field-relative XY velocities.
   *
   * <p>Inspired by FRC Team 2910's 2025 DRIVE_TO_POINT implementation:
   *
   * <ul>
   *   <li>Linear PID (kP=3.0, kD=0.1) drives distance error to zero
   *   <li>Static friction FF (0.02 × maxVel) added when > 0.5″ away to overcome friction
   *   <li>Heading PID (kP=5.0) with continuous input holds the target angle
   *   <li>Velocity output is clamped to maxVelocity for controlled approach
   * </ul>
   *
   * @param target The desired target pose
   * @param maxVelocity Maximum translation velocity (m/s) for the approach
   * @param driveTolerance Position tolerance (m) to end the command
   * @param thetaTolerance Heading tolerance (rad) to end the command
   * @return A command that drives to the pose and ends when within tolerance
   */
  private Command buildDriveToPose(
      Pose2d target, double maxVelocity, double driveTolerance, double thetaTolerance) {
    // Create PID controllers fresh for each command instance
    PIDController driveController =
        new PIDController(
            Constants.AutoConstants.DRIVE_TO_POSE_KP, 0, Constants.AutoConstants.DRIVE_TO_POSE_KD);
    PIDController thetaController =
        new PIDController(Constants.AutoConstants.DRIVE_TO_POSE_THETA_KP, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    double frictionFF = Constants.AutoConstants.DRIVE_TO_POSE_FRICTION_FF * maxVelocity;

    return Commands.run(
            () -> {
              Pose2d currentPose = drive.getPose();

              // Translation: compute direction and distance to target
              Translation2d translationToTarget =
                  target.getTranslation().minus(currentPose.getTranslation());
              double linearDistance = translationToTarget.getNorm();
              Rotation2d directionOfTravel = translationToTarget.getAngle();

              // Static friction feedforward: only add when > 0.5 inches away
              double friction = (linearDistance >= Units.inchesToMeters(0.5)) ? frictionFF : 0.0;

              // PID on distance → velocity magnitude, clamped to max
              double velocityOutput =
                  Math.min(
                      Math.abs(driveController.calculate(linearDistance, 0.0)) + friction,
                      maxVelocity);

              // Project scalar velocity along direction of travel
              double vx = velocityOutput * directionOfTravel.getCos();
              double vy = velocityOutput * directionOfTravel.getSin();

              // Heading PID
              double omega =
                  thetaController.calculate(
                      currentPose.getRotation().getRadians(), target.getRotation().getRadians());

              drive.driveFieldRelative(vx, vy, omega);

              // Logging
              Logger.recordOutput("Auto/DriveToPose/DistanceToTarget", linearDistance);
              Logger.recordOutput("Auto/DriveToPose/VelocityOutput", velocityOutput);
              Logger.recordOutput(
                  "Auto/DriveToPose/HeadingError",
                  Math.abs(
                      MathUtil.angleModulus(
                          currentPose.getRotation().getRadians()
                              - target.getRotation().getRadians())));
              Logger.recordOutput("Auto/DriveToPose/TargetPose", target);
            },
            drive)
        .until(
            () -> {
              Pose2d currentPose = drive.getPose();
              double distError = currentPose.getTranslation().getDistance(target.getTranslation());
              double headingError =
                  Math.abs(
                      MathUtil.angleModulus(
                          currentPose.getRotation().getRadians()
                              - target.getRotation().getRadians()));
              return distError <= driveTolerance && headingError <= thetaTolerance;
            });
  }

  /**
   * If the target pose is near a TRENCH, snap its heading to the nearest cardinal direction
   * (0°/90°/180°/270°). Otherwise, return the pose unchanged.
   *
   * <p>This uses the approach buffer defined in {@link FieldConstants#TRENCH_APPROACH_BUFFER} so
   * the robot is already oriented correctly before it enters the trench.
   *
   * @param pose The original target pose
   * @return The pose with heading snapped to cardinal if near a trench, otherwise unchanged
   */
  private static Pose2d trenchAwarePose(Pose2d pose) {
    if (FieldConstants.isNearTrench(pose.getTranslation())) {
      Rotation2d snapped = FieldConstants.snapToCardinal(pose.getRotation());
      return new Pose2d(pose.getTranslation(), snapped);
    }
    return pose;
  }

  /**
   * Check if the robot is currently outside the aiming zone (in the neutral zone or beyond). Uses a
   * simple X-coordinate threshold — no complex zone enum needed.
   *
   * <p>The aiming zone includes both the ALLIANCE_ZONE and the HUB_ZONE (up to NEUTRAL_ZONE.minX =
   * 5.50m). The turret starts aiming as soon as the robot leaves the neutral zone.
   *
   * <p>The robot pose from odometry is alliance-relative (seeded via alliance-flipped StartPose).
   * We convert to blue-origin for the check.
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
   * Shared zone-aware feeding logic used by {@link #zoneAwareIntake()}, {@link
   * #buildShootWhileDrivingCore}, and {@link #buildDriveAndClimb}. Updates the {@link
   * #zoneAwareFeeding} hysteresis latch and applies the appropriate superstructure state.
   *
   * <p>When outside the aiming zone, applies {@code outsideState} and resets the latch. Inside the
   * aiming zone, uses acceleration + hood readiness with hysteresis to decide between {@code
   * feedingState} and {@code aimingState}.
   *
   * @param feedingState State to apply when feeding (low accel + hood ready)
   * @param aimingState State to apply when aiming but not feeding
   * @param outsideState State to apply when outside the aiming zone
   */
  private void updateZoneAwareFeeding(
      Superstructure.SuperstructureState feedingState,
      Superstructure.SuperstructureState aimingState,
      Superstructure.SuperstructureState outsideState) {
    if (isOutsideAimingZone()) {
      zoneAwareFeeding = false;
      superstructure.forceWantedState(outsideState);
    } else {
      boolean hoodReady = superstructure.isHoodAtSetpoint();
      double accel = drive.getFilteredAcceleration();
      if (zoneAwareFeeding) {
        if (accel > MAX_FEED_ACCELERATION + FEED_ACCEL_HYSTERESIS || !hoodReady) {
          zoneAwareFeeding = false;
        }
      } else {
        if (accel <= MAX_FEED_ACCELERATION && hoodReady) {
          zoneAwareFeeding = true;
        }
      }
      superstructure.forceWantedState(zoneAwareFeeding ? feedingState : aimingState);
    }
  }

  /**
   * Return an <b>instant</b> command that sets the superstructure to the correct default state
   * based on the robot's current field zone. In the aiming zone (alliance + HUB), the default is
   * AIMING_WHILE_INTAKING so the turret always tracks the HUB. In the neutral/opponent zone, the
   * default is ONLY_INTAKE (turret stowed).
   *
   * <p>Use this instead of {@code superstructure.onlyIntake()} whenever the auto command needs a
   * "reset to default" after a scoring or intake action. This ensures the robot is always aiming
   * when it is in the aiming zone.
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
   * Build a zone-aware intake command that continuously manages the superstructure state based on
   * the robot's field zone, acceleration, and hood readiness:
   *
   * <ul>
   *   <li><b>Outside aiming zone (neutral/opponent):</b> ONLY_INTAKE — turret stowed, just collect
   *       FUEL.
   *   <li><b>In aiming zone, hood not at setpoint:</b> AIMING_WHILE_INTAKING — turret tracks HUB
   *       and hood deploys, but don't feed yet (hood still moving after leaving trench).
   *   <li><b>In aiming zone, hood at setpoint, high acceleration:</b> AIMING_WHILE_INTAKING —
   *       turret tracks HUB, but don't feed (shots inaccurate while accelerating).
   *   <li><b>In aiming zone, hood at setpoint, low acceleration:</b> SHOOTING_WHILE_INTAKING —
   *       turret tracks HUB AND conveyor/indexer feed FUEL. Opportunistic scoring whenever the
   *       robot is cruising near the HUB with hood settled.
   * </ul>
   *
   * <p>This turns the robot into a continuous opportunistic shooter during auto — any time it's in
   * the aiming zone, the hood is settled, and it's not accelerating hard, it dumps FUEL into the
   * HUB.
   *
   * <p>This is auto-specific behavior — in teleop the driver controls when to shoot.
   *
   * <p>This command runs forever (use as a parallel with a deadline like pathfindTo).
   */
  private Command zoneAwareIntake() {
    return Commands.run(
            () -> {
              updateZoneAwareFeeding(
                  Superstructure.SuperstructureState.SHOOTING_WHILE_INTAKING,
                  Superstructure.SuperstructureState.AIMING_WHILE_INTAKING,
                  Superstructure.SuperstructureState.ONLY_INTAKE);

              Logger.recordOutput(
                  "DashboardAuto/ZoneAware/OutsideAimingZone", isOutsideAimingZone());
              Logger.recordOutput("DashboardAuto/ZoneAware/Feeding", zoneAwareFeeding);
            },
            superstructure) // Require superstructure — prevents other commands from overwriting
        .withName("ZoneAwareIntake");
  }
}
