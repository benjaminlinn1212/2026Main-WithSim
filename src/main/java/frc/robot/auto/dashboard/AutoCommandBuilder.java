// Copyright (c) 2026 FRC Team 0 (Amped)
// Dashboard-driven autonomous system — Command builder

package frc.robot.auto.dashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.dashboard.FieldConstants.ClimbLevel;
import frc.robot.auto.dashboard.FieldConstants.ScoringWaypoint;
import frc.robot.subsystems.Superstructure;
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
 * checks {@link Timer#getMatchTime()} at runtime before starting each intake→score cycle. If the
 * remaining match time is insufficient for the cycle plus any climb reserve, it skips remaining
 * cycles and proceeds directly to climb.
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

  // ===== Shoot-While-Driving Tuning =====

  /**
   * Maximum linear acceleration (m/s²) at which the robot is allowed to start feeding FUEL to the
   * shooter. Shooting at high acceleration degrades FUEL accuracy, so we wait until the robot has
   * decelerated (or is at cruise) before commanding the conveyor/indexer.
   */
  private static final double MAX_FEED_ACCELERATION = 3.5;

  /**
   * Hysteresis band for the acceleration gate. Once feeding starts (accel ≤ MAX_FEED_ACCELERATION),
   * it continues until accel exceeds MAX_FEED_ACCELERATION + this value. Prevents stutter when the
   * acceleration oscillates near the threshold.
   */
  private static final double FEED_ACCEL_HYSTERESIS = 1.0;

  /**
   * Time (seconds) to wait for turret/hood/shooter to settle after transitioning to ONLY_SHOOTING
   * before the conveyor actually starts feeding FUEL. In shoot-while-driving mode, this happens
   * while the robot is already moving toward the next intake.
   */
  private static final double SWD_AIM_SETTLE_TIME = 0.3;

  /**
   * Time (seconds) to keep feeding FUEL after starting to shoot. Must be long enough for all loaded
   * FUEL to exit. In shoot-while-driving mode this overlaps with the path.
   */
  private static final double SWD_FEED_TIME = 0.4;

  private final DriveSwerveDrivetrain drive;
  private final Superstructure superstructure;

  /**
   * Hysteresis latch for the zone-aware acceleration gate. When true, the robot is currently
   * feeding (SHOOTING_WHILE_INTAKING). It stays true until acceleration exceeds
   * MAX_FEED_ACCELERATION + FEED_ACCEL_HYSTERESIS. When false, it stays false until acceleration
   * drops below MAX_FEED_ACCELERATION. Prevents state stutter when accel oscillates near threshold.
   */
  private boolean zoneAwareFeeding = false;

  public AutoCommandBuilder(DriveSwerveDrivetrain drive, Superstructure superstructure) {
    this.drive = drive;
    this.superstructure = superstructure;
  }

  /**
   * Build a single Command that executes the entire auto plan with <b>runtime time checks</b>.
   *
   * <p>The planner generates a priority-ordered list of actions, but this builder doesn't blindly
   * execute all of them. Instead, it splits actions into phases:
   *
   * <ol>
   *   <li><b>Pre-cycle</b>: SetStartPose, ScorePreload — always executed.
   *   <li><b>Cycles</b>: IntakeAt + ScoreAt pairs — each gated by a runtime check of
   *       Timer.getMatchTime(). Before starting a cycle, the robot estimates whether it has enough
   *       time to complete the cycle AND still reach the TOWER for climbing.
   *   <li><b>Post-cycle</b>: DriveTo (TOWER) + Climb — executed when cycles end (either all
   *       completed or time ran out).
   * </ol>
   *
   * @param actions Ordered list of actions from the planner
   * @param settings The auto settings (for climb config and time margins)
   * @return A command that executes the full sequence with runtime time management
   */
  public Command buildAutoCommand(List<AutoAction> actions, AutoSettings settings) {
    // Reset mutable state from any previous auto run
    zoneAwareFeeding = false;

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

    // ===== Phase 2: Cycles with runtime time gates =====
    // Each cycle is an IntakeAt + ScoreAt pair. Before starting each pair,
    // check the match timer to see if there's enough time.
    //
    // Estimate needed time for one cycle:
    //   - CYCLE_TIME_ESTIMATE = typical intake-to-score cycle duration
    //   - climbReserve = drive-to-tower (distance-based) + climb duration + safety margin
    //
    // If Timer.getMatchTime() < CYCLE_TIME_ESTIMATE + climbReserve → skip to climb.
    //
    // We pair IntakeAt+ScoreAt together. If there's an odd trailing action, still gate it.

    final AutoSettings settingsRef = settings;

    // Check if post-cycle starts with DriveTo(TOWER) + Climb — used to optimize the last cycle
    final boolean climbFollows =
        postCycleActions.size() >= 2
            && postCycleActions.get(0).getType() == AutoAction.Type.DRIVE_TO
            && postCycleActions.get(1).getType() == AutoAction.Type.CLIMB;
    final Pose2d climbTargetForLastCycle =
        climbFollows ? postCycleActions.get(0).getTargetPose() : null;

    for (int i = 0; i < cycleActions.size(); i += 2) {
      AutoAction intakeAction = cycleActions.get(i);
      AutoAction scoreAction = (i + 1 < cycleActions.size()) ? cycleActions.get(i + 1) : null;
      final boolean isLastCycle = (i + 2 >= cycleActions.size());

      final int cycleNum = (i / 2) + 1;

      // Runtime gate: check if we have enough time for this cycle + climb
      // Use Commands.either: if time remains → run cycle, else → log skip and break
      Command cycleCommand;
      if (scoreAction != null) {
        // If this is the last cycle and climb follows, replace the score action with
        // SWD directly to the tower. The robot shoots mid-transit on the way to climb
        // instead of stopping at the HUB, saving ~1-2s.
        Command scoreCmd;
        if (isLastCycle && climbTargetForLastCycle != null) {
          scoreCmd = buildShootWhileDrivingToClimb(climbTargetForLastCycle);
        } else {
          scoreCmd = buildActionCommand(scoreAction);
        }
        cycleCommand =
            Commands.sequence(
                logStep(intakeAction),
                buildActionCommand(intakeAction),
                logStep(scoreAction),
                scoreCmd);
      } else {
        cycleCommand = Commands.sequence(logStep(intakeAction), buildActionCommand(intakeAction));
      }

      // Wrap the cycle in a runtime time check
      sequence.addCommands(
          Commands.either(
              // Time remains → execute the cycle
              Commands.sequence(
                  Commands.runOnce(
                      () -> {
                        double reserve = estimateClimbReserve(drive.getPose(), settingsRef);
                        Logger.recordOutput(
                            "DashboardAuto/CycleTimeCheck",
                            String.format(
                                "Cycle %d: %.1fs remaining, need ~%.1fs cycle + %.1fs climb reserve",
                                cycleNum, Timer.getMatchTime(), estimateCycleTime(), reserve));
                      }),
                  cycleCommand),
              // Not enough time → log and the rest of the cycles will be skipped
              Commands.runOnce(
                  () -> {
                    double reserve = estimateClimbReserve(drive.getPose(), settingsRef);
                    Logger.recordOutput(
                        "DashboardAuto/CycleTimeCheck",
                        String.format(
                            "Cycle %d SKIPPED: only %.1fs remaining (need %.1fs)",
                            cycleNum, Timer.getMatchTime(), estimateCycleTime() + reserve));
                    Logger.recordOutput("DashboardAuto/SkippedToClimb", true);
                  }),
              // Condition: enough time for one more cycle + climb
              () -> hasTimeForCycle(estimateClimbReserve(drive.getPose(), settingsRef))));
    }

    // ===== Phase 3: Post-cycle (DriveTo TOWER + Climb) =====
    // When the last cycle completed normally, its SCORE_AT was already replaced with SWD
    // directly to the tower (the robot is already at the climb pose). In that case the
    // DriveTo action here is a near-zero-distance pathfind that finishes instantly.
    //
    // When cycles were SKIPPED at runtime (time gate), the robot may still be far from the
    // tower. The SWD here ensures it shoots any loaded FUEL on the way back.
    //
    // Either way, always wrap DriveTo+Climb with SWD — it's harmless if there's nothing
    // to shoot, and critical when there is.
    for (int i = 0; i < postCycleActions.size(); i++) {
      AutoAction action = postCycleActions.get(i);
      sequence.addCommands(logStep(action));

      // If this is a DriveTo followed by a Climb, always shoot on the way
      if (action.getType() == AutoAction.Type.DRIVE_TO
          && i + 1 < postCycleActions.size()
          && postCycleActions.get(i + 1).getType() == AutoAction.Type.CLIMB) {
        Pose2d climbTarget = action.getTargetPose();
        sequence.addCommands(buildShootWhileDrivingToClimb(climbTarget));
      } else {
        Command cmd = buildActionCommand(action);
        if (cmd != null) sequence.addCommands(cmd);
      }
    }

    sequence.addCommands(
        Commands.runOnce(() -> Logger.recordOutput("DashboardAuto/CurrentStep", "COMPLETE")));
    // Stop intake at the end of auto
    sequence.addCommands(superstructure.idle());
    sequence.addCommands(Commands.print("[DashboardAuto] Auto sequence complete!"));

    sequence.setName("DashboardAuto");
    return sequence;
  }

  // ===== Runtime Time Management =====

  /**
   * Check at runtime whether there's enough match time for another intake→score cycle plus climb.
   *
   * <p>If {@link Timer#getMatchTime()} returns a negative value (no FMS connection, practice mode,
   * or simulation without DS), we conservatively assume there IS enough time so the robot runs all
   * planned cycles (matching the old pre-timed behavior).
   *
   * @param climbReserve Estimated seconds needed for climb (0 if no climb planned)
   * @return true if there's enough time for another cycle
   */
  private boolean hasTimeForCycle(double climbReserve) {
    double matchTimeRemaining = Timer.getMatchTime();

    // Timer.getMatchTime() returns -1.0 when not connected to FMS / no match data.
    // In that case, run all planned cycles (fall back to planner's estimates).
    if (matchTimeRemaining < 0) {
      Logger.recordOutput("DashboardAuto/MatchTimeRemaining", -1.0);
      Logger.recordOutput("DashboardAuto/CycleEstimate", 0.0);
      Logger.recordOutput("DashboardAuto/TimeNeeded", 0.0);
      return true;
    }

    double cycleEstimate = estimateCycleTime();

    // Need time for: this cycle + climb reserve + small safety margin
    double needed = cycleEstimate + climbReserve + 0.5;

    Logger.recordOutput("DashboardAuto/MatchTimeRemaining", matchTimeRemaining);
    Logger.recordOutput("DashboardAuto/CycleEstimate", cycleEstimate);
    Logger.recordOutput("DashboardAuto/TimeNeeded", needed);

    return matchTimeRemaining > needed;
  }

  /**
   * Rough estimate of one intake→score cycle duration. Uses a conservative fixed estimate since we
   * don't know the exact path AD* will generate.
   *
   * <p>Breakdown: ~3s drive to intake + ~3s drive to score + 0.5s score ≈ 6.5s typical cycle.
   */
  private static double estimateCycleTime() {
    return 6.5; // seconds
  }

  /**
   * Estimate the time needed after cycles end to drive to the TOWER and climb. Uses the robot's
   * current pose to compute actual drive distance to the climb position.
   *
   * @param currentPose The robot's current pose (for distance-based drive time)
   * @param settings The auto settings with climb configuration
   * @return Estimated seconds for climb (drive + climb + safety), or 0 if no climb planned
   */
  private static double estimateClimbReserve(Pose2d currentPose, AutoSettings settings) {
    if (!settings.shouldAttemptClimb()) {
      return 0.0;
    }
    ClimbLevel cl = settings.getClimbLevel();
    double driveToTower =
        FieldConstants.estimateDriveTime(currentPose, settings.getClimbPose().getPose());
    // Drive time (pose-based) + climb duration + 1s safety margin
    return driveToTower + cl.estimatedClimbDuration + 1.0;
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
      case WAIT:
        return buildWait((AutoAction.Wait) action);
      default:
        return Commands.print("[DashboardAuto] Unknown action type: " + action.getType());
    }
  }

  /** Reset the robot's pose estimate. */
  private Command buildSetStartPose(AutoAction.SetStartPose action) {
    return Commands.runOnce(() -> drive.setPose(action.getPose()), drive).withName("SetStartPose");
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
              // Robot starts in ALLIANCE_ZONE — aim immediately while keeping intake running
              superstructure.aimingWhileIntaking(),
              pathfindTo(target),
              buildStopAndShootSequence(),
              // Stay aiming in alliance zone after scoring
              zoneAwareDefaultState())
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
            buildStopAndShootSequence(),
            // Stay aiming in alliance zone after scoring
            zoneAwareDefaultState())
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
   * Shoot while driving to the TOWER for climbing. The robot dumps any remaining FUEL mid-transit
   * as a last-ditch scoring effort when time is almost up. After the shoot sequence completes, the
   * robot goes idle so the climb command can take over.
   *
   * @param climbTarget The TOWER climb pose
   */
  private Command buildShootWhileDrivingToClimb(Pose2d climbTarget) {
    return buildShootWhileDrivingCore(climbTarget, "TOWER_TRANSIT").withName("SWD_ToTower");
  }

  /**
   * Core shoot-while-driving logic shared by both mid-cycle SWD and climb-transit SWD. The robot
   * drives to {@code destination} at full speed while shooting the HUB mid-transit. The turret
   * tracks in real-time via ShooterSetpoint.
   *
   * <p>Architecture:
   *
   * <ol>
   *   <li>Start driving to {@code destination} (zone-aware intake from prior step)
   *   <li>In parallel: wait until leaving neutral zone, then start AIMING_WHILE_INTAKING
   *   <li>After aim settles → SHOOTING_WHILE_INTAKING (conveyor feeds FUEL while intake stays on)
   *   <li>Feed for SWD_FEED_TIME, then back to zone-aware intake for remainder of path
   *   <li>The path is the deadline — when the drive finishes, everything ends
   * </ol>
   *
   * @param destination Where the robot is actually driving to
   * @param logLabel Label for logging (scoring waypoint name or "TOWER_TRANSIT")
   */
  private Command buildShootWhileDrivingCore(Pose2d destination, String logLabel) {
    return Commands.sequence(
        Commands.print("[DashboardAuto] Shoot-while-driving (" + logLabel + ")"),
        // Drive at full speed to destination while shooting in parallel (zone-gated)
        Commands.deadline(
            // Deadline: the path finishing is what ends this group
            pathfindTo(destination),
            // In parallel: wait for non-neutral zone → aim → settle → low accel → shoot → resume
            Commands.sequence(
                // Wait until robot leaves the neutral/opponent zone before starting to aim
                Commands.waitUntil(() -> !isInNeutralOrOpponentZone()),
                // Start aiming while keeping intake running
                superstructure.aimingWhileIntaking(),
                // Wait for turret/hood/shooter to settle while driving
                Commands.waitSeconds(SWD_AIM_SETTLE_TIME),
                // Wait until robot acceleration is low enough for accurate shots
                Commands.waitUntil(() -> drive.getFilteredAcceleration() <= MAX_FEED_ACCELERATION),
                // Start feeding FUEL while keeping intake running
                superstructure.shootingWhileIntaking(),
                Commands.runOnce(
                    () -> Logger.recordOutput("DashboardAuto/ShootWhileDriving", logLabel)),
                // Feed for long enough to dump all loaded FUEL
                Commands.waitSeconds(SWD_FEED_TIME),
                // Done shooting — return to zone-aware default for remainder of path
                // (AIMING_WHILE_INTAKING in alliance zone, ONLY_INTAKE in neutral zone)
                zoneAwareDefaultState())),
        // Safety: ensure zone-appropriate state after the path finishes
        zoneAwareDefaultState());
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
   */
  private Command buildIntakeAt(AutoAction.IntakeAt action) {
    FieldConstants.IntakeLocation loc = action.getLocation();
    boolean allianceZoneIntake =
        loc.zone == FieldConstants.Zone.ALLIANCE_ZONE
            || loc.zone == FieldConstants.Zone.OUTPOST_AREA;

    if (allianceZoneIntake) {
      // Alliance-zone intake (OUTPOST / DEPOT): use zone-aware intake so the robot
      // opportunistically aims and shoots while driving through the alliance/HUB zone.
      // We use zoneAwareIntake() (continuous polling) rather than buildShootWhileDrivingCore()
      // because these paths are short — the one-shot SWD core's settle+shoot sequence
      // gets killed by the deadline before it completes on short alliance-zone drives.
      return Commands.sequence(
              Commands.print("[DashboardAuto] Intaking at " + loc.name() + " (alliance zone)"),
              Commands.deadline(pathfindTo(loc.getPose()), zoneAwareIntake()),
              // Stay aiming in alliance zone — only go ONLY_INTAKE if in neutral zone
              zoneAwareDefaultState())
          .withName("IntakeAt_" + loc.name());
    } else {
      // Neutral zone intake — zone-aware intake (turret stowed in neutral zone, aims in alliance)
      return Commands.sequence(
              Commands.print("[DashboardAuto] Intaking at " + loc.name()),
              Commands.deadline(pathfindTo(loc.getPose()), zoneAwareIntake()),
              zoneAwareDefaultState())
          .withName("IntakeAt_" + loc.name());
    }
  }

  /** Drive to an arbitrary pose. */
  private Command buildDriveTo(AutoAction.DriveTo action) {
    Pose2d target = action.getTarget();
    return Commands.sequence(
            Commands.print("[DashboardAuto] Driving to " + action.getLabel()), pathfindTo(target))
        .withName("DriveTo_" + action.getLabel());
  }

  /** Execute the TOWER climb sequence. Stops intake before entering climb mode. */
  private Command buildClimb(AutoAction.Climb action) {
    return Commands.sequence(
            Commands.print("[DashboardAuto] Climbing TOWER " + action.getClimbLevel().name()),
            // Stop intake — climb mode requires all mechanisms stowed
            superstructure.idle(),
            superstructure.enterClimbMode())
        .withName("Climb_" + action.getClimbLevel().name());
  }

  /** Wait for a specified duration. */
  private Command buildWait(AutoAction.Wait action) {
    return Commands.sequence(
            Commands.print(
                "[DashboardAuto] Waiting " + action.getSeconds() + "s: " + action.getReason()),
            Commands.waitSeconds(action.getSeconds()))
        .withName("Wait_" + action.getReason());
  }

  // ===== Shared Helpers =====

  /**
   * Build the stop-and-shoot sequence: aim (while intaking) → wait for low acceleration → settle →
   * shoot → resume intake.
   *
   * <p>With continuous intake, we use AIMING_WHILE_INTAKING and SHOOTING_WHILE_INTAKING states so
   * the intake never stops. The acceleration wait ensures FUEL is only fed to the shooter after the
   * robot has decelerated below {@link #MAX_FEED_ACCELERATION}.
   *
   * @return Command sequence to fire FUEL at the HUB while stationary
   */
  private Command buildStopAndShootSequence() {
    return Commands.sequence(
        // Aim at hub while keeping intake running
        superstructure.aimingWhileIntaking(),
        // Wait until robot has decelerated enough for accurate shots
        Commands.waitUntil(() -> drive.getFilteredAcceleration() <= MAX_FEED_ACCELERATION),
        // Let turret/hood/shooter settle
        Commands.waitSeconds(0.2),
        // Fire while keeping intake running
        superstructure.shootingWhileIntaking(),
        // Wait for FUEL to leave
        Commands.waitSeconds(0.15),
        // Return to zone-aware default (aim in alliance zone, intake-only in neutral)
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
   * X-coordinate threshold (blue-origin meters) separating alliance territory from
   * neutral/opponent. Everything with blue-origin X ≤ this value is "our side" (alliance zone + HUB
   * zone). Above this is neutral zone or opponent side.
   *
   * <p>Aligned with {@link FieldConstants#TRENCH_MAX_X} so that the zone-aware intake does not
   * attempt to aim inside the trench approach buffer (the Superstructure would stow the turret
   * there anyway, causing stutter).
   */
  private static final double ALLIANCE_SIDE_MAX_X = FieldConstants.TRENCH_MAX_X; // 5.2m

  /**
   * Check if the robot is currently on the neutral/opponent side of the field. Uses a simple
   * X-coordinate threshold — no complex zone enum needed.
   *
   * <p>The robot pose from odometry is alliance-relative (seeded via alliance-flipped StartPose).
   * We convert to blue-origin for the check.
   */
  private boolean isInNeutralOrOpponentZone() {
    var translation = drive.getPose().getTranslation();
    // Convert alliance coords to blue-origin for the X check
    double blueX =
        FieldConstants.isRedAlliance()
            ? FieldConstants.FIELD_LENGTH - translation.getX()
            : translation.getX();
    return blueX > ALLIANCE_SIDE_MAX_X;
  }

  /**
   * Return an <b>instant</b> command that sets the superstructure to the correct default state
   * based on the robot's current field zone. In the alliance/HUB zone, the default is
   * AIMING_WHILE_INTAKING so the turret always tracks the HUB. In the neutral/opponent zone, the
   * default is ONLY_INTAKE (turret stowed).
   *
   * <p>Use this instead of {@code superstructure.onlyIntake()} whenever the auto command needs a
   * "reset to default" after a scoring or intake action. This ensures the robot is always aiming
   * when it is on friendly turf — the user's requirement is that aiming only stops when the
   * drivetrain physically reaches the climb pose.
   */
  private Command zoneAwareDefaultState() {
    return Commands.runOnce(
            () -> {
              if (isInNeutralOrOpponentZone()) {
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
   * the robot's field zone and acceleration:
   *
   * <ul>
   *   <li><b>Neutral zone / opponent side:</b> ONLY_INTAKE — turret stowed, just collect FUEL.
   *   <li><b>Alliance / HUB zone, high acceleration:</b> AIMING_WHILE_INTAKING — turret tracks HUB,
   *       but don't feed yet (shots inaccurate while accelerating).
   *   <li><b>Alliance / HUB zone, low acceleration:</b> SHOOTING_WHILE_INTAKING — turret tracks HUB
   *       AND conveyor/indexer feed FUEL. Opportunistic scoring whenever the robot is cruising near
   *       the HUB.
   * </ul>
   *
   * <p>This turns the robot into a continuous opportunistic shooter during auto — any time it's in
   * friendly territory and not accelerating hard, it dumps FUEL into the HUB. No need for explicit
   * "stop and shoot" detours when driving between alliance-zone locations (e.g. depot → outpost).
   *
   * <p>This is auto-specific behavior — in teleop the driver controls when to shoot.
   *
   * <p>This command runs forever (use as a parallel with a deadline like pathfindTo).
   */
  private Command zoneAwareIntake() {
    return Commands.run(
            () -> {
              boolean inNeutralOrOpponent = isInNeutralOrOpponentZone();

              if (inNeutralOrOpponent) {
                // Far from HUB — just intake, turret stowed
                zoneAwareFeeding = false;
                superstructure.forceWantedState(Superstructure.SuperstructureState.ONLY_INTAKE);
              } else {
                // In alliance/HUB zone — decide between aiming and shooting
                double accel = drive.getFilteredAcceleration();

                // Hysteresis: once feeding starts, keep feeding until accel exceeds the
                // upper threshold. Once stopped, don't restart until accel drops below
                // the lower threshold. Prevents stutter at the boundary.
                if (zoneAwareFeeding) {
                  if (accel > MAX_FEED_ACCELERATION + FEED_ACCEL_HYSTERESIS) {
                    zoneAwareFeeding = false;
                  }
                } else {
                  if (accel <= MAX_FEED_ACCELERATION) {
                    zoneAwareFeeding = true;
                  }
                }

                if (zoneAwareFeeding) {
                  superstructure.forceWantedState(
                      Superstructure.SuperstructureState.SHOOTING_WHILE_INTAKING);
                } else {
                  superstructure.forceWantedState(
                      Superstructure.SuperstructureState.AIMING_WHILE_INTAKING);
                }
              }

              // Logging
              Logger.recordOutput(
                  "DashboardAuto/ZoneAware/InNeutralOrOpponent", inNeutralOrOpponent);
              Logger.recordOutput("DashboardAuto/ZoneAware/Feeding", zoneAwareFeeding);
            },
            superstructure) // Require superstructure — prevents other commands from overwriting
        .withName("ZoneAwareIntake");
  }
}
