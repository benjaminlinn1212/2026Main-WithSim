// Copyright (c) 2026 FRC Team 0 (Amped)
// Dashboard-driven autonomous system — The planning brain for REBUILT

package frc.robot.auto.dashboard;

import static frc.robot.auto.dashboard.AutoTuning.*;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto.dashboard.FieldConstants.ClimbLevel;
import frc.robot.auto.dashboard.FieldConstants.ClimbPose;
import frc.robot.auto.dashboard.FieldConstants.IntakeLocation;
import frc.robot.auto.dashboard.FieldConstants.ScoringWaypoint;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * The autonomous planning engine for REBUILT. Given an {@link AutoSettings} configuration, produces
 * an ordered list of {@link AutoAction}s that respects all constraints and fits within the
 * 20-second AUTO period.
 *
 * <p>REBUILT AUTO planning strategy (inspired by 254/6328):
 *
 * <ol>
 *   <li>Reset pose to known start position behind the ROBOT STARTING LINE.
 *   <li>Drive to best HUB shooting position and score preloaded FUEL.
 *   <li>For each intake in the sequence string: collect FUEL from OUTPOST/DEPOT/NEUTRAL ZONE →
 *       drive to HUB shooting position → score FUEL. The number of cycles matches the sequence
 *       length exactly — no wrapping, no extra cycles added.
 *   <li>Time-budget every action; stop adding cycles when time runs out.
 *   <li>Optionally drive to TOWER and climb to selected RUNG level if configured and time remains.
 *   <li>Note: LEVEL 1 TOWER climb is worth 15 pts during AUTO (vs 10 pts teleop).
 * </ol>
 *
 * <p><b>Scoring strategy is zone-based (not a dashboard toggle):</b>
 *
 * <ul>
 *   <li><b>D/O (DEPOT / OUTPOST) intakes:</b> Always shoot-while-driving (SWD). The robot naturally
 *       passes through the alliance/HUB zone on the way to the next intake, so it dumps FUEL
 *       mid-transit without stopping. The turret tracks the HUB in real-time.
 *   <li><b>U/L (NEUTRAL_ZONE_UPPER / NEUTRAL_ZONE_LOWER) intakes:</b> Always stop-and-shoot. The
 *       robot must drive back from the neutral zone to a scoring waypoint to score, since it cannot
 *       shoot from the neutral zone. After scoring, it drives back out to the next intake.
 * </ul>
 *
 * <p><b>Runtime time management:</b> The planner generates a generous plan (as many cycles as
 * plausibly fit). The {@link AutoCommandBuilder} makes the actual time decisions at runtime using
 * an FPGA-based countdown timer — at each intake pose, it checks whether there's enough time to
 * score and continue, or whether to abort to climb.
 *
 * <p>The planner is purely deterministic and stateless — it produces the same plan for the same
 * settings. It runs in microseconds so it can be called every disabled periodic cycle.
 */
public class AutoPlanner {

  private AutoPlanner() {}

  /**
   * Generate a complete autonomous action plan from the given settings.
   *
   * @param settings Dashboard-configured settings
   * @return Ordered list of actions for the auto period
   */
  public static List<AutoAction> plan(AutoSettings settings) {
    List<AutoAction> actions = new ArrayList<>();
    double timeRemaining = FieldConstants.AUTO_DURATION;
    double timeMargin = settings.getTimeMarginMultiplier();

    // Current pose tracker (for distance/time estimation)
    Pose2d currentPose = settings.getStartPose().getPose();

    // ===== Step 1: Set start pose =====
    actions.add(new AutoAction.SetStartPose(currentPose));

    // ===== Step 2: Score preloaded FUEL =====
    // All preloaded FUEL is dumped in a single scoring action.
    // SWD preload when first intake is D/O (robot passes through alliance zone on the way).
    // Stop-and-shoot preload when first intake is U/L (robot heads to neutral zone immediately).
    // When Score Preload is disabled on the dashboard, skip this step entirely.
    if (settings.isScorePreload()) {
      ScoringWaypoint preloadTarget =
          pickBestScoringWaypoint(currentPose, settings.getScoringPriority());

      if (preloadTarget != null) {
        double preloadScoreTime = STOP_AND_SHOOT_DURATION * timeMargin;
        boolean preloadAdded = false;

        // SWD preload: peek at the first intake location and drive there while shooting.
        // Only works if the first intake is D/O (alliance zone) — the robot passes through
        // the HUB zone on the way, so it can dump FUEL mid-transit.
        // If the first intake is U/L (neutral zone), the robot drives away from the HUB
        // immediately — fall through to stop-and-shoot below.
        IntakeLocation firstIntake =
            pickBestIntakeLocation(settings.getIntakePriority(), new HashSet<>(), 0);
        if (firstIntake != null && firstIntake.zone != FieldConstants.Zone.NEUTRAL_ZONE) {
          double driveTime =
              FieldConstants.estimateDriveTime(currentPose, firstIntake.getPose()) * timeMargin;
          if (driveTime + preloadScoreTime < timeRemaining) {
            actions.add(new AutoAction.ScorePreload(preloadTarget, true, firstIntake.getPose()));
            timeRemaining -= driveTime + preloadScoreTime;
            currentPose = firstIntake.getPose();
            preloadAdded = true;
          }
        }

        if (!preloadAdded) {
          // Stop-and-shoot preload: drive to scoring waypoint, stop, aim, fire.
          // Used when first intake is U/L (neutral zone) or no intake is configured.
          double driveTime =
              FieldConstants.estimateDriveTime(currentPose, preloadTarget.toPose()) * timeMargin;
          if (driveTime + preloadScoreTime < timeRemaining) {
            actions.add(new AutoAction.ScorePreload(preloadTarget));
            timeRemaining -= driveTime + preloadScoreTime;
            currentPose = preloadTarget.toPose();
          }
        }
      }
    }

    // ===== Step 3: Cycle FUEL scoring =====
    // Each cycle: collect FUEL → score FUEL.
    //
    // Scoring strategy is ZONE-BASED (not a dashboard toggle):
    //   - D/O (DEPOT/OUTPOST) intakes: Always SWD. The robot passes through the alliance/HUB
    //     zone on the way out to the next intake, dumping FUEL mid-transit.
    //   - U/L (NEUTRAL_ZONE_UPPER/LOWER) intakes: Always stop-and-shoot. The robot must drive
    //     back from the neutral zone to a scoring waypoint, shoot, then drive back out.
    //
    // The planner generates a GENEROUS plan — as many cycles as plausibly fit. The
    // AutoCommandBuilder makes the actual time decisions at runtime using an FPGA-based
    // countdown timer, deciding whether to score+continue or abort to climb.
    //
    // The number of cycles is determined STRICTLY by the intake sequence string length.
    // "D" = 1 cycle, "DUO" = up to 3 cycles, "DUUUUUUUU" = up to 9 cycles.
    // The planner never adds cycles beyond what the user typed. Cycles are trimmed if
    // the time budget runs out.

    // ===== Reserve climb budget =====
    // Before planning cycles, set aside time for climb so cycles can never eat the
    // climb budget. The builder handles the actual climb decision at runtime, but the
    // planner must ensure the plan leaves room. We'll add the climb back in Step 4.
    double climbReserve = 0.0;
    if (settings.shouldAttemptClimb()) {
      ClimbLevel cl = settings.getClimbLevel();
      ClimbPose cp = settings.getClimbPose();
      // Estimate from the start pose (worst case — we'll refine in Step 4)
      double driveToTower =
          FieldConstants.estimateDriveTime(currentPose, cp.getPose()) * timeMargin;
      double climbTime = cl.estimatedClimbDuration * timeMargin;
      climbReserve = driveToTower + climbTime;
      timeRemaining -= climbReserve;
      Logger.recordOutput(
          "AutoPlanner/ClimbReserve",
          String.format(
              "Reserved %.1fs for climb (drive=%.1f, climb=%.1f)",
              climbReserve, driveToTower, climbTime));
    }

    int cyclesCompleted = 0;
    // Track depleted intake locations (OUTPOST/DEPOT are one-shot; NEUTRAL ZONE is reusable)
    Set<IntakeLocation> depletedIntakes = new HashSet<>();
    // Track how many times each reusable intake location has been visited.
    // On repeat visits (visitNumber ≥ 2) to a neutral zone location, the robot drives deeper
    // past the original waypoint because the nearby FUEL was already collected.
    Map<IntakeLocation, Integer> intakeVisitCounts = new HashMap<>();

    while (cyclesCompleted < settings.getIntakePriority().size()) {
      // Pick FUEL intake location — advances through the sequence string each cycle
      IntakeLocation intakeLoc =
          pickBestIntakeLocation(settings.getIntakePriority(), depletedIntakes, cyclesCompleted);

      if (intakeLoc == null) {
        Logger.recordOutput("AutoPlanner/StopReason", "No valid FUEL intake locations");
        break;
      }

      // Pick next HUB shooting position based on intake pose, not current pose.
      // This prevents stop-and-shoot cycles from always re-picking the same waypoint
      // (e.g. after shooting at HUB_CENTER, currentPose = HUB_CENTER, so distance = 0
      // and it would always pick HUB_CENTER again). Using the intake pose means we
      // pick the scoring waypoint closest to where the robot will be returning from.
      ScoringWaypoint nextTarget =
          pickBestScoringWaypoint(intakeLoc.getPose(), settings.getScoringPriority());

      if (nextTarget == null) {
        Logger.recordOutput("AutoPlanner/StopReason", "No valid HUB shooting positions");
        break;
      }

      // Estimate cycle time — NO timeMargin applied here because the planner is
      // deliberately generous. The AutoCommandBuilder handles real-time time decisions
      // at each intake pose using FPGA timestamps. We use raw estimates so we plan
      // as many cycles as could plausibly fit.
      double driveToIntakeTime = FieldConstants.estimateDriveTime(currentPose, intakeLoc.getPose());

      // Determine scoring strategy: SWD vs stop-and-shoot.
      // Peek ahead at the next intake to decide — U/L intakes can SWD if the next
      // destination is in the alliance zone (robot drives back through HUB zone).
      boolean isNeutralZoneIntake = (intakeLoc.zone == FieldConstants.Zone.NEUTRAL_ZONE);

      IntakeLocation peekNextIntake =
          pickBestIntakeLocation(
              settings.getIntakePriority(), depletedIntakes, cyclesCompleted + 1);
      boolean nextIsAllianceZone =
          peekNextIntake != null && peekNextIntake.zone != FieldConstants.Zone.NEUTRAL_ZONE;

      // SWD when: (a) D/O intake (always), or (b) U/L intake heading to alliance zone next
      boolean willSWD = !isNeutralZoneIntake || nextIsAllianceZone;

      double scoringTime;
      if (!willSWD) {
        // U/L stop-and-shoot: drive intake→score + shoot (raw estimate, no margin)
        double driveToScore =
            FieldConstants.estimateDriveTime(intakeLoc.getPose(), nextTarget.toPose());
        scoringTime = driveToScore + STOP_AND_SHOOT_DURATION;
      } else {
        // SWD: scoring happens mid-transit (raw estimate, no margin)
        scoringTime = SWD_SCORE_DURATION;
      }

      double cycleTime = driveToIntakeTime + scoringTime;

      // The planner is GENEROUS — it plans as many cycles as can plausibly fit based on
      // raw cycle time alone. Climb reserve is NOT subtracted here because the
      // AutoCommandBuilder handles climb-merge decisions at runtime with real-time data.
      // The planner only stops when the raw cycle time exceeds the remaining budget.
      if (cycleTime > timeRemaining) {
        Logger.recordOutput(
            "AutoPlanner/StopReason",
            String.format(
                "Time budget exceeded: cycle needs %.1fs, have %.1fs", cycleTime, timeRemaining));
        break;
      }

      // Add the intake action — track visit number for reusable (neutral zone) locations.
      // On the 2nd+ visit, the robot drives deeper past the original waypoint because nearby
      // FUEL was already collected on the previous pass.
      int visitNumber = intakeVisitCounts.getOrDefault(intakeLoc, 0) + 1;
      intakeVisitCounts.put(intakeLoc, visitNumber);
      actions.add(new AutoAction.IntakeAt(intakeLoc, visitNumber));
      timeRemaining -= driveToIntakeTime;
      currentPose = intakeLoc.getPose();

      // Mark non-reusable locations as depleted (OUTPOST/DEPOT can only be visited once)
      if (!intakeLoc.reusable) {
        depletedIntakes.add(intakeLoc);
      }

      // Add the score action — SWD vs stop-and-shoot was determined above (willSWD).
      if (!willSWD) {
        // U/L with next intake also U/L (or no next intake): must stop-and-shoot.
        // The robot can't SWD because it would drive away from the HUB to reach
        // the next neutral zone intake.
        actions.add(new AutoAction.ScoreAt(nextTarget));
        timeRemaining -= scoringTime;
        currentPose = nextTarget.toPose();
      } else {
        // SWD: robot passes through HUB zone on the way to the next destination.
        // This covers D/O intakes AND U/L intakes heading to an alliance zone intake.
        if (peekNextIntake != null) {
          actions.add(new AutoAction.ScoreAt(nextTarget, true, peekNextIntake.getPose()));
          timeRemaining -= scoringTime;
          currentPose = peekNextIntake.getPose();
        } else {
          // Last cycle, no next intake — SWD with null destination (fallback to waypoint)
          actions.add(new AutoAction.ScoreAt(nextTarget, true, null));
          timeRemaining -= scoringTime;
          currentPose = nextTarget.toPose();
        }
      }

      cyclesCompleted++;
    }

    // ===== Step 4: TOWER Climb =====
    // Note: Only LEVEL 1 is available during AUTO (15 pts). LEVEL 2/3 are teleop only.
    // But we plan it here so the robot is already in position for teleop climb.
    // Give back the climb reserve we subtracted in Step 3, then re-estimate from the
    // actual post-cycle pose (which may be closer or farther from the tower).
    timeRemaining += climbReserve;

    if (settings.shouldAttemptClimb()) {
      ClimbLevel cl = settings.getClimbLevel();
      ClimbPose cp = settings.getClimbPose();
      double driveToTower =
          FieldConstants.estimateDriveTime(currentPose, cp.getPose()) * timeMargin;
      double climbTime = cl.estimatedClimbDuration * timeMargin;

      if (driveToTower + climbTime <= timeRemaining) {
        actions.add(new AutoAction.DriveTo(cp.getPose(), "TOWER " + cl.name()));
        actions.add(new AutoAction.Climb(cl, cp));
        timeRemaining -= driveToTower + climbTime;
        currentPose = cp.getPose();
      } else {
        // This should rarely happen now that we reserve time in Step 3.
        // If it does, the builder will still attempt climb using its own time checks.
        Logger.recordOutput("AutoPlanner/ClimbSkipped", "Not enough time for TOWER climb");
      }
    }

    // Log plan summary
    Logger.recordOutput("AutoPlanner/TotalActions", actions.size());
    Logger.recordOutput("AutoPlanner/CyclesPlanned", cyclesCompleted);
    Logger.recordOutput("AutoPlanner/TimeRemaining", timeRemaining);
    Logger.recordOutput("AutoPlanner/PlanSummary", summarizePlan(actions));

    return Collections.unmodifiableList(actions);
  }

  // ===== Location Selection Heuristics =====

  /**
   * Pick the closest scoring waypoint from the allowed list.
   *
   * <p>Simply returns whichever waypoint in the dashboard's allowed list is nearest to the given
   * position. No priority weighting, no revisit penalty — just closest distance wins.
   *
   * @param referencePose The pose to measure distance from (typically the intake location)
   * @param allowed The dashboard-configured list of allowed scoring waypoints
   * @return The closest allowed waypoint, or null if the list is empty
   */
  private static ScoringWaypoint pickBestScoringWaypoint(
      Pose2d referencePose, List<ScoringWaypoint> allowed) {

    ScoringWaypoint best = null;
    double bestDistance = Double.MAX_VALUE;

    for (ScoringWaypoint loc : allowed) {
      double distance = referencePose.getTranslation().getDistance(loc.getPosition());
      if (distance < bestDistance) {
        bestDistance = distance;
        best = loc;
      }
    }

    return best;
  }

  /**
   * Pick the next intake location from the sequence list.
   *
   * <p>OUTPOST and DEPOT are one-shot: once visited, they're added to {@code depleted} and excluded
   * from future cycles. NEUTRAL ZONE locations are reusable and never depleted.
   *
   * <p>Returns the intake location at position {@code sequenceIndex} in the sequence. If the index
   * is past the end of the sequence, returns null — the planner should stop adding cycles. If the
   * location at that index is depleted (one-shot intake already used), returns null.
   *
   * @param sequence The intake sequence from dashboard settings
   * @param depleted Set of already-visited one-shot intakes
   * @param sequenceIndex Which cycle we're on (0-based). Directly indexes into the sequence.
   */
  private static IntakeLocation pickBestIntakeLocation(
      List<IntakeLocation> sequence, Set<IntakeLocation> depleted, int sequenceIndex) {

    if (sequence.isEmpty() || sequenceIndex >= sequence.size()) {
      return null; // sequence exhausted — no more cycles
    }

    IntakeLocation loc = sequence.get(sequenceIndex);
    if (depleted.contains(loc)) {
      return null; // one-shot intake already used at this position
    }
    return loc;
  }

  // ===== Utilities =====

  /** Generate a human-readable summary of the plan for dashboard display. */
  public static String summarizePlan(List<AutoAction> actions) {
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < actions.size(); i++) {
      if (i > 0) sb.append(" → ");
      sb.append(actions.get(i).describe());
    }
    return sb.toString();
  }

  /**
   * Estimate the total duration of a plan.
   *
   * @param actions The planned actions
   * @param startPose The starting pose (for computing drive distances)
   * @return Estimated total seconds
   */
  public static double estimateTotalDuration(List<AutoAction> actions, Pose2d startPose) {
    double total = 0.0;
    Pose2d currentPose = startPose;

    for (AutoAction action : actions) {
      // Add drive time to reach the action's target
      Pose2d target = action.getTargetPose();
      if (target != null) {
        total += FieldConstants.estimateDriveTime(currentPose, target);
        currentPose = target;
      }
      // Add the action's own duration
      total += action.estimatedDuration();
    }

    return total;
  }
}
