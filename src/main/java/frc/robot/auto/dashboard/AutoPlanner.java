// Copyright (c) 2026 FRC Team 0 (Amped)
// Dashboard-driven autonomous system — The planning brain for REBUILT

package frc.robot.auto.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto.dashboard.FieldConstants.ClimbLevel;
import frc.robot.auto.dashboard.FieldConstants.ClimbPose;
import frc.robot.auto.dashboard.FieldConstants.IntakeLocation;
import frc.robot.auto.dashboard.FieldConstants.ScoringWaypoint;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
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
 *   <li>For each remaining cycle (up to maxCycles): collect FUEL from OUTPOST/DEPOT/NEUTRAL ZONE →
 *       drive to HUB shooting position → score FUEL.
 *   <li>Time-budget every action; stop adding cycles when time runs out.
 *   <li>Optionally drive to TOWER and climb to selected RUNG level if configured and time remains.
 *   <li>Note: LEVEL 1 TOWER climb is worth 15 pts during AUTO (vs 10 pts teleop).
 * </ol>
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
    // With SWD enabled, the robot drives directly toward the first intake and shoots mid-transit
    // instead of detouring to a HUB scoring waypoint.
    {
      ScoringWaypoint preloadTarget =
          pickBestScoringWaypoint(currentPose, settings.getScoringPriority(), null);

      if (preloadTarget != null) {
        double preloadScoreTime = FieldConstants.SCORE_DURATION * timeMargin;

        if (settings.isShootWhileDriving()) {
          // SWD preload: peek at the first intake location and drive there while shooting
          IntakeLocation firstIntake =
              pickBestIntakeLocation(currentPose, settings.getIntakePriority(), new HashSet<>(), 0);
          if (firstIntake != null) {
            double driveTime =
                FieldConstants.estimateDriveTime(currentPose, firstIntake.getPose()) * timeMargin;
            if (driveTime + preloadScoreTime < timeRemaining) {
              actions.add(new AutoAction.ScorePreload(preloadTarget, true, firstIntake.getPose()));
              timeRemaining -= driveTime + preloadScoreTime;
              currentPose = firstIntake.getPose();
            }
          }
        } else {
          // Stop-and-shoot preload: drive to scoring waypoint, stop, aim, fire
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
    // Each cycle: collect FUEL → drive to HUB → shoot FUEL
    int cyclesCompleted = 0;
    // Track which HUB positions we've already shot from to diversify angles
    List<ScoringWaypoint> scoredLocations = new ArrayList<>();
    // Track depleted intake locations (OUTPOST/DEPOT are one-shot; NEUTRAL ZONE is reusable)
    Set<IntakeLocation> depletedIntakes = new HashSet<>();

    while (cyclesCompleted < settings.getMaxCycles()) {
      // Pick next HUB shooting position
      ScoringWaypoint nextTarget =
          pickBestScoringWaypoint(currentPose, settings.getScoringPriority(), scoredLocations);

      if (nextTarget == null) {
        Logger.recordOutput("AutoPlanner/StopReason", "No valid HUB shooting positions");
        break;
      }

      // Pick FUEL intake location — advances through the sequence string each cycle
      IntakeLocation intakeLoc =
          pickBestIntakeLocation(
              currentPose, settings.getIntakePriority(), depletedIntakes, cyclesCompleted);

      if (intakeLoc == null) {
        Logger.recordOutput("AutoPlanner/StopReason", "No valid FUEL intake locations");
        break;
      }

      // Estimate cycle time: drive-to-intake (full speed) + drive-to-HUB + score
      double driveToIntakeTime =
          FieldConstants.estimateDriveTime(currentPose, intakeLoc.getPose()) * timeMargin;
      double driveToScoreTime =
          FieldConstants.estimateDriveTime(intakeLoc.getPose(), nextTarget.toPose()) * timeMargin;
      double scoreTime = FieldConstants.SCORE_DURATION * timeMargin;

      double cycleTime = driveToIntakeTime + driveToScoreTime + scoreTime;

      // Reserve time for TOWER climb if needed
      double climbReserve = 0.0;
      if (settings.shouldAttemptClimb()) {
        ClimbLevel cl = settings.getClimbLevel();
        ClimbPose cp = settings.getClimbPose();
        double driveToTower =
            FieldConstants.estimateDriveTime(nextTarget.toPose(), cp.getPose()) * timeMargin;
        climbReserve = driveToTower + cl.estimatedClimbDuration * timeMargin;
      }

      if (cycleTime + climbReserve > timeRemaining) {
        Logger.recordOutput(
            "AutoPlanner/StopReason",
            String.format(
                "Time budget exceeded: need %.1fs, have %.1fs",
                cycleTime + climbReserve, timeRemaining));
        break;
      }

      // Add the cycle
      actions.add(new AutoAction.IntakeAt(intakeLoc));
      timeRemaining -= driveToIntakeTime;
      currentPose = intakeLoc.getPose();

      // Mark non-reusable locations as depleted (OUTPOST/DEPOT can only be visited once)
      if (!intakeLoc.reusable) {
        depletedIntakes.add(intakeLoc);
      }

      // Determine if shoot-while-driving is feasible for this cycle.
      // SWD works when the robot naturally passes through the alliance/HUB zone on its way
      // to the next intake (e.g., DEPOT→OUTPOST, OUTPOST→NEUTRAL, NEUTRAL→DEPOT).
      // SWD does NOT work for NEUTRAL→NEUTRAL: the robot stays in the neutral zone and
      // must intentionally drive back to the HUB to score (stop-and-shoot).
      boolean shootWhileMoving = false;
      IntakeLocation nextIntake = null;
      if (settings.isShootWhileDriving()) {
        // Peek ahead: what would the NEXT cycle's intake be?
        nextIntake =
            pickBestIntakeLocation(
                nextTarget.toPose(),
                settings.getIntakePriority(),
                depletedIntakes,
                cyclesCompleted + 1);

        if (intakeLoc.zone == FieldConstants.Zone.NEUTRAL_ZONE) {
          // SWD only if the next intake pulls us back through the alliance zone,
          // or if this is the last cycle (no next intake / next intake is non-neutral)
          shootWhileMoving =
              nextIntake == null || nextIntake.zone != FieldConstants.Zone.NEUTRAL_ZONE;
        } else {
          // Coming from DEPOT/OUTPOST — we always pass through the HUB zone on the way out
          shootWhileMoving = true;
        }
      }

      if (shootWhileMoving && nextIntake != null) {
        // SWD: robot drives directly to the next intake while shooting mid-transit.
        // No detour to a scoring waypoint — turret tracks HUB automatically.
        actions.add(new AutoAction.ScoreAt(nextTarget, true, nextIntake.getPose()));
        // SWD ends at the next intake, not the scoring waypoint
        timeRemaining -= driveToScoreTime + scoreTime;
        currentPose = nextIntake.getPose();
      } else if (shootWhileMoving) {
        // SWD but last cycle (no next intake) — drive toward scoring waypoint as fallback
        actions.add(new AutoAction.ScoreAt(nextTarget, true, null));
        timeRemaining -= driveToScoreTime + scoreTime;
        currentPose = nextTarget.toPose();
      } else {
        // Stop-and-shoot: drive to scoring waypoint, stop, aim, fire
        actions.add(new AutoAction.ScoreAt(nextTarget));
        timeRemaining -= driveToScoreTime + scoreTime;
        currentPose = nextTarget.toPose();
      }

      scoredLocations.add(nextTarget);
      cyclesCompleted++;
    }

    // ===== Step 4: TOWER Climb =====
    // Note: Only LEVEL 1 is available during AUTO (15 pts). LEVEL 2/3 are teleop only.
    // But we plan it here so the robot is already in position for teleop climb.
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
   * Pick the best scoring location given the current pose, priority list, and previously scored
   * locations.
   *
   * <p>Strategy:
   *
   * <ol>
   *   <li>Prefer locations earlier in the priority list.
   *   <li>Among equal-priority locations, prefer closer ones.
   *   <li>Deprioritize locations we've already scored at (but don't exclude — we might revisit).
   * </ol>
   */
  private static ScoringWaypoint pickBestScoringWaypoint(
      Pose2d currentPose, List<ScoringWaypoint> priority, List<ScoringWaypoint> alreadyScored) {

    ScoringWaypoint best = null;
    double bestScore = Double.MAX_VALUE;

    for (int i = 0; i < priority.size(); i++) {
      ScoringWaypoint loc = priority.get(i);

      // Compute a score: lower = better
      // Priority index is the primary factor, distance is secondary
      double priorityWeight = i * 5.0; // 5 meters per priority position
      double distance = currentPose.getTranslation().getDistance(loc.getPosition());

      // Penalty for revisiting
      double revisitPenalty = 0.0;
      if (alreadyScored != null && alreadyScored.contains(loc)) {
        revisitPenalty = 3.0; // 3m penalty for revisiting
      }

      double score = priorityWeight + distance + revisitPenalty;

      if (score < bestScore) {
        bestScore = score;
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
   * <p>Strategy:
   *
   * <ol>
   *   <li>Start at {@code sequenceIndex % sequence.size()} and walk forward (wrapping around),
   *       returning the first location that is not depleted. This means the input string "UDO"
   *       produces the order U → D → O → U → U → U… (D and O are one-shot and get depleted, so
   *       after the first pass U is the only remaining option).
   *   <li>If no sequence location is usable, fall back to the closest non-depleted location from
   *       all IntakeLocations.
   * </ol>
   *
   * @param sequenceIndex Which cycle we're on (0-based). Advances through the sequence list.
   */
  private static IntakeLocation pickBestIntakeLocation(
      Pose2d currentPose,
      List<IntakeLocation> sequence,
      Set<IntakeLocation> depleted,
      int sequenceIndex) {

    if (!sequence.isEmpty()) {
      int size = sequence.size();
      int start = sequenceIndex % size;
      // Walk forward from the sequence position, wrapping once through the full list
      for (int i = 0; i < size; i++) {
        IntakeLocation loc = sequence.get((start + i) % size);
        if (!depleted.contains(loc)) {
          return loc;
        }
      }
    }

    // Fallback: closest non-depleted location not in the sequence list
    IntakeLocation best = null;
    double bestDist = Double.MAX_VALUE;

    for (IntakeLocation loc : IntakeLocation.values()) {
      if (depleted.contains(loc)) {
        continue;
      }
      double dist = currentPose.getTranslation().getDistance(loc.getPose().getTranslation());
      if (dist < bestDist) {
        bestDist = dist;
        best = loc;
      }
    }

    return best;
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
