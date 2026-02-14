// Copyright (c) 2026 FRC Team 0 (Amped)
// Dashboard-driven autonomous system — The planning brain for REBUILT

package frc.robot.auto.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto.dashboard.FieldConstants.ClimbLevel;
import frc.robot.auto.dashboard.FieldConstants.ClimbPose;
import frc.robot.auto.dashboard.FieldConstants.IntakeLocation;
import frc.robot.auto.dashboard.FieldConstants.Lane;
import frc.robot.auto.dashboard.FieldConstants.ScoringWaypoint;
import java.util.ArrayList;
import java.util.Collections;
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
 *   <li>If preloaded FUEL, drive to best HUB shooting position and score.
 *   <li>For each remaining cycle (up to maxCycles): collect FUEL from OUTPOST/DEPOT/NEUTRAL ZONE →
 *       drive to HUB shooting position → score FUEL.
 *   <li>Filter all locations through lane constraints (TRENCH/BUMP sides) and partner avoidance.
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
    Set<Lane> effectiveLanes = settings.getEffectiveLanes();

    // ===== Step 1: Set start pose =====
    actions.add(new AutoAction.SetStartPose(currentPose));

    // ===== Step 2: Score preloaded FUEL =====
    // The robot can preload 1-8 FUEL. All preloaded FUEL is dumped in a single scoring action
    // at the first HUB shooting position. More FUEL = slightly longer score duration.
    if (settings.hasPreload()) {
      ScoringWaypoint preloadTarget =
          pickBestScoringWaypoint(currentPose, settings.getScoringPriority(), effectiveLanes, null);

      if (preloadTarget != null) {
        // Drive to HUB shooting position and dump preloaded FUEL
        double driveTime =
            FieldConstants.estimateDriveTime(currentPose, preloadTarget.toPose()) * timeMargin;
        // Scoring time is roughly constant regardless of preload count
        // (the actual command is aim + shoot, not duration-scaled)
        double preloadScoreTime = FieldConstants.SCORE_DURATION * timeMargin;

        if (driveTime + preloadScoreTime < timeRemaining) {
          actions.add(new AutoAction.ScorePreload(preloadTarget));
          timeRemaining -= driveTime + preloadScoreTime;
          currentPose = preloadTarget.toPose();
          Logger.recordOutput("AutoPlanner/PreloadFuelCount", settings.getPreloadCount());
        }
      }
    }

    // ===== Step 3: Cycle FUEL scoring =====
    // Each cycle: collect FUEL → drive to HUB → shoot FUEL
    int cyclesCompleted = 0;
    // Track which HUB positions we've already shot from to diversify angles
    List<ScoringWaypoint> scoredLocations = new ArrayList<>();

    while (cyclesCompleted < settings.getMaxCycles()) {
      // Pick next HUB shooting position
      ScoringWaypoint nextTarget =
          pickBestScoringWaypoint(
              currentPose, settings.getScoringPriority(), effectiveLanes, scoredLocations);

      if (nextTarget == null) {
        Logger.recordOutput("AutoPlanner/StopReason", "No valid HUB shooting positions");
        break;
      }

      // Pick FUEL intake location (OUTPOST, DEPOT, or NEUTRAL ZONE)
      IntakeLocation intakeLoc =
          pickBestIntakeLocation(currentPose, settings.getPreferredIntake(), effectiveLanes);

      if (intakeLoc == null) {
        Logger.recordOutput("AutoPlanner/StopReason", "No valid FUEL intake locations");
        break;
      }

      // Estimate cycle time: drive-to-intake + intake + drive-to-HUB + score
      double driveToIntakeTime =
          FieldConstants.estimateDriveTime(currentPose, intakeLoc.getPose()) * timeMargin;
      double intakeTime = FieldConstants.INTAKE_DURATION * timeMargin;
      double driveToScoreTime =
          FieldConstants.estimateDriveTime(intakeLoc.getPose(), nextTarget.toPose()) * timeMargin;
      double scoreTime = FieldConstants.SCORE_DURATION * timeMargin;

      double cycleTime = driveToIntakeTime + intakeTime + driveToScoreTime + scoreTime;

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
      timeRemaining -= driveToIntakeTime + intakeTime;
      currentPose = intakeLoc.getPose();

      actions.add(new AutoAction.ScoreAt(nextTarget, settings.isShootWhileDriving()));
      timeRemaining -= driveToScoreTime + scoreTime;
      currentPose = nextTarget.toPose();

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
   * Pick the best scoring location given the current pose, priority list, lane constraints, and
   * previously scored locations.
   *
   * <p>Strategy:
   *
   * <ol>
   *   <li>Filter by lane constraint.
   *   <li>Prefer locations earlier in the priority list.
   *   <li>Among equal-priority locations, prefer closer ones.
   *   <li>Deprioritize locations we've already scored at (but don't exclude — we might revisit).
   * </ol>
   */
  private static ScoringWaypoint pickBestScoringWaypoint(
      Pose2d currentPose,
      List<ScoringWaypoint> priority,
      Set<Lane> allowedLanes,
      List<ScoringWaypoint> alreadyScored) {

    ScoringWaypoint best = null;
    double bestScore = Double.MAX_VALUE;

    for (int i = 0; i < priority.size(); i++) {
      ScoringWaypoint loc = priority.get(i);

      // Lane filter
      if (!allowedLanes.contains(loc.lane)) {
        continue;
      }

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
   * Pick the best intake location given the current pose, preferred location, and lane constraints.
   */
  private static IntakeLocation pickBestIntakeLocation(
      Pose2d currentPose, IntakeLocation preferred, Set<Lane> allowedLanes) {

    // If the preferred location is in an allowed lane, use it
    if (allowedLanes.contains(preferred.lane)) {
      return preferred;
    }

    // Otherwise, find the closest allowed intake location
    IntakeLocation best = null;
    double bestDist = Double.MAX_VALUE;

    for (IntakeLocation loc : IntakeLocation.values()) {
      if (!allowedLanes.contains(loc.lane)) {
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
      Pose2d target = getActionTargetPose(action);
      if (target != null) {
        total += FieldConstants.estimateDriveTime(currentPose, target);
        currentPose = target;
      }
      // Add the action's own duration
      total += action.estimatedDuration();
    }

    return total;
  }

  /** Get the target pose of an action (where the robot ends up after executing it). */
  private static Pose2d getActionTargetPose(AutoAction action) {
    return action.getTargetPose();
  }
}
