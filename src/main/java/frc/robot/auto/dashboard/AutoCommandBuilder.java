// Copyright (c) 2026 FRC Team 0 (Amped)
// Dashboard-driven autonomous system — Command builder

package frc.robot.auto.dashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.auto.dashboard.FieldConstants.ScoringWaypoint;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import frc.robot.util.ChezySequenceCommandGroup;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * Converts a list of {@link AutoAction}s from the {@link AutoPlanner} into a single runnable WPILib
 * {@link Command}.
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

  private final DriveSwerveDrivetrain drive;
  private final Superstructure superstructure;

  @SuppressWarnings("unused") // Reserved for future motion compensation / shoot-while-driving
  private final RobotState robotState;

  public AutoCommandBuilder(
      DriveSwerveDrivetrain drive, Superstructure superstructure, RobotState robotState) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.robotState = robotState;
  }

  /**
   * Build a single Command that executes the entire auto plan.
   *
   * @param actions Ordered list of actions from the planner
   * @return A command that executes the full sequence
   */
  public Command buildAutoCommand(List<AutoAction> actions) {
    if (actions.isEmpty()) {
      return Commands.print("[DashboardAuto] Empty action list — doing nothing.")
          .withName("DashboardAuto_Empty");
    }

    ChezySequenceCommandGroup sequence = new ChezySequenceCommandGroup();

    sequence.addCommands(
        Commands.print("[DashboardAuto] Starting auto with " + actions.size() + " actions"));

    for (int i = 0; i < actions.size(); i++) {
      AutoAction action = actions.get(i);
      final int step = i + 1;
      final int total = actions.size();

      // Log step
      sequence.addCommands(
          Commands.runOnce(
              () ->
                  Logger.recordOutput(
                      "DashboardAuto/CurrentStep", step + "/" + total + ": " + action.describe())));

      // Build the command for this action
      Command actionCmd = buildActionCommand(action);
      if (actionCmd != null) {
        sequence.addCommands(actionCmd);
      }
    }

    sequence.addCommands(
        Commands.runOnce(() -> Logger.recordOutput("DashboardAuto/CurrentStep", "COMPLETE")));
    sequence.addCommands(Commands.print("[DashboardAuto] Auto sequence complete!"));

    sequence.setName("DashboardAuto");
    return sequence;
  }

  // ===== Per-Action Command Builders =====

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
   * Score the preloaded FUEL. The robot is assumed to be at/near the HUB shooting position. Drive
   * to the exact pose, then aim and shoot.
   */
  private Command buildScorePreload(AutoAction.ScorePreload action) {
    Pose2d target = action.getLocation().toPose();
    return Commands.sequence(
            Commands.print("[DashboardAuto] Scoring preload at " + action.getLocation().name()),
            // Short drive to exact scoring pose
            Commands.defer(
                () -> AutoBuilder.pathfindToPose(target, getPathConstraints(), 0.0), Set.of(drive)),
            // Aim and shoot
            buildScoringSequence(action.getLocation()))
        .withName("ScorePreload_" + action.getLocation().name());
  }

  /** Drive to a HUB shooting position, then aim and shoot FUEL. */
  private Command buildScoreAt(AutoAction.ScoreAt action) {
    Pose2d target = action.getLocation().toPose();

    if (action.isShootWhileMoving()) {
      // Shoot-while-driving: start aiming superstructure while pathfinding
      // TODO: Consider using superstructure.intakeWhileAimingHub() for combined states
      //       when collecting FUEL en route to the HUB (intake + aim simultaneously).
      return Commands.sequence(
              Commands.print(
                  "[DashboardAuto] Shoot-while-driving to " + action.getLocation().name()),
              Commands.parallel(
                  // Drive to scoring location
                  Commands.defer(
                      () -> AutoBuilder.pathfindToPose(target, getPathConstraints(), 0.0),
                      Set.of(drive)),
                  // Pre-aim while driving
                  superstructure.aimHubFromAllianceZone()),
              // Fire once arrived and aimed
              buildScoringSequence(action.getLocation()))
          .withName("ScoreWhileMoving_" + action.getLocation().name());
    } else {
      // Standard: drive first, then aim and shoot
      return Commands.sequence(
              Commands.print("[DashboardAuto] Driving to score at " + action.getLocation().name()),
              Commands.defer(
                  () -> AutoBuilder.pathfindToPose(target, getPathConstraints(), 0.0),
                  Set.of(drive)),
              buildScoringSequence(action.getLocation()))
          .withName("ScoreAt_" + action.getLocation().name());
    }
  }

  /**
   * Drive to a FUEL intake location (OUTPOST, DEPOT, or NEUTRAL ZONE) and collect FUEL.
   *
   * <p>TODO: Differentiate intake behavior by location type:
   *
   * <ul>
   *   <li>OUTPOST — human player feeds via CHUTE; may need different positioning/timing
   *   <li>DEPOT — floor-level bin; current intakeFromGround() is appropriate
   *   <li>NEUTRAL ZONE — scattered ground FUEL; current intakeFromGround() is appropriate
   * </ul>
   *
   * <p>When Superstructure gains an intakeFromOutpost() command, switch on
   * action.getLocation().zone.
   */
  private Command buildIntakeAt(AutoAction.IntakeAt action) {
    Pose2d target = action.getLocation().getPose();
    return Commands.sequence(
            Commands.print("[DashboardAuto] Intaking at " + action.getLocation().name()),
            Commands.parallel(
                // Drive to intake location
                Commands.defer(
                    () -> AutoBuilder.pathfindToPose(target, getPathConstraints(), 0.0),
                    Set.of(drive)),
                // Deploy intake while driving
                superstructure.intakeFromGround()),
            // Brief wait to ensure FUEL is secured
            Commands.waitSeconds(0.3),
            // Stow intake
            superstructure.idle())
        .withName("IntakeAt_" + action.getLocation().name());
  }

  /** Drive to an arbitrary pose. */
  private Command buildDriveTo(AutoAction.DriveTo action) {
    Pose2d target = action.getTarget();
    return Commands.sequence(
            Commands.print("[DashboardAuto] Driving to " + action.getLabel()),
            Commands.defer(
                () -> AutoBuilder.pathfindToPose(target, getPathConstraints(), 0.0), Set.of(drive)))
        .withName("DriveTo_" + action.getLabel());
  }

  /** Execute the TOWER climb sequence. */
  private Command buildClimb(AutoAction.Climb action) {
    return Commands.sequence(
            Commands.print("[DashboardAuto] Climbing TOWER " + action.getClimbLevel().name()),
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
   * Build the aim + fire sequence for scoring FUEL into the HUB.
   *
   * @param location The HUB shooting position (for logging)
   * @return Command sequence to aim at HUB and fire FUEL
   */
  private Command buildScoringSequence(ScoringWaypoint location) {
    return Commands.sequence(
        // Aim at hub
        superstructure.aimHubFromAllianceZone(),
        // Brief settle time
        Commands.waitSeconds(0.2),
        // Fire
        superstructure.scoreHubFromAllianceZone(),
        // Brief post-shot delay
        Commands.waitSeconds(0.15),
        // Return to idle
        superstructure.idle());
  }

  /**
   * Get the path constraints for pathfinding. Delegates to the drive subsystem's single source of
   * truth (which reads from Constants.AutoConstants).
   */
  private com.pathplanner.lib.path.PathConstraints getPathConstraints() {
    return drive.getPathConstraints();
  }
}
