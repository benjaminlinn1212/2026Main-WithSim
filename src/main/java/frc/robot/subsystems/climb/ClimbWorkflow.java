package frc.robot.subsystems.climb;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;

/**
 * Full autonomous climb workflow for 3-level climb (L1, L2, L3). Handles pathfinding to climb
 * position and executing the full climb sequence.
 */
public class ClimbWorkflow {

  public static final Pose2d CLIMB_POSITION =
      new Pose2d(new Translation2d(5.0, 4.0), Rotation2d.fromDegrees(0.0));

  public static final PathConstraints CLIMB_APPROACH_CONSTRAINTS =
      new PathConstraints(2.0, 2.0, Units.degreesToRadians(360), Units.degreesToRadians(540));

  /** Full autonomous climb: pathfind → enter mode → execute all states */
  public static Command fullAutoClimb(Superstructure superstructure) {
    return Commands.sequence(
            Commands.print("Auto Climb: Pathfinding..."),
            AutoBuilder.pathfindToPose(CLIMB_POSITION, CLIMB_APPROACH_CONSTRAINTS, 0.0),
            Commands.waitSeconds(0.5),
            Commands.print("Auto Climb: Entering climb mode..."),
            superstructure.enterClimbMode(),
            Commands.waitSeconds(0.5),
            Commands.print("Auto Climb: Executing climb sequence..."),
            executeAllClimbStates(superstructure.getClimbSubsystem()),
            Commands.print("Auto Climb: COMPLETE!"))
        .withName("FullAutoClimb");
  }

  private static Command executeAllClimbStates(ClimbSubsystem climb) {
    return Commands.sequence(
        // STOWED
        climb.setStateCommand(ClimbState.STOWED),
        Commands.waitSeconds(0.5),

        // REACH_L1 (2.0s path)
        climb.setStateCommand(ClimbState.REACH_L1),
        Commands.waitSeconds(2.5),

        // PULL_L1_AUTO (2.0s path)
        climb.setStateCommand(ClimbState.PULL_L1_AUTO),
        Commands.waitSeconds(2.5),

        // DROP_L1_AUTO (1.5s path)
        climb.setStateCommand(ClimbState.DROP_L1_AUTO),
        Commands.waitSeconds(2.0),

        // PULL_L1 (2.5s path)
        climb.setStateCommand(ClimbState.PULL_L1),
        Commands.waitSeconds(3.0),

        // REACH_L2 (2.5s path)
        climb.setStateCommand(ClimbState.REACH_L2),
        Commands.waitSeconds(3.0),

        // PULL_L2 (2.5s path)
        climb.setStateCommand(ClimbState.PULL_L2),
        Commands.waitSeconds(3.0),

        // REACH_L3 (2.5s path)
        climb.setStateCommand(ClimbState.REACH_L3),
        Commands.waitSeconds(3.0),

        // PULL_L3 (3.0s path)
        climb.setStateCommand(ClimbState.PULL_L3),
        Commands.waitSeconds(3.5));
  }
}
