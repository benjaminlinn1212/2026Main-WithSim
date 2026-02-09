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
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.function.BooleanSupplier;

/**
 * Full autonomous climb workflow for 3-level climb (L1, L2, L3). Handles pathfinding to climb
 * position and executing the full climb sequence.
 */
public class ClimbWorkflow {

  public static final Pose2d CLIMB_POSITION =
      new Pose2d(new Translation2d(2.0, 2.0), Rotation2d.fromDegrees(0.0));

  public static final PathConstraints CLIMB_APPROACH_CONSTRAINTS =
      new PathConstraints(2.0, 2.0, Units.degreesToRadians(360), Units.degreesToRadians(540));

  /** Full autonomous climb: pathfind → enter mode → execute all states */
  public static Command fullAutoClimb(
      Superstructure superstructure, DriveSwerveDrivetrain drivetrain) {
    BooleanSupplier readyForClimb = superstructure::isReadyForClimb;

    return Commands.sequence(
            Commands.print("Auto Climb: Pathfinding..."),
            AutoBuilder.pathfindToPose(CLIMB_POSITION, CLIMB_APPROACH_CONSTRAINTS, 0.0),
            Commands.print("Auto Climb: At position, entering climb mode..."),
            superstructure.enterClimbMode(),
            Commands.waitUntil(readyForClimb),
            Commands.print("Auto Climb: All subsystems stowed, executing climb sequence..."),
            executeAllClimbStates(superstructure.getClimbSubsystem()),
            Commands.print("Auto Climb: COMPLETE!"))
        .withName("FullAutoClimb");
  }

  private static Command executeAllClimbStates(ClimbSubsystem climb) {
    return Commands.sequence(
        // STOWED
        Commands.print("======================================"),
        Commands.print("[CLIMB] State: STOWED"),
        climb.setStateCommand(ClimbState.STOWED),
        Commands.print("[CLIMB] ✓ STOWED complete"),

        // REACH_L1 (2.0s path)
        Commands.print("[CLIMB] State: REACHING L1..."),
        climb.setStateCommand(ClimbState.REACH_L1),
        Commands.print("[CLIMB] ✓ REACHED L1"),

        // PULL_L1 (2.5s path)
        Commands.print("[CLIMB] State: PULLING L1..."),
        climb.setStateCommand(ClimbState.PULL_L1),
        Commands.print("[CLIMB] ✓ PULLED L1"),

        // REACH_L2 (2.5s path)
        Commands.print("[CLIMB] State: REACHING L2..."),
        climb.setStateCommand(ClimbState.REACH_L2),
        Commands.print("[CLIMB] ✓ REACHED L2"),

        // PULL_L2 (2.5s path)
        Commands.print("[CLIMB] State: PULLING L2..."),
        climb.setStateCommand(ClimbState.PULL_L2),
        Commands.print("[CLIMB] ✓ PULLED L2"),

        // REACH_L3 (2.5s path)
        Commands.print("[CLIMB] State: REACHING L3..."),
        climb.setStateCommand(ClimbState.REACH_L3),
        Commands.print("[CLIMB] ✓ REACHED L3"),

        // PULL_L3 (3.0s path)
        Commands.print("[CLIMB] State: PULLING L3..."),
        climb.setStateCommand(ClimbState.PULL_L3),
        Commands.print("[CLIMB] ✓ PULLED L3"),
        Commands.print("======================================"),
        Commands.print("[CLIMB] ALL STATES COMPLETE!"));
  }
}
