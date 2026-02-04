// Copyright (c) 2026 FRC Team 0 (Amped)
// Based on Team 254's PathfindingAuto.java dynamic sequence execution

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.commands.PathfindingAutoAlignCommand;
import frc.robot.factories.AutoFactory;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import frc.robot.util.ChezySequenceCommandGroup;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Dynamic autonomous routine that executes a sequence of actions based on a character string. Based
 * on Team 254's PathfindingAuto pattern where each character represents a scoring location or
 * action.
 *
 * <p>Example sequences: - "ABC" = Score at A, then B, then C - "AIA" = Score at A, intake, score at
 * A again - "ABCD" = Score at all four positions
 */
public class PathfindingAuto extends ChezySequenceCommandGroup {
  private final DriveSwerveDrivetrain drive;
  private final Superstructure superstructure;
  private final RobotState robotState;
  private final String scoreSequence;

  // Track warmup commands for path preloading
  protected final List<PathfindingAutoAlignCommand> warmupCommands = new ArrayList<>();

  /**
   * Create a pathfinding auto from a character string.
   *
   * @param drive Drive subsystem
   * @param superstructure Superstructure subsystem
   * @param robotState Robot state tracker
   * @param scoreSequence String of characters representing the sequence (e.g., "ABC")
   */
  public PathfindingAuto(
      DriveSwerveDrivetrain drive,
      Superstructure superstructure,
      RobotState robotState,
      String scoreSequence) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.robotState = robotState;
    this.scoreSequence = scoreSequence.toLowerCase(); // 254 uses lowercase

    System.out.println(
        "[PathfindingAuto] Constructor called for sequence: '"
            + this.scoreSequence
            + "' at "
            + System.currentTimeMillis());

    // Build sequence directly in constructor (like Team 254)
    // Each command is deferred individually, not the whole sequence
    for (int i = 0; i < this.scoreSequence.length(); i++) {
      char action = this.scoreSequence.charAt(i);

      // Defer each command creation (so AutoBuilder is ready when scheduled)
      Command deferredCommand =
          Commands.defer(
              () -> {
                Command cmd = getCommandForAction(action);
                System.out.println(
                    "[PathfindingAuto] Creating command for action '" + action + "'");
                return cmd;
              },
              Set.of(drive));

      addCommands(deferredCommand);
    }

    System.out.println(
        "[PathfindingAuto] Sequence built with " + this.scoreSequence.length() + " actions");
  }

  /**
   * Get the command for a specific action character (Team 254 parsing style).
   *
   * @param action The action character (a, b, c, d, i, w, t, etc.) - lowercase per 254
   * @return The command to execute, or null if invalid
   */
  private Command getCommandForAction(char action) {
    // Handle scoring positions (a-d)
    char uppercase = Character.toUpperCase(action);
    if (uppercase >= 'A' && uppercase <= 'D') {
      return AutoFactory.getPathfindToWaypointCommand(
          drive, superstructure, robotState, uppercase, warmupCommands);
    }

    // Handle special actions
    switch (Character.toLowerCase(action)) {
      case 't':
        // Test obstacle avoidance
        return AutoFactory.getTestObstacleAvoidanceCommand(drive);

      case 'i':
        // Intake
        return AutoFactory.getIntakeCommand(superstructure);

      case 'w':
        // Wait 1 second
        return Commands.waitSeconds(1.0);

      case 'x':
        // Wait 0.5 seconds (shorter wait)
        return Commands.waitSeconds(0.5);

      case ' ':
        // Space = no operation (for readability in sequence string)
        return Commands.none();

      default:
        System.err.println("[PathfindingAuto] Unknown action: " + action);
        return Commands.none();
    }
  }

  /**
   * Get the score sequence string.
   *
   * @return The sequence string
   */
  public String getScoreSequence() {
    return scoreSequence;
  }

  /**
   * Get warmup command for preloading paths. Called during autonomousInit(). Runs while disabled to
   * preload all pathfinding computations.
   *
   * @return Command to warmup all paths used in this auto
   */
  public Command getWarmupCommand() {
    if (warmupCommands.isEmpty()) {
      return Commands.none();
    }

    ChezySequenceCommandGroup warmupSequence = new ChezySequenceCommandGroup();
    warmupSequence.addCommands(
        Commands.print("[PathfindingAuto] Warming up " + warmupCommands.size() + " paths..."));

    // Warmup each pathfinding command (preloads approach paths)
    Pose2d startPose = drive.getPose();
    for (PathfindingAutoAlignCommand cmd : warmupCommands) {
      cmd.getWarmupCommand(startPose);
    }

    warmupSequence.addCommands(Commands.print("[PathfindingAuto] Warmup complete!"));

    // Allow running while disabled
    return warmupSequence.ignoringDisable(true);
  }

  /**
   * Create a "simple" sequence auto that scores at all waypoints.
   *
   * @param drive Drive subsystem
   * @param superstructure Superstructure subsystem
   * @param robotState Robot state tracker
   * @return PathfindingAuto that scores at A, B, C, D
   */
  public static PathfindingAuto allWaypoints(
      DriveSwerveDrivetrain drive, Superstructure superstructure, RobotState robotState) {
    return new PathfindingAuto(drive, superstructure, robotState, "ABCD");
  }

  /**
   * Create a "fast" sequence auto that scores at A and B only.
   *
   * @param drive Drive subsystem
   * @param superstructure Superstructure subsystem
   * @param robotState Robot state tracker
   * @return PathfindingAuto that scores at A and B
   */
  public static PathfindingAuto fastSequence(
      DriveSwerveDrivetrain drive, Superstructure superstructure, RobotState robotState) {
    return new PathfindingAuto(drive, superstructure, robotState, "AB");
  }

  /**
   * Create a sequence with intakes between scores.
   *
   * @param drive Drive subsystem
   * @param superstructure Superstructure subsystem
   * @param robotState Robot state tracker
   * @return PathfindingAuto with intake actions
   */
  public static PathfindingAuto withIntakes(
      DriveSwerveDrivetrain drive, Superstructure superstructure, RobotState robotState) {
    return new PathfindingAuto(drive, superstructure, robotState, "AIBIC");
  }
}
