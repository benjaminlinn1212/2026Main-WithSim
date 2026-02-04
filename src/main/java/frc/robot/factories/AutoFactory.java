// Copyright (c) 2026 FRC Team 0 (Amped)
// Based on Team 254's 2025 AutoFactory pattern

package frc.robot.factories;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.commands.PathfindingAutoAlignCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.List;
import java.util.Set;

/**
 * Factory for creating autonomous commands using static methods. Based on Team 254's AutoFactory
 * pattern where all methods are static and accept subsystems as parameters.
 */
public class AutoFactory {

  // Scoring positions (example coordinates - adjust for your field)
  private static final Pose2d POSITION_A =
      new Pose2d(new Translation2d(2.5, 7.0), Rotation2d.fromDegrees(60));
  private static final Pose2d POSITION_B =
      new Pose2d(new Translation2d(2.5, 5.5), Rotation2d.fromDegrees(0));
  private static final Pose2d POSITION_C =
      new Pose2d(new Translation2d(2.5, 4.0), Rotation2d.fromDegrees(-60));
  private static final Pose2d POSITION_D =
      new Pose2d(new Translation2d(2.5, 2.5), Rotation2d.fromDegrees(-90));

  /**
   * Get pathfinding command to score at a specific waypoint.
   *
   * @param drive Drive subsystem
   * @param superstructure Superstructure subsystem
   * @param robotState Robot state tracker
   * @param waypoint Waypoint character (A, B, C, D)
   * @param warmupCommands Optional list to track warmup commands
   * @return Command to pathfind and score at waypoint
   */
  public static Command getPathfindToWaypointCommand(
      DriveSwerveDrivetrain drive,
      Superstructure superstructure,
      RobotState robotState,
      char waypoint,
      List<PathfindingAutoAlignCommand> warmupCommands) {

    Pose2d targetPose;
    String approachPathName;

    switch (Character.toUpperCase(waypoint)) {
      case 'A':
        targetPose = POSITION_A;
        approachPathName = null;
        break;
      case 'B':
        targetPose = POSITION_B;
        approachPathName = null;
        break;
      case 'C':
        targetPose = POSITION_C;
        approachPathName = null;
        break;
      case 'D':
        targetPose = POSITION_D;
        approachPathName = "Approach D";
        break;
      default:
        System.err.println("[AutoFactory] Unknown waypoint: " + waypoint);
        return Commands.none();
    }

    PathfindingAutoAlignCommand pathfindingCmd =
        new PathfindingAutoAlignCommand(
            drive, robotState, () -> targetPose, approachPathName, AutoFactory::getPath);

    if (warmupCommands != null) {
      warmupCommands.add(pathfindingCmd);
    }

    // SIMPLIFIED: Just pathfind, no subsystem actions for now
    return Commands.sequence(
        Commands.print("[AutoFactory] Starting pathfind to " + waypoint),
        pathfindingCmd,
        Commands.print("[AutoFactory] Reached waypoint " + waypoint));
  }

  /**
   * Overload without warmup tracking.
   *
   * @param drive Drive subsystem
   * @param superstructure Superstructure subsystem
   * @param robotState Robot state tracker
   * @param waypoint Waypoint character
   * @return Command to pathfind and score
   */
  public static Command getPathfindToWaypointCommand(
      DriveSwerveDrivetrain drive,
      Superstructure superstructure,
      RobotState robotState,
      char waypoint) {
    return getPathfindToWaypointCommand(drive, superstructure, robotState, waypoint, null);
  }

  /**
   * Get intake command.
   *
   * @param superstructure Superstructure subsystem
   * @return Command to intake game piece
   */
  public static Command getIntakeCommand(Superstructure superstructure) {
    // SIMPLIFIED: Just print and wait
    return Commands.sequence(
        Commands.print("[AutoFactory] Intaking..."), Commands.waitSeconds(0.5));
  }

  /**
   * Get test obstacle avoidance command. Pathfinds from (1,1) to field midpoint to test AD*
   * algorithm.
   *
   * @param drive Drive subsystem
   * @return Test command
   */
  public static Command getTestObstacleAvoidanceCommand(DriveSwerveDrivetrain drive) {
    Pose2d startPose = new Pose2d(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0));
    Pose2d midpointPose = new Pose2d(new Translation2d(8.77, 4.026), Rotation2d.fromDegrees(0));

    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(startPose), drive),
        Commands.print("[Test] Robot pose set to (1,1)"),
        Commands.print("[Test] Starting obstacle avoidance test from (1,1) to midpoint..."),
        Commands.defer(
            () -> AutoBuilder.pathfindToPose(midpointPose, drive.getPathConstraints(), 0.0),
            Set.of(drive)),
        Commands.print("[Test] Reached midpoint! Obstacle avoidance test complete."));
  }

  /**
   * Get a path by name. Loads from PathPlanner deploy directory.
   *
   * @param pathName Name of the path file
   * @return PathPlannerPath or null if loading fails
   */
  private static PathPlannerPath getPath(String pathName) {
    try {
      return PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      System.err.println("[AutoFactory] Failed to load path: " + pathName);
      return null;
    }
  }

  /**
   * Get the scoring position for a waypoint character.
   *
   * @param waypoint The waypoint character
   * @return The pose, or null if invalid
   */
  public static Pose2d getWaypointPose(char waypoint) {
    switch (Character.toUpperCase(waypoint)) {
      case 'A':
        return POSITION_A;
      case 'B':
        return POSITION_B;
      case 'C':
        return POSITION_C;
      case 'D':
        return POSITION_D;
      default:
        return null;
    }
  }
}
