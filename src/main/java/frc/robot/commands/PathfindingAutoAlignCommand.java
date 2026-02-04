// Copyright (c) 2026 FRC Team 0 (Amped)
// Based on Team 254's PathfindingAutoAlignCommand with join path support

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Pathfinding command that aligns to a target pose using pathfinding and an optional "join path"
 * for smooth final approach. Based on Team 254's PathfindingAutoAlignCommand pattern.
 */
public class PathfindingAutoAlignCommand extends Command {
  private final DriveSwerveDrivetrain drive;
  private final RobotState robotState;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final String approachPathName;
  private final Function<String, PathPlannerPath> pathLoader;

  private Command pathfindingCommand;

  /**
   * Create a pathfinding align command with optional join path.
   *
   * @param drive The swerve drive subsystem
   * @param robotState The robot state for current pose tracking
   * @param targetPoseSupplier Supplier for the final target pose
   * @param approachPathName Name of the approach path (for join path) - can be null for direct
   *     pathfinding
   * @param pathLoader Function to load PathPlannerPath by name
   */
  public PathfindingAutoAlignCommand(
      DriveSwerveDrivetrain drive,
      RobotState robotState,
      Supplier<Pose2d> targetPoseSupplier,
      String approachPathName,
      Function<String, PathPlannerPath> pathLoader) {
    this.drive = drive;
    this.robotState = robotState;
    this.targetPoseSupplier = targetPoseSupplier;
    this.approachPathName = approachPathName;
    this.pathLoader = pathLoader;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = targetPoseSupplier.get();
    double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    System.out.println("[PathfindingAutoAlign] initialize() called");
    System.out.println("[PathfindingAutoAlign]   Current: " + currentPose);
    System.out.println("[PathfindingAutoAlign]   Target:  " + targetPose);
    System.out.println(
        "[PathfindingAutoAlign]   Distance: " + String.format("%.2f", distance) + " meters");

    if (approachPathName != null && pathLoader != null) {
      // Build join path: pathfinding -> approach path
      PathPlannerPath approachPath = pathLoader.apply(approachPathName);

      if (approachPath != null) {
        // Use pathfindThenFollowPath for smooth transition
        pathfindingCommand = AutoBuilder.pathfindThenFollowPath(approachPath, getPathConstraints());
        System.out.println(
            "[PathfindingAutoAlign] Pathfinding with join path to: " + targetPose.getTranslation());
      } else {
        // Fallback: direct pathfinding without join path
        pathfindDirect(targetPose);
      }
    } else {
      // No join path specified - direct pathfinding
      pathfindDirect(targetPose);
    }

    // Initialize the pathfinding command (don't schedule it separately!)
    if (pathfindingCommand != null) {
      System.out.println(
          "[PathfindingAutoAlign] Initializing wrapped command: " + pathfindingCommand.getName());
      pathfindingCommand.initialize();
    } else {
      System.err.println("[PathfindingAutoAlign] ERROR: pathfindingCommand is null!");
    }
  }

  /**
   * Fallback method for direct pathfinding without join path.
   *
   * @param targetPose The target pose
   */
  private void pathfindDirect(Pose2d targetPose) {
    pathfindingCommand =
        AutoBuilder.pathfindToPose(
            targetPose, getPathConstraints(), 0.0 // Goal end velocity
            );
    System.out.println(
        "[PathfindingAutoAlign] Direct pathfinding to: " + targetPose.getTranslation());
  }

  /**
   * Get path constraints for pathfinding. Uses moderate speed for safe autonomous navigation.
   *
   * @return PathConstraints for pathfinding
   */
  private PathConstraints getPathConstraints() {
    // Moderate constraints for safe auto navigation
    return new PathConstraints(
        3.0, // Max velocity (m/s)
        3.0, // Max acceleration (m/s²)
        Math.toRadians(360), // Max angular velocity (rad/s)
        Math.toRadians(540) // Max angular acceleration (rad/s²)
        );
  }

  @Override
  public void execute() {
    // Execute the pathfinding command
    if (pathfindingCommand != null) {
      pathfindingCommand.execute();
    } else {
      System.err.println(
          "[PathfindingAutoAlign] ERROR: execute() called but pathfindingCommand is null!");
    }
  }

  @Override
  public boolean isFinished() {
    if (pathfindingCommand == null) {
      System.err.println(
          "[PathfindingAutoAlign] isFinished() called but pathfindingCommand is null - returning true");
      return true; // Finish immediately if no command
    }

    boolean finished = pathfindingCommand.isFinished();

    if (finished) {
      System.out.println("[PathfindingAutoAlign] Wrapped command reports finished!");
    }

    return finished;
  }

  @Override
  public void end(boolean interrupted) {
    if (pathfindingCommand != null) {
      pathfindingCommand.end(interrupted);
    }
  }

  /**
   * Get warmup command for preloading path. Called during autonomous init.
   *
   * @param startPose Starting pose for warmup
   * @return This command for chaining
   */
  public PathfindingAutoAlignCommand getWarmupCommand(Pose2d startPose) {
    // Preload the approach path if specified
    if (approachPathName != null && pathLoader != null) {
      pathLoader.apply(approachPathName);
    }
    return this;
  }
}
