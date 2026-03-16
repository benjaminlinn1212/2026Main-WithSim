// Copyright (c) 2026 FRC Team 10922 (Amped)
// Simple auto that follows a single pre-drawn PathPlanner path.

package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.dashboard.FieldConstants;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.Set;

/**
 * Utility auto that follows a single pre-drawn PathPlanner path. The robot's starting pose is
 * seeded from the path's starting holonomic pose. No superstructure actions — purely for testing
 * path following.
 */
public class RunPathAuto {

  private final DriveSwerveDrivetrain drive;
  private final String pathName;

  /**
   * @param drive The swerve drivetrain subsystem.
   * @param pathName The path file name (without extension) in deploy/pathplanner/paths/.
   */
  public RunPathAuto(DriveSwerveDrivetrain drive, String pathName) {
    this.drive = drive;
    this.pathName = pathName;
  }

  /**
   * Build the auto command. Seeds the pose from the path's start, then follows it.
   *
   * @return A deferred command that seeds the pose and follows the path.
   */
  public Command buildCommand() {
    return Commands.defer(
        () -> {
          try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            Pose2d startPose = allianceAwarePose(path);

            return Commands.sequence(
                // Seed pose — same pattern as OutpostAuto / UpperClimbAuto
                Commands.runOnce(
                    () -> {
                      if (startPose != null) {
                        drive.setPose(startPose);
                      }
                    }),
                AutoBuilder.followPath(path));
          } catch (Exception e) {
            System.err.println("[RunPathAuto] Failed to load path: " + pathName);
            e.printStackTrace();
            return Commands.print("[RunPathAuto] ERROR: Path not found: " + pathName);
          }
        },
        Set.of(drive));
  }

  /**
   * Returns the starting pose for the configured path (alliance-corrected), or {@code null} if the
   * path cannot be loaded. Used by {@code RobotContainer.getAutoStartingPose()} for pre-seeding
   * during disabled.
   */
  public Pose2d getStartingPose() {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return allianceAwarePose(path);
    } catch (Exception e) {
      return null;
    }
  }

  /**
   * Get the path's starting holonomic pose, flipped for red alliance (point symmetry). Matches the
   * behavior of {@code FieldConstants.StartPose.getPose()}.
   */
  private static Pose2d allianceAwarePose(PathPlannerPath path) {
    Pose2d bluePose = path.getStartingHolonomicPose().orElse(null);
    if (bluePose == null) {
      return null;
    }
    return FieldConstants.isRedAlliance() ? FieldConstants.flipPose(bluePose) : bluePose;
  }
}
