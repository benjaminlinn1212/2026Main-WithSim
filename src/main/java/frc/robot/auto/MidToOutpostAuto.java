// Copyright (c) 2026 FRC Team 10922 (Amped)
// Mid-start outpost autonomous — drive directly from center start to outpost, then shoot + jiggle.

package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.dashboard.FieldConstants;
import frc.robot.auto.dashboard.FieldConstants.StartPose;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * Simple outpost auto from a mid-field (CENTER) start. Sequence: seed pose → follow path to outpost
 * → extend intake to outpost pose → start shooting → wait 2 s → activate jiggle → continue shooting
 * until auto ends → idle.
 */
public class MidToOutpostAuto {

  // ==================== Path Names (must match files in deploy/pathplanner/paths/) ==============

  /** Path: CENTER start → OUTPOST. */
  private static final String PATH_MID_TO_OUTPOST = "Mid To Outpost";

  // ==================== Route Constants ====================

  /** Starting pose for this auto. Also used by RobotContainer.getAutoStartingPose(). */
  public static final StartPose START_POSE = StartPose.CENTER;

  // ==================== Timing Constants ====================

  /** Dwell at outpost before activating the jiggle (seconds). */
  private static final double DWELL_BEFORE_JIGGLE_SECONDS = 2.0;

  /**
   * Total shoot duration at outpost (seconds). Should be long — human player feeds continuously.
   */
  private static final double OUTPOST_SHOOT_DURATION_SECONDS = 12.0;

  // ==================== Dependencies ====================

  private final DriveSwerveDrivetrain drive;
  private final Superstructure superstructure;

  // ==================== Runtime State ====================

  private double autoStartTimestamp = 0.0;

  // ==================== Constructor ====================

  public MidToOutpostAuto(DriveSwerveDrivetrain drive, Superstructure superstructure) {
    this.drive = drive;
    this.superstructure = superstructure;
  }

  // ==================== Public Entry Points ====================

  /** Deferred command — safe for auto choosers. Builds a fresh command graph each invocation. */
  public Command getCommand() {
    return Commands.defer(this::buildCommand, Set.of(drive));
  }

  /**
   * Build the full auto command graph. Public so RobotContainer can call it directly for a fresh
   * instance each auto run (avoids the "already composed" error).
   */
  public Command buildCommand() {
    return Commands.deadline(
            Commands.sequence(
                buildInit(), buildDriveToOutpost(), buildOutpostSequence(), buildCleanup()),
            Commands.run(
                () ->
                    Logger.recordOutput(
                        "MidToOutpostAuto/AutoTimeRemaining", getAutoTimeRemaining())))
        .withName("MidToOutpostAuto");
  }

  // ==================== Phase Builders ====================

  /** Phase 0: Seed pose, reset superstructure, and capture auto start time. */
  private Command buildInit() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              autoStartTimestamp = Timer.getFPGATimestamp();
              drive.setPose(START_POSE.getPose());
            }),
        Commands.runOnce(superstructure::forceIdleState),
        Commands.print("[MidToOutpostAuto] Starting — Center start, outpost finish"));
  }

  /**
   * Phase 1: Follow path from CENTER start to OUTPOST with intake extended in outpost mode. We run
   * ONLY_INTAKE with outpost pivot throughout the path so the intake is ready to collect FUEL on
   * arrival.
   */
  private Command buildDriveToOutpost() {
    return Commands.sequence(
            Commands.print("[MidToOutpostAuto] Driving to outpost"),
            Commands.runOnce(() -> superstructure.setIntakeOutpostMode(true)),
            Commands.deadline(
                followPath(PATH_MID_TO_OUTPOST),
                Commands.run(
                    () -> superstructure.forceWantedState(SuperstructureState.ONLY_INTAKE),
                    superstructure)))
        .withName("DriveToOutpost");
  }

  /**
   * Phase 2: At the outpost — start shooting (aiming while intaking + feeding), wait 2 s, then
   * activate the intake pivot jiggle to dislodge stuck FUEL. Continue shooting for the remaining
   * duration.
   */
  private Command buildOutpostSequence() {
    return Commands.sequence(
            Commands.print("[MidToOutpostAuto] At outpost — starting to shoot"),
            // Start aiming + intaking + feeding
            superstructure.aimingWhileIntaking(),
            Commands.runOnce(() -> superstructure.setFeedingRequested(true)),
            // Wait before activating jiggle
            Commands.waitSeconds(DWELL_BEFORE_JIGGLE_SECONDS),
            Commands.print("[MidToOutpostAuto] Activating outpost jiggle"),
            Commands.runOnce(() -> superstructure.setIntakeOutpostJiggleMode(true)),
            // Continue shooting for the remaining duration
            Commands.waitSeconds(OUTPOST_SHOOT_DURATION_SECONDS - DWELL_BEFORE_JIGGLE_SECONDS),
            // Stop feeding and disable jiggle/outpost modes
            Commands.runOnce(
                () -> {
                  superstructure.setFeedingRequested(false);
                  superstructure.setIntakeOutpostJiggleMode(false);
                  superstructure.setIntakeOutpostMode(false);
                }))
        .withName("OutpostShootAndJiggle");
  }

  /** Phase 3: Return to idle. */
  private Command buildCleanup() {
    return Commands.sequence(superstructure.idle(), Commands.print("[MidToOutpostAuto] Complete!"));
  }

  // ==================== Time Management Helpers ====================

  private double getAutoTimeRemaining() {
    if (autoStartTimestamp <= 0.0) {
      return FieldConstants.AUTO_DURATION;
    }
    double elapsed = Timer.getFPGATimestamp() - autoStartTimestamp;
    return Math.max(0.0, FieldConstants.AUTO_DURATION - elapsed);
  }

  // ==================== Path Following Helpers ====================

  /**
   * Load a pre-drawn PathPlanner path by name and return a deferred follow command. PathPlanner
   * handles alliance flipping automatically based on the {@code shouldFlipPath} supplier configured
   * in {@code AutoBuilder.configure()}.
   *
   * @param pathName The path file name (without extension) in deploy/pathplanner/paths/
   * @return A deferred command that follows the path
   */
  private Command followPath(String pathName) {
    return Commands.defer(
        () -> {
          try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.followPath(path);
          } catch (Exception e) {
            System.err.println("[MidToOutpostAuto] Failed to load path: " + pathName);
            e.printStackTrace();
            return Commands.print("[MidToOutpostAuto] ERROR: Path not found: " + pathName);
          }
        },
        Set.of(drive));
  }
}
