// Copyright (c) 2026 FRC Team 0 (Amped)
// Hardcoded fallback autonomous routines — one per lane (UPPER, CENTER, LOWER)

package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.auto.dashboard.FieldConstants;
import frc.robot.auto.dashboard.FieldConstants.IntakeLocation;
import frc.robot.auto.dashboard.FieldConstants.ScoringWaypoint;
import frc.robot.auto.dashboard.FieldConstants.StartPose;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Hardcoded fallback autonomous routines for when the dashboard auto system isn't configured or as
 * a reliable alternative. Three lane-based autos are provided: UPPER, CENTER, and LOWER. Each auto
 * follows a simple pattern: set start pose → score preload → (intake → score) cycle.
 *
 * <p>Easy to configure: adjust the constants at the top of each lane configuration, or add/remove
 * cycles by editing the {@code buildLaneAuto} call.
 *
 * <p>Selection: the main auto chooser has a "Hardcoded Auto" entry that defers to a sub-chooser
 * where you pick which lane (UPPER / CENTER / LOWER).
 */
public class HardcodedAutos {

  // ===================================================================
  // CONFIGURATION — Edit these to change what each hardcoded auto does.
  // Each lane defines: start pose, scoring waypoint, intake location,
  // and how many intake→score cycles to attempt after the preload.
  // ===================================================================

  // --- UPPER LANE ---
  private static final StartPose UPPER_START = StartPose.UPPER;
  private static final ScoringWaypoint UPPER_SCORE = ScoringWaypoint.HUB_UPPER;
  private static final IntakeLocation UPPER_INTAKE = IntakeLocation.NEUTRAL_ZONE_UPPER;
  private static final int UPPER_CYCLES = 1;

  // --- CENTER LANE ---
  private static final StartPose CENTER_START = StartPose.CENTER;
  private static final ScoringWaypoint CENTER_SCORE = ScoringWaypoint.HUB_CENTER;
  private static final IntakeLocation CENTER_INTAKE = IntakeLocation.NEUTRAL_ZONE_UPPER;
  private static final int CENTER_CYCLES = 1;

  // --- LOWER LANE ---
  private static final StartPose LOWER_START = StartPose.LOWER;
  private static final ScoringWaypoint LOWER_SCORE = ScoringWaypoint.HUB_LOWER;
  private static final IntakeLocation LOWER_INTAKE = IntakeLocation.NEUTRAL_ZONE_LOWER;
  private static final int LOWER_CYCLES = 1;

  // ===================================================================

  private final DriveSwerveDrivetrain drive;
  private final Superstructure superstructure;

  @SuppressWarnings("unused")
  private final RobotState robotState;

  private final LoggedDashboardChooser<String> laneChooser;

  public HardcodedAutos(
      DriveSwerveDrivetrain drive, Superstructure superstructure, RobotState robotState) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.robotState = robotState;

    // Sub-chooser: pick which hardcoded auto lane
    laneChooser = new LoggedDashboardChooser<>("Auto/Hardcoded Lane");
    laneChooser.addDefaultOption("Upper Lane", "UPPER");
    laneChooser.addOption("Center Lane", "CENTER");
    laneChooser.addOption("Lower Lane", "LOWER");
  }

  /**
   * Get the deferred command that reads the lane chooser at auto init time and returns the
   * corresponding hardcoded auto command.
   */
  public Command getCommand() {
    return Commands.defer(
        () -> {
          String selected = laneChooser.get();
          if (selected == null) selected = "UPPER";
          switch (selected) {
            case "CENTER":
              return buildLaneAuto(
                  "Center", CENTER_START, CENTER_SCORE, CENTER_INTAKE, CENTER_CYCLES);
            case "LOWER":
              return buildLaneAuto("Lower", LOWER_START, LOWER_SCORE, LOWER_INTAKE, LOWER_CYCLES);
            case "UPPER":
            default:
              return buildLaneAuto("Upper", UPPER_START, UPPER_SCORE, UPPER_INTAKE, UPPER_CYCLES);
          }
        },
        Set.of(drive));
  }

  // ===== Auto Builder =====

  /**
   * Build a complete lane auto: set start pose → score preload → N × (intake → score).
   *
   * @param label Human-readable lane name for logging
   * @param start Where the robot starts
   * @param scoreWaypoint Where to shoot FUEL
   * @param intakeLocation Where to collect FUEL
   * @param cycles Number of intake→score cycles after the preload
   */
  private Command buildLaneAuto(
      String label,
      StartPose start,
      ScoringWaypoint scoreWaypoint,
      IntakeLocation intakeLocation,
      int cycles) {

    Command auto =
        Commands.sequence(
            // Reset superstructure state (in case previous auto ended in CLIMB_MODE)
            Commands.runOnce(() -> superstructure.forceIdleState()),

            // Log start
            Commands.print("[HardcodedAuto] Running " + label + " lane auto"),

            // 1. Set start pose
            Commands.runOnce(() -> drive.setPose(start.getPose()), drive),

            // 2. Score preload — aim while driving to scoring waypoint, then shoot
            Commands.print("[HardcodedAuto] Scoring preload"),
            superstructure.onlyAiming(),
            pathfindTo(scoreWaypoint.toPose()),
            buildScoringSequence());

    // 3. Intake→Score cycles
    for (int i = 0; i < cycles; i++) {
      final int cycle = i + 1;
      auto =
          auto.andThen(
              Commands.print("[HardcodedAuto] Cycle " + cycle + "/" + cycles + " — intaking"),
              // Start intaking (instant), then drive to intake location
              superstructure.onlyIntake(),
              pathfindTo(intakeLocation.getPose()),
              Commands.waitSeconds(0.3),
              Commands.print("[HardcodedAuto] Cycle " + cycle + "/" + cycles + " — scoring"),
              // Start aiming while driving to score
              superstructure.onlyAiming(),
              pathfindTo(scoreWaypoint.toPose()),
              buildScoringSequence());
    }

    // 4. End in idle
    auto = auto.andThen(superstructure.idle(), Commands.print("[HardcodedAuto] Complete!"));

    return auto.withName("HardcodedAuto_" + label);
  }

  // ===== Shared Helpers =====

  /**
   * Aim at HUB → settle → fire → brief delay → idle.
   *
   * <p>With the 254-style architecture, state commands are instant and periodic() continuously
   * applies them. No deadline/withTimeout hacks needed.
   */
  private Command buildScoringSequence() {
    return Commands.sequence(
        // Aim at hub (instant — periodic applies continuously)
        superstructure.onlyAiming(),
        // Let turret/hood/shooter settle
        Commands.waitSeconds(0.2),
        // Fire (instant — periodic starts feeding)
        superstructure.onlyShooting(),
        // Wait for FUEL to leave
        Commands.waitSeconds(0.15),
        // Return to idle (instant)
        superstructure.idle());
  }

  /**
   * Deferred pathfind command with trench-aware heading snapping. Mirrors the same logic from
   * {@link frc.robot.auto.dashboard.AutoCommandBuilder}.
   */
  private Command pathfindTo(Pose2d target) {
    return Commands.defer(
        () -> AutoBuilder.pathfindToPose(trenchAwarePose(target), drive.getPathConstraints(), 0.0),
        Set.of(drive));
  }

  /** Snap heading to cardinal if near a trench. */
  private static Pose2d trenchAwarePose(Pose2d pose) {
    if (FieldConstants.isNearTrench(pose.getTranslation())) {
      Rotation2d snapped = FieldConstants.snapToCardinal(pose.getRotation());
      return new Pose2d(pose.getTranslation(), snapped);
    }
    return pose;
  }
}
