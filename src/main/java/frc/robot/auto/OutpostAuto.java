// Copyright (c) 2026 FRC Team 10922 (Amped)
// Hardcoded outpost autonomous — see class Javadoc for the full sequence.

package frc.robot.auto;

import static frc.robot.auto.dashboard.AutoTuning.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.auto.dashboard.FieldConstants;
import frc.robot.auto.dashboard.FieldConstants.IntakeLocation;
import frc.robot.auto.dashboard.FieldConstants.ScoringWaypoint;
import frc.robot.auto.dashboard.FieldConstants.StartPose;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * Hardcoded outpost autonomous routine.
 *
 * <p>Starts from the LOWER position, cycles through the lower neutral zone for FUEL, then
 * shoot-while-drives to the OUTPOST and finishes with a prolonged stop-and-shoot while jiggling the
 * intake pivot to dislodge FUEL from the human player CHUTE.
 *
 * <h3>Sequence</h3>
 *
 * <ol>
 *   <li>Seed pose at {@link StartPose#LOWER} (alliance-corrected)
 *   <li>Drive to {@link ScoringWaypoint#HUB_LOWER}, stop-and-shoot preload
 *   <li>Drive to {@link IntakeLocation#NEUTRAL_ZONE_LOWER}, intake FUEL
 *   <li>Drive to {@link ScoringWaypoint#HUB_LOWER}, stop-and-shoot
 *   <li>Drive to {@link IntakeLocation#NEUTRAL_ZONE_LOWER}, intake FUEL again
 *   <li>Shoot-while-driving to {@link IntakeLocation#OUTPOST} (feed only in alliance zone)
 *   <li>At outpost: dwell → jiggle intake pivot → stop-and-shoot
 *   <li>Cleanup and idle
 * </ol>
 *
 * <p>This is a fully hardcoded auto — no dashboard timing system or start-pose selector needed.
 * Select "Hardcoded Auto" → "Outpost" in the auto choosers.
 */
public class OutpostAuto {

  // ==================== Route Constants ====================

  /** Starting pose for this auto. Also used by RobotContainer.getAutoStartingPose(). */
  public static final StartPose START_POSE = StartPose.LOWER;

  /** Where we score (stop-and-shoot waypoint). */
  private static final ScoringWaypoint SCORE_WAYPOINT = ScoringWaypoint.HUB_LOWER;

  /** Where we pick up FUEL in the neutral zone. */
  private static final IntakeLocation INTAKE_LOCATION = IntakeLocation.NEUTRAL_ZONE_LOWER;

  /** Final destination — the human player outpost. */
  private static final IntakeLocation OUTPOST_LOCATION = IntakeLocation.OUTPOST;

  // ==================== Timing Constants ====================

  /** Dwell at the intake location to collect FUEL before leaving (seconds). */
  private static final double INTAKE_DWELL_SECONDS = 0.0;

  /** Feed duration for normal stop-and-shoot cycles (seconds). */
  private static final double SHOOT_DURATION_SECONDS = 2.0;

  /**
   * Feed duration for the outpost stop-and-shoot (seconds). Much longer than a normal cycle because
   * we may accumulate extra FUEL from the SWD leg and the human player continues feeding FUEL
   * through the CHUTE.
   */
  private static final double OUTPOST_SHOOT_DURATION_SECONDS = 10.0;

  /** Dwell at outpost position before activating the jiggle (seconds). */
  private static final double OUTPOST_DWELL_BEFORE_JIGGLE_SECONDS = 1.5;

  // ==================== Dependencies ====================

  private final DriveSwerveDrivetrain drive;
  private final Superstructure superstructure;

  /**
   * PID controller for the trench rotation feedback override. Snaps heading to horizontal (0°/180°)
   * when near a trench — the intake is deployed so the robot is wider along 90°/270°.
   */
  @SuppressWarnings("resource")
  private final PIDController trenchRotationPID;

  // ==================== Constructor ====================

  public OutpostAuto(DriveSwerveDrivetrain drive, Superstructure superstructure) {
    this.drive = drive;
    this.superstructure = superstructure;

    this.trenchRotationPID =
        new PIDController(Constants.AutoConstants.PATH_FOLLOWING_ROTATION_KP, 0, 0);
    trenchRotationPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  // ==================== Public Entry Points ====================

  /** Deferred command — safe for auto choosers. Builds a fresh command graph each invocation. */
  public Command getCommand() {
    return Commands.defer(this::buildCommand, Set.of(drive));
  }

  /**
   * Build the full auto command graph. Public so {@code getHardcodedAutoCommand()} can call it
   * directly for a fresh instance each auto run (avoids the "already composed" error).
   */
  public Command buildCommand() {
    return Commands.sequence(
            buildInit(),
            buildScorePreload(),
            buildNeutralZoneCycle("Cycle 1"),
            buildNeutralZoneCycle("Cycle 2"),
            buildShootWhileDrivingToOutpost(),
            buildOutpostSequence(),
            buildCleanup())
        .withName("OutpostAuto");
  }

  // ==================== Phase Builders ====================

  /** Phase 0: Seed pose, install trench rotation override, reset superstructure. */
  private Command buildInit() {
    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(START_POSE.getPose())),
        Commands.runOnce(this::installTrenchRotationOverride),
        Commands.runOnce(superstructure::forceIdleState),
        Commands.print("[OutpostAuto] Starting — Lower start, outpost finish"));
  }

  /** Phase 1: Drive to HUB_LOWER and stop-and-shoot the preloaded FUEL. */
  private Command buildScorePreload() {
    return Commands.sequence(
            Commands.print("[OutpostAuto] Scoring preload at " + SCORE_WAYPOINT.name()),
            Commands.deadline(pathfindTo(SCORE_WAYPOINT.toPose()), zoneAwareIntake()),
            buildStopAndShootSequence())
        .withName("ScorePreload");
  }

  /**
   * Phase 2/3: Drive to the neutral zone, collect FUEL, drive back to HUB_LOWER, stop-and-shoot.
   *
   * @param label Human-readable cycle label for logging (e.g. "Cycle 1")
   */
  private Command buildNeutralZoneCycle(String label) {
    return Commands.sequence(
            // Intake leg
            Commands.print("[OutpostAuto] " + label + " — intaking at " + INTAKE_LOCATION.name()),
            Commands.deadline(pathfindTo(INTAKE_LOCATION.getPose()), zoneAwareIntake()),
            Commands.waitSeconds(INTAKE_DWELL_SECONDS),
            // Scoring leg
            Commands.print("[OutpostAuto] " + label + " — scoring at " + SCORE_WAYPOINT.name()),
            Commands.deadline(pathfindTo(SCORE_WAYPOINT.toPose()), zoneAwareIntake()),
            buildStopAndShootSequence())
        .withName("NeutralZoneCycle_" + label.replace(" ", ""));
  }

  /**
   * Phase 4: Shoot-while-driving from the neutral zone to the outpost. Feeds FUEL only while the
   * robot is inside the alliance zone (blueX ≤ {@link FieldConstants.Zone#ALLIANCE_ZONE maxX}) to
   * avoid wasting shots from beyond effective range.
   */
  private Command buildShootWhileDrivingToOutpost() {
    return Commands.sequence(
            Commands.print("[OutpostAuto] Shoot-while-driving to outpost"),
            Commands.deadline(
                pathfindTo(OUTPOST_LOCATION.getPose()),
                Commands.run(
                    () -> {
                      superstructure.forceWantedState(SuperstructureState.AIMING_WHILE_INTAKING);
                      boolean shouldFeed = isInAllianceZone();
                      superstructure.setFeedingRequested(shouldFeed);
                      Logger.recordOutput("OutpostAuto/SWD/Active", true);
                      Logger.recordOutput("OutpostAuto/SWD/Feeding", shouldFeed);
                    },
                    superstructure)),
            Commands.runOnce(
                () -> {
                  superstructure.setFeedingRequested(false);
                  Logger.recordOutput("OutpostAuto/SWD/Active", false);
                }))
        .withName("SWD_ToOutpost");
  }

  /**
   * Phase 5: At the outpost — enable outpost intake mode, dwell briefly so FUEL falls from the
   * CHUTE, then activate the intake pivot jiggle to dislodge stuck pieces while stop-and-shooting
   * with the extended outpost duration.
   */
  private Command buildOutpostSequence() {
    return Commands.sequence(
            Commands.print("[OutpostAuto] At outpost — enabling outpost intake mode"),
            Commands.runOnce(() -> superstructure.setIntakeOutpostMode(true)),
            Commands.waitSeconds(OUTPOST_DWELL_BEFORE_JIGGLE_SECONDS),
            Commands.print("[OutpostAuto] Activating outpost jiggle + stop-and-shoot"),
            Commands.runOnce(() -> superstructure.setIntakeOutpostJiggleMode(true)),
            buildStopAndShootSequence(OUTPOST_SHOOT_DURATION_SECONDS),
            Commands.runOnce(
                () -> {
                  superstructure.setIntakeOutpostJiggleMode(false);
                  superstructure.setIntakeOutpostMode(false);
                }))
        .withName("OutpostShoot");
  }

  /** Phase 6: Clear all overrides and return to idle. */
  private Command buildCleanup() {
    return Commands.sequence(
        Commands.runOnce(PPHolonomicDriveController::clearRotationFeedbackOverride),
        superstructure.idle(),
        Commands.print("[OutpostAuto] Complete!"));
  }

  // ==================== Reusable Command Helpers ====================

  /** Stop-and-shoot with the default {@link #SHOOT_DURATION_SECONDS}. */
  private Command buildStopAndShootSequence() {
    return buildStopAndShootSequence(SHOOT_DURATION_SECONDS);
  }

  /**
   * Stop-and-shoot: aim at the HUB, feed for a fixed duration, then return to zone-aware default
   * state. Uses a simple timed feed rather than current-based detection for reliability in a
   * hardcoded auto.
   *
   * @param durationSeconds How long to feed FUEL into the shooter
   */
  private Command buildStopAndShootSequence(double durationSeconds) {
    return Commands.sequence(
        superstructure.aimingWhileIntaking(),
        Commands.runOnce(() -> superstructure.setFeedingRequested(true)),
        Commands.waitSeconds(durationSeconds),
        Commands.runOnce(() -> superstructure.setFeedingRequested(false)),
        zoneAwareDefaultState());
  }

  /**
   * Continuous zone-aware intake. Every cycle, sets the superstructure state based on field
   * position:
   *
   * <ul>
   *   <li>Outside aiming zone (neutral zone) → {@link SuperstructureState#ONLY_INTAKE}
   *   <li>Inside aiming zone (alliance/HUB zone) → {@link
   *       SuperstructureState#AIMING_WHILE_INTAKING}
   * </ul>
   */
  private Command zoneAwareIntake() {
    return Commands.run(
            () -> {
              boolean outsideAimingZone = isOutsideAimingZone();
              superstructure.forceWantedState(
                  outsideAimingZone
                      ? SuperstructureState.ONLY_INTAKE
                      : SuperstructureState.AIMING_WHILE_INTAKING);
              Logger.recordOutput("OutpostAuto/ZoneAware/OutsideAimingZone", outsideAimingZone);
            },
            superstructure)
        .withName("ZoneAwareIntake");
  }

  /** Instant zone-aware state set (used after stop-and-shoot to pick the correct idle state). */
  private Command zoneAwareDefaultState() {
    return Commands.runOnce(
            () ->
                superstructure.forceWantedState(
                    isOutsideAimingZone()
                        ? SuperstructureState.ONLY_INTAKE
                        : SuperstructureState.AIMING_WHILE_INTAKING))
        .withName("ZoneAwareDefault");
  }

  // ==================== Field Zone Helpers ====================

  /** Convert the current robot pose to blue-alliance X coordinate. */
  private double getBlueX() {
    double rawX = drive.getPose().getX();
    return FieldConstants.isRedAlliance() ? FieldConstants.FIELD_LENGTH - rawX : rawX;
  }

  /** True when the robot is past the aiming zone boundary (in the neutral zone or beyond). */
  private boolean isOutsideAimingZone() {
    return getBlueX() > AIMING_ZONE_MAX_X;
  }

  /** True when the robot is inside the alliance zone (safe to feed during SWD). */
  private boolean isInAllianceZone() {
    return getBlueX() <= FieldConstants.Zone.ALLIANCE_ZONE.maxX;
  }

  // ==================== Pathfinding Helpers ====================

  /** Deferred pathfind-to-pose with trench-aware heading snapping. */
  private Command pathfindTo(Pose2d target) {
    return Commands.defer(
        () -> AutoBuilder.pathfindToPose(trenchAwarePose(target), drive.getPathConstraints(), 0.0),
        Set.of(drive));
  }

  /**
   * Snap the target heading to horizontal (0°/180°) if near a trench. The intake is deployed during
   * this auto, so the robot must pass through the trench aligned horizontally.
   */
  private Pose2d trenchAwarePose(Pose2d pose) {
    if (FieldConstants.isNearTrench(
        pose.getTranslation(), Constants.AutoConstants.TRENCH_APPROACH_BUFFER)) {
      return new Pose2d(pose.getTranslation(), FieldConstants.snapToHorizontal(pose.getRotation()));
    }
    return pose;
  }

  // ==================== Rotation Override ====================

  /**
   * Install the PathPlanner rotation feedback override for trench snapping. While near a trench,
   * the PID drives the heading toward the nearest horizontal angle (0° or 180°). Outside trench
   * zones, returns 0.0 (no override — PathPlanner uses its own rotation controller).
   */
  private void installTrenchRotationOverride() {
    trenchRotationPID.reset();
    PPHolonomicDriveController.overrideRotationFeedback(
        () -> {
          Pose2d pose = drive.getPose();
          if (FieldConstants.isNearTrench(
              pose.getTranslation(), Constants.AutoConstants.TRENCH_APPROACH_BUFFER)) {
            Rotation2d snapped = FieldConstants.snapToHorizontal(pose.getRotation());
            return trenchRotationPID.calculate(
                pose.getRotation().getRadians(), snapped.getRadians());
          }
          return 0.0;
        });
  }
}
