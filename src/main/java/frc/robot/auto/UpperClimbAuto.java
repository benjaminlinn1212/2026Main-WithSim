// Copyright (c) 2026 FRC Team 10922 (Amped)
// Hardcoded upper-lane autonomous with L1 climb finish using pre-drawn PathPlanner paths.

package frc.robot.auto;

import static frc.robot.auto.dashboard.AutoTuning.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.dashboard.FieldConstants;
import frc.robot.auto.dashboard.FieldConstants.ClimbPose;
import frc.robot.auto.dashboard.FieldConstants.StartPose;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Hardcoded upper-lane auto with L1 climb. Uses pre-drawn PathPlanner paths for trench traversal.
 * Sequence: seed pose -> start-to-hub -> shoot preload -> hub-to-climb (intake + SWD in one path)
 * -> extend/retract L1.
 */
public class UpperClimbAuto {

  // ==================== Path Names (must match files in deploy/pathplanner/paths/) ==============

  /** Path: Start position → HUB_UPPER scoring position. */
  private static final String PATH_START_TO_HUB = "Upper Start To Hub";

  /** Path: HUB_UPPER → neutral zone (intake) → back to HUB (stop-and-shoot). Round trip. */
  private static final String PATH_HUB_ROUND_TRIP = "Upper Hub";

  // ==================== Route Constants ====================

  /** Starting pose for this auto. Also used by RobotContainer.getAutoStartingPose(). */
  public static final StartPose START_POSE = StartPose.UPPER;

  // ==================== Timing Constants ====================

  /** Feed duration for stop-and-shoot cycles (seconds). */
  private static final double SHOOT_DURATION_SECONDS = 2.0;

  // ==================== Dependencies ====================

  private final DriveSwerveDrivetrain drive;
  private final Superstructure superstructure;
  private final ClimbSubsystem climb;
  private final Supplier<ClimbPose> climbPoseSupplier;

  // ==================== Runtime State ====================

  private double autoStartTimestamp = 0.0;

  // ==================== Constructor ====================

  public UpperClimbAuto(
      DriveSwerveDrivetrain drive,
      Superstructure superstructure,
      ClimbSubsystem climb,
      Supplier<ClimbPose> climbPoseSupplier) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.climb = climb;
    this.climbPoseSupplier = climbPoseSupplier;
  }

  // ==================== Public Entry Points ====================

  /** Deferred command — safe for auto choosers. */
  public Command getCommand() {
    return Commands.defer(this::buildCommand, Set.of(drive));
  }

  /** Build the full auto command graph. */
  public Command buildCommand() {
    return Commands.deadline(
            Commands.sequence(
                buildInit(),
                buildScorePreload(),
                buildIntakeThenStopAndShoot(),
                // buildSWDToClimb(),
                // buildClimbSequence(),
                buildCleanup()),
            Commands.run(
                () ->
                    Logger.recordOutput(
                        "UpperClimbAuto/AutoTimeRemaining", getAutoTimeRemaining())))
        .withName("UpperClimbAuto");
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
        Commands.print("[UpperClimbAuto] Starting — Upper start, L1 climb finish"));
  }

  /** Phase 1: Follow path to HUB_UPPER and stop-and-shoot the preloaded FUEL. */
  private Command buildScorePreload() {
    return Commands.sequence(
            Commands.print("[UpperClimbAuto] Scoring preload"),
            Commands.deadline(followPath(PATH_START_TO_HUB), zoneAwareIntake()),
            buildStopAndShootSequence())
        .withName("ScorePreload");
  }

  /**
   * Phase 2 (simplified): Follow hub → neutral → hub round trip with intake, then stop-and-shoot.
   * Only 2 total paths in this auto (start→hub from preload, hub round trip here).
   */
  private Command buildIntakeThenStopAndShoot() {
    return Commands.sequence(
            Commands.print("[UpperClimbAuto] Hub round trip (intake)"),
            // Round trip: intake while driving hub → neutral → hub
            Commands.deadline(
                followPath(PATH_HUB_ROUND_TRIP),
                Commands.run(
                    () -> superstructure.forceWantedState(SuperstructureState.ONLY_INTAKE),
                    superstructure)),
            // Stop and shoot
            buildStopAndShootSequence(),
            // Wait 1 second then stow intake (idle)
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> superstructure.forceWantedState(SuperstructureState.IDLE)))
        .withName("IntakeThenStopAndShoot");
  }

  /*
   * Phase 2 (original — SWD to climb, commented out):
   *
   * private Command buildSWDToClimb() {
   *   return Commands.sequence(
   *           Commands.print("[UpperClimbAuto] Hub to climb (intake + SWD)"),
   *           Commands.parallel(
   *               Commands.deadline(
   *                   followPath(PATH_HUB_TO_CLIMB),
   *                   Commands.run(
   *                       () -> {
   *                         boolean outsideAimingZone = isOutsideAimingZone();
   *                         if (outsideAimingZone) {
   *                           superstructure.forceWantedState(SuperstructureState.ONLY_INTAKE);
   *                           superstructure.setFeedingRequested(false);
   *                         } else {
   *                           superstructure.forceWantedState(SuperstructureState.ONLY_AIMING);
   *                           superstructure.setFeedingRequested(isInAllianceZone());
   *                         }
   *                       },
   *                       superstructure)),
   *               Commands.defer(
   *                   () -> {
   *                     final int[] allianceZoneEntryCount = {isInAllianceZone() ? 1 : 0};
   *                     final boolean[] wasInAllianceZone = {isInAllianceZone()};
   *                     return Commands.sequence(
   *                         Commands.waitUntil(
   *                             () -> {
   *                               boolean inZone = isInAllianceZone();
   *                               if (inZone && !wasInAllianceZone[0]) {
   *                                 allianceZoneEntryCount[0]++;
   *                               }
   *                               wasInAllianceZone[0] = inZone;
   *                               return allianceZoneEntryCount[0] >= 2;
   *                             }),
   *                         climb.setStateCommand(ClimbState.EXTEND_L1_AUTO));
   *                   },
   *                   Set.of())),
   *           Commands.runOnce(
   *               () -> {
   *                 superstructure.setFeedingRequested(false);
   *                 superstructure.forceWantedState(SuperstructureState.IDLE);
   *               }))
   *       .withName("SWD_ToClimb");
   * }
   */

  /*
   * Phase 3 (original — climb sequence, commented out):
   *
   * private Command buildClimbSequence() {
   *   ClimbPose climbPose = climbPoseSupplier.get();
   *   return Commands.sequence(
   *           Commands.print("[UpperClimbAuto] At standoff — approaching tower"),
   *           buildDriveToPose(
   *               climbPose.getPose(),
   *               Constants.AutoConstants.CLIMB_APPROACH_MAX_VELOCITY_MPS,
   *               Constants.AutoConstants.CLIMB_APPROACH_TOLERANCE_M,
   *               Constants.AutoConstants.CLIMB_APPROACH_THETA_TOLERANCE_RAD),
   *           Commands.runOnce(() -> drive.stop(), drive),
   *           Commands.print("[UpperClimbAuto] Reached tower — retracting"),
   *           climb.setStateCommand(ClimbState.RETRACT_L1_AUTO))
   *       .withName("ClimbSequence");
   * }
   */

  /** Final phase: idle everything. */
  private Command buildCleanup() {
    return Commands.sequence(superstructure.idle(), Commands.print("[UpperClimbAuto] Complete!"));
  }

  // ==================== Reusable Command Helpers ====================

  /** Stop-and-shoot: aim at the HUB, feed for a fixed duration, return to zone-aware default. */
  private Command buildStopAndShootSequence() {
    return Commands.sequence(
        superstructure.aimingWhileIntaking(),
        Commands.runOnce(() -> superstructure.setFeedingRequested(true)),
        Commands.waitSeconds(SHOOT_DURATION_SECONDS),
        Commands.runOnce(() -> superstructure.setFeedingRequested(false)),
        zoneAwareDefaultState());
  }

  /**
   * Zone-aware intake: sets superstructure state based on field position every cycle. Outside
   * aiming zone → ONLY_INTAKE. Inside aiming zone → AIMING_WHILE_INTAKING.
   */
  private Command zoneAwareIntake() {
    return Commands.run(
            () -> {
              boolean outsideAimingZone = isOutsideAimingZone();
              superstructure.forceWantedState(
                  outsideAimingZone
                      ? SuperstructureState.ONLY_INTAKE
                      : SuperstructureState.AIMING_WHILE_INTAKING);
              Logger.recordOutput("UpperClimbAuto/ZoneAware/OutsideAimingZone", outsideAimingZone);
            },
            superstructure)
        .withName("ZoneAwareIntake");
  }

  /** Instant zone-aware state set (used after stop-and-shoot). */
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
   * Load a pre-drawn PathPlanner path by name and return a follow command. PathPlanner handles
   * alliance flipping automatically based on the {@code shouldFlipPath} supplier configured in
   * {@code AutoBuilder.configure()}.
   *
   * <p>Paths are loaded eagerly (not deferred) since {@code buildCommand()} is already called
   * inside a {@code Commands.defer()}. This avoids a single-cycle drive gap between sequential
   * path-follow commands that causes drivetrain stutter.
   *
   * @param pathName The path file name (without extension) in deploy/pathplanner/paths/
   * @return A command that follows the path
   */
  private Command followPath(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      System.err.println("[UpperClimbAuto] Failed to load path: " + pathName);
      e.printStackTrace();
      return Commands.print("[UpperClimbAuto] ERROR: Path not found: " + pathName);
    }
  }

  /** PID drive-to-pose for final approach (same as AutoCommandBuilder). */
  private Command buildDriveToPose(
      edu.wpi.first.math.geometry.Pose2d target,
      double maxVelocity,
      double driveTolerance,
      double thetaTolerance) {
    return Commands.sequence(
        Commands.runOnce(() -> drive.setDesiredPoseForDriveToPoint(target, maxVelocity), drive),
        Commands.waitUntil(() -> drive.isAtDriveToPointSetpoint(driveTolerance, thetaTolerance)));
  }
}
