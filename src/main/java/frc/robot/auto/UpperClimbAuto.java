// Copyright (c) 2026 FRC Team 10922 (Amped)
// Hardcoded upper-lane autonomous with L1 climb finish using pre-drawn PathPlanner paths.

package frc.robot.auto;

import static frc.robot.auto.dashboard.AutoTuning.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.auto.dashboard.FieldConstants;
import frc.robot.auto.dashboard.FieldConstants.ClimbPose;
import frc.robot.auto.dashboard.FieldConstants.StartPose;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.climb.ClimbState;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * Hardcoded upper-lane autonomous with L1 climb using pre-drawn PathPlanner paths.
 *
 * <p>All driving legs use {@link PathPlannerPath#fromPathFile} so heading profiles (especially
 * through the upper trench) are designed in the PathPlanner GUI rather than computed at runtime.
 * This eliminates trench collision issues caused by pathfindToPose generating unpredictable
 * rotation trajectories.
 *
 * <h3>Required Paths (draw/edit in PathPlanner GUI)</h3>
 *
 * <ul>
 *   <li>{@code "Upper Start To Hub"} — Start pose → HUB_UPPER scoring position
 *   <li>{@code "Upper Hub To Neutral 1"} — HUB_UPPER → NEUTRAL_ZONE_UPPER (through upper trench, 0°
 *       heading)
 *   <li>{@code "Upper Hub To Neutral 2"} — HUB_UPPER → NEUTRAL_ZONE_UPPER deeper (through upper
 *       trench, 0° heading, alternate cycle 2 path)
 *   <li>{@code "Upper Neutral To Hub"} — NEUTRAL_ZONE_UPPER → HUB_UPPER (through upper trench, 0°
 *       heading)
 *   <li>{@code "Upper Neutral To Climb"} — NEUTRAL_ZONE_UPPER → Climb standoff (through upper
 *       trench, SWD leg)
 * </ul>
 *
 * <h3>Sequence</h3>
 *
 * <ol>
 *   <li>Seed pose at {@link StartPose#UPPER} (alliance-corrected)
 *   <li>Follow "Upper Start To Hub", stop-and-shoot preload
 *   <li>Follow "Upper Hub To Neutral 1", intake FUEL (Cycle 1)
 *   <li>Follow "Upper Neutral To Hub", stop-and-shoot (Cycle 1)
 *   <li>Follow "Upper Hub To Neutral 2", intake FUEL (Cycle 2)
 *   <li>Follow "Upper Neutral To Climb" with shoot-while-driving (feed only in alliance zone)
 *   <li>Straight-line approach into tower, extend + retract L1 climb
 *   <li>Cleanup and idle
 * </ol>
 *
 * <p>This is a fully hardcoded auto — no dashboard timing system needed. Select "Upper Climb Auto"
 * in the auto chooser.
 */
public class UpperClimbAuto {

  // ==================== Path Names (must match files in deploy/pathplanner/paths/) ==============

  /** Path: Start position → HUB_UPPER scoring position. */
  private static final String PATH_START_TO_HUB = "Upper Start To Hub";

  /** Path: HUB_UPPER → NEUTRAL_ZONE_UPPER (outbound through upper trench). */
  private static final String PATH_HUB_TO_NEUTRAL = "Upper Hub To Neutral 1";

  /** Alternate HUB → NEUTRAL path for the second intake pass (deeper endpoint). */
  private static final String PATH_HUB_TO_NEUTRAL_ALT = "Upper Hub To Neutral 2";

  /** Path: NEUTRAL_ZONE_UPPER → HUB_UPPER (inbound through upper trench). */
  private static final String PATH_NEUTRAL_TO_HUB = "Upper Neutral To Hub";

  /** Path: NEUTRAL_ZONE_UPPER → Climb standoff (SWD leg through upper trench to tower). */
  private static final String PATH_NEUTRAL_TO_CLIMB = "Upper Neutral To Climb";

  // ==================== Route Constants ====================

  /** Starting pose for this auto. Also used by RobotContainer.getAutoStartingPose(). */
  public static final StartPose START_POSE = StartPose.UPPER;

  /** Which TOWER face to approach for climb. */
  private static final ClimbPose CLIMB_POSE = ClimbPose.OUTPOST_SIDE;

  // ==================== Timing Constants ====================

  /** Dwell at the intake location to collect FUEL before leaving (seconds). */
  private static final double INTAKE_DWELL_SECONDS = 0.0;

  /** Feed duration for stop-and-shoot cycles (seconds). */
  private static final double SHOOT_DURATION_SECONDS = 1.7;

  // ==================== Dependencies ====================

  private final DriveSwerveDrivetrain drive;
  private final Superstructure superstructure;
  private final ClimbSubsystem climb;

  // ==================== Runtime State ====================

  private double autoStartTimestamp = 0.0;

  // ==================== Constructor ====================

  public UpperClimbAuto(
      DriveSwerveDrivetrain drive, Superstructure superstructure, ClimbSubsystem climb) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.climb = climb;
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
                buildNeutralZoneCycle("Cycle 1"),
                buildIntakeThenSWDToClimb(),
                buildClimbSequence(),
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
   * Phase 2: Follow path to neutral zone, collect FUEL, follow path back to HUB_UPPER,
   * stop-and-shoot.
   *
   * @param label Human-readable cycle label for logging (e.g. "Cycle 1")
   */
  private Command buildNeutralZoneCycle(String label) {
    return Commands.sequence(
            // Intake leg
            Commands.print("[UpperClimbAuto] " + label + " — intaking at NEUTRAL_ZONE_UPPER"),
            Commands.deadline(followPath(PATH_HUB_TO_NEUTRAL), zoneAwareIntake()),
            Commands.waitSeconds(INTAKE_DWELL_SECONDS),
            // Scoring leg
            Commands.print("[UpperClimbAuto] " + label + " — scoring at HUB_UPPER"),
            Commands.deadline(followPath(PATH_NEUTRAL_TO_HUB), zoneAwareIntake()),
            buildStopAndShootSequence())
        .withName("NeutralZoneCycle_" + label.replace(" ", ""));
  }

  /**
   * Phase 3: Follow alternate path to the neutral zone to intake FUEL, then follow drawn path
   * directly to the climb standoff with shoot-while-driving. Climb arms extend in parallel
   * mid-transit (once intake is stowed). Feeds only while inside the alliance zone.
   *
   * <p>Uses the same pattern as {@code AutoCommandBuilder.buildDriveAndClimb}: the SWD path and
   * climb extend run in {@code Commands.parallel()}, so both must complete before the PID approach
   * begins. If the path finishes before the extend, the robot waits at the standoff for the arms.
   */
  private Command buildIntakeThenSWDToClimb() {
    return Commands.sequence(
            // Intake leg (use alternate pre-drawn intake path on the second pass)
            Commands.print("[UpperClimbAuto] Cycle 2 — intaking at NEUTRAL_ZONE_UPPER (alt path)"),
            Commands.deadline(followPath(PATH_HUB_TO_NEUTRAL_ALT), zoneAwareIntake()),
            Commands.waitSeconds(INTAKE_DWELL_SECONDS),
            // SWD leg — shoot on the move to climb standoff with climb extending in parallel
            Commands.print("[UpperClimbAuto] Cycle 2 — SWD to climb standoff"),
            Commands.parallel(
                // Group 1: Drive to standoff with zone-aware SWD
                Commands.deadline(
                    followPath(PATH_NEUTRAL_TO_CLIMB),
                    Commands.run(
                        () -> {
                          boolean outsideAimingZone = isOutsideAimingZone();
                          if (outsideAimingZone) {
                            superstructure.forceWantedState(SuperstructureState.ONLY_INTAKE);
                            superstructure.setFeedingRequested(false);
                          } else {
                            superstructure.forceWantedState(
                                SuperstructureState.AIMING_WHILE_INTAKING);
                            superstructure.setFeedingRequested(isInAllianceZone());
                          }
                          Logger.recordOutput("UpperClimbAuto/SWD/Active", true);
                          Logger.recordOutput("UpperClimbAuto/SWD/Feeding", isInAllianceZone());
                        },
                        superstructure)),
                // Group 2: Extend climb arms once intake is stowed (mid-transit)
                Commands.sequence(
                    Commands.waitUntil(() -> superstructure.isIntakeStowed()),
                    Commands.print("[UpperClimbAuto] Intake stowed — extending climb"),
                    climb.setStateCommand(ClimbState.EXTEND_L1_AUTO))),
            // Both path and extend done — idle superstructure for final approach
            Commands.runOnce(
                () -> {
                  superstructure.setFeedingRequested(false);
                  superstructure.forceWantedState(SuperstructureState.IDLE);
                  Logger.recordOutput("UpperClimbAuto/SWD/Active", false);
                }))
        .withName("IntakeThenSWD_ToClimb");
  }

  /**
   * Phase 4: Arms extended, robot at standoff — PID straight-line approach into the tower, then
   * retract to pull the robot up.
   */
  private Command buildClimbSequence() {
    return Commands.sequence(
            Commands.print("[UpperClimbAuto] At standoff — approaching tower"),
            // Straight-line approach into tower
            buildDriveToPose(
                CLIMB_POSE.getPose(),
                Constants.AutoConstants.CLIMB_APPROACH_MAX_VELOCITY_MPS,
                Constants.AutoConstants.CLIMB_APPROACH_TOLERANCE_M,
                Constants.AutoConstants.CLIMB_APPROACH_THETA_TOLERANCE_RAD),
            Commands.runOnce(() -> drive.stop(), drive),
            // Retract to pull robot up
            Commands.print("[UpperClimbAuto] Reached tower — retracting"),
            climb.setStateCommand(ClimbState.RETRACT_L1_AUTO))
        .withName("ClimbSequence");
  }

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
            System.err.println("[UpperClimbAuto] Failed to load path: " + pathName);
            e.printStackTrace();
            return Commands.print("[UpperClimbAuto] ERROR: Path not found: " + pathName);
          }
        },
        Set.of(drive));
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
