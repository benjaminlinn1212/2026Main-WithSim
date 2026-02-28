// Copyright (c) 2026 FRC Team 10922 (Amped)
// Sweep autonomous — pathfind to start of predrawn sweep path, follow its XY while spinning at
// constant angular velocity, optionally climb afterward

package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.auto.dashboard.AutoSettings;
import frc.robot.auto.dashboard.DashboardAutoManager;
import frc.robot.auto.dashboard.FieldConstants;
import frc.robot.auto.dashboard.FieldConstants.StartPose;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climb.ClimbState;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.Optional;
import java.util.Set;

/**
 * "Sweep" autonomous. Pathfinds to the start of a predrawn S-shape path, then follows the path's XY
 * translation while spinning at a constant angular velocity. Superstructure stays idle. If "Attempt
 * TOWER Climb" is checked, climbs afterward.
 *
 * <p>The constant spin is achieved by overriding PathPlanner's rotation target with a continuously
 * advancing angle. The override also handles trench snapping (since it replaces the global override
 * from RobotContainer), ensuring the robot locks heading when passing through trenches during both
 * the sweep and the climb pathfinding phases.
 *
 * <p>Draw/edit the sweep pattern in PathPlanner: {@code deploy/pathplanner/paths/Sweep Path.path}.
 */
public class SweepAuto {

  private static final String UPPER_SWEEP_PATH_NAME = "Upper Sweep Path";
  private static final String LOWER_SWEEP_PATH_NAME = "Lower Sweep Path";

  /** Constant angular velocity magnitude during the sweep (rad/s). */
  private static final double SWEEP_OMEGA_RAD_PER_SEC = Math.PI;

  private final DriveSwerveDrivetrain drive;
  private final Superstructure superstructure;
  private final ClimbSubsystem climb;
  private final DashboardAutoManager dashboardAutoManager;

  /** When true, the rotation override returns a spinning target. When false, returns empty. */
  private boolean spinActive = false;

  private final Timer spinTimer = new Timer();
  private double startHeadingRad = 0.0;
  /** Signed omega for the current sweep — set at auto init based on start pose. */
  private double sweepOmega = SWEEP_OMEGA_RAD_PER_SEC;

  public SweepAuto(
      DriveSwerveDrivetrain drive,
      Superstructure superstructure,
      ClimbSubsystem climb,
      DashboardAutoManager dashboardAutoManager) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.climb = climb;
    this.dashboardAutoManager = dashboardAutoManager;
  }

  public Command getCommand() {
    return Commands.defer(this::buildSweepAuto, Set.of(drive));
  }

  /**
   * Returns the rotation override supplier. Trench snapping is ALWAYS active (since this override
   * replaces the global one from RobotContainer). When spinning, returns a continuously advancing
   * angle unless near a trench. When not spinning (e.g. climb pathfinding), still snaps to cardinal
   * near trenches so the robot doesn't get stuck.
   */
  private Optional<Rotation2d> getRotationOverride() {
    // Near a trench — always lock heading to cardinal so we fit through the 22.25in ceiling
    if (FieldConstants.isNearTrench(drive.getPose().getTranslation())) {
      return Optional.of(
          superstructure.isIntakeDeployed()
              ? FieldConstants.snapToHorizontal(drive.getPose().getRotation())
              : FieldConstants.snapToCardinal(drive.getPose().getRotation()));
    }
    if (!spinActive) {
      return Optional.empty();
    }
    double targetRad = startHeadingRad + sweepOmega * spinTimer.get();
    return Optional.of(Rotation2d.fromRadians(targetRad));
  }

  private Command buildSweepAuto() {
    AutoSettings settings = dashboardAutoManager.getSettings();
    StartPose startPose = settings.getStartPose();

    // Pick path and spin direction based on start pose
    boolean isUpper = (startPose == StartPose.UPPER);
    String pathName = isUpper ? UPPER_SWEEP_PATH_NAME : LOWER_SWEEP_PATH_NAME;
    sweepOmega = isUpper ? SWEEP_OMEGA_RAD_PER_SEC : -SWEEP_OMEGA_RAD_PER_SEC;

    PathPlannerPath sweepPath;
    try {
      sweepPath = PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      return Commands.print("[SweepAuto] ERROR: Could not load path '" + pathName + "'");
    }

    // Install our override (coexists with trench-snap — returns empty when not spinning)
    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationOverride);

    Command sweepSequence =
        Commands.sequence(
            Commands.runOnce(() -> superstructure.forceIdleState()),
            // Pathfind to path start (spinActive=false → normal rotation)
            AutoBuilder.pathfindToPose(
                sweepPath.getStartingHolonomicPose().orElse(drive.getPose()),
                drive.getPathConstraints(),
                0.0),
            // Start spinning, then follow the predrawn path
            Commands.runOnce(
                () -> {
                  startHeadingRad = drive.getPose().getRotation().getRadians();
                  spinTimer.restart();
                  spinActive = true;
                }),
            AutoBuilder.followPath(sweepPath)
                .finallyDo(
                    (interrupted) -> {
                      spinActive = false;
                      spinTimer.stop();
                    }));

    if (settings.shouldAttemptClimb()) {
      Pose2d climbTarget = settings.getClimbPose().getPose();
      sweepSequence =
          sweepSequence.andThen(
              superstructure.idle(),
              Commands.waitUntil(() -> superstructure.isIntakeStowed()),
              pathfindThenDriveStraight(climbTarget),
              climb.setStateCommand(ClimbState.EXTEND_L1_AUTO),
              climb.setStateCommand(ClimbState.RETRACT_L1_AUTO));
    }

    return sweepSequence.withName("SweepAuto");
  }

  // ===== Climb Approach Helpers (mirrors AutoCommandBuilder) =====

  /**
   * Pathfind to a standoff point near the target, then PID drive-to-pose in a straight line. This
   * ensures the climb hook approaches the tower dead-straight for a clean latch.
   */
  private Command pathfindThenDriveStraight(Pose2d target) {
    double approachDist = Constants.AutoConstants.CLIMB_APPROACH_DISTANCE_M;
    Translation2d standoffOffset =
        new Translation2d(approachDist, target.getRotation().plus(Rotation2d.k180deg));
    Pose2d standoffPose =
        new Pose2d(target.getTranslation().plus(standoffOffset), target.getRotation());

    double maxVel = Constants.AutoConstants.CLIMB_APPROACH_MAX_VELOCITY_MPS;
    double tolerance = Constants.AutoConstants.CLIMB_APPROACH_TOLERANCE_M;
    double thetaTolerance = Constants.AutoConstants.CLIMB_APPROACH_THETA_TOLERANCE_RAD;

    return Commands.sequence(
        Commands.defer(
            () -> AutoBuilder.pathfindToPose(standoffPose, drive.getPathConstraints(), 0.0),
            Set.of(drive)),
        buildDriveToPose(target, maxVel, tolerance, thetaTolerance),
        Commands.runOnce(() -> drive.stop(), drive));
  }

  /**
   * 2910-style PID drive-to-pose. Linear PID drives distance error to zero while a heading PID
   * holds the target angle. Static friction feedforward prevents stalling at small distances.
   */
  private Command buildDriveToPose(
      Pose2d target, double maxVelocity, double driveTolerance, double thetaTolerance) {
    PIDController driveController =
        new PIDController(
            Constants.AutoConstants.DRIVE_TO_POSE_KP, 0, Constants.AutoConstants.DRIVE_TO_POSE_KD);
    PIDController thetaController =
        new PIDController(Constants.AutoConstants.DRIVE_TO_POSE_THETA_KP, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    double frictionFF = Constants.AutoConstants.DRIVE_TO_POSE_FRICTION_FF * maxVelocity;

    return Commands.run(
            () -> {
              Pose2d currentPose = drive.getPose();
              Translation2d translationToTarget =
                  target.getTranslation().minus(currentPose.getTranslation());
              double linearDistance = translationToTarget.getNorm();
              Rotation2d directionOfTravel = translationToTarget.getAngle();

              double friction = (linearDistance >= Units.inchesToMeters(0.5)) ? frictionFF : 0.0;
              double velocityOutput =
                  Math.min(
                      Math.abs(driveController.calculate(linearDistance, 0.0)) + friction,
                      maxVelocity);

              double vx = velocityOutput * directionOfTravel.getCos();
              double vy = velocityOutput * directionOfTravel.getSin();
              double omega =
                  thetaController.calculate(
                      currentPose.getRotation().getRadians(), target.getRotation().getRadians());

              drive.driveFieldRelative(vx, vy, omega);
            },
            drive)
        .until(
            () -> {
              Pose2d currentPose = drive.getPose();
              double distError = currentPose.getTranslation().getDistance(target.getTranslation());
              double headingError =
                  Math.abs(
                      MathUtil.angleModulus(
                          currentPose.getRotation().getRadians()
                              - target.getRotation().getRadians()));
              return distError <= driveTolerance && headingError <= thetaTolerance;
            });
  }
}
