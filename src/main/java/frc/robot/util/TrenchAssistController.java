package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.auto.dashboard.FieldConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Teleop trench assist controller. Helps the driver pass through the 22.25-inch-tall trench tunnels
 * with two PID controllers:
 *
 * <ol>
 *   <li><b>Orientation PID</b> (heading error &rarr; omega) &mdash; snaps the chassis heading to
 *       the nearest cardinal (or horizontal when intake deployed).
 *   <li><b>Lateral PID</b> (Y-position error &rarr; vy) &mdash; centers the robot in the trench.
 * </ol>
 *
 * <h3>Driver-feel design</h3>
 *
 * <ul>
 *   <li><b>Blend, not add.</b> Both corrections use {@code MathUtil.interpolate} between the
 *       driver's raw input and the PID target, scaled by a proximity blend factor. This means the
 *       driver always has partial authority and the transition is smooth.
 *   <li><b>Soft activation ramps.</b> Speed and velocity-heading gates ramp linearly instead of
 *       switching on/off at a threshold, preventing jarring transitions.
 *   <li><b>Alliance-correct lateral.</b> The lateral correction is computed in blue-origin Y then
 *       sign-flipped for red alliance before being applied to field-relative vy.
 * </ul>
 */
public class TrenchAssistController {

  /** Immutable result returned by {@link #calculate}. */
  public record Result(double vx, double vy, double omega) {}

  // ----- PID Controllers -----
  private final PIDController orientationPID;
  private final PIDController lateralPID;

  // ----- State -----
  private final BooleanSupplier intakeDeployedSupplier;
  private boolean wasActive = false;

  /**
   * @param intakeDeployedSupplier supplies whether the intake is currently deployed. When true,
   *     heading snaps to 0/180 degrees only instead of all four cardinals.
   */
  public TrenchAssistController(BooleanSupplier intakeDeployedSupplier) {
    this.intakeDeployedSupplier = intakeDeployedSupplier;

    orientationPID =
        new PIDController(
            Constants.DriveConstants.TrenchAssist.ORIENTATION_KP,
            Constants.DriveConstants.TrenchAssist.ORIENTATION_KI,
            Constants.DriveConstants.TrenchAssist.ORIENTATION_KD);
    orientationPID.enableContinuousInput(-Math.PI, Math.PI);

    lateralPID =
        new PIDController(
            Constants.DriveConstants.TrenchAssist.LATERAL_KP,
            Constants.DriveConstants.TrenchAssist.LATERAL_KI,
            Constants.DriveConstants.TrenchAssist.LATERAL_KD);
  }

  /**
   * Compute trench-assisted drive outputs for this cycle.
   *
   * @param currentPose robot's current field-relative pose
   * @param vxFieldMps desired field-relative X velocity (m/s)
   * @param vyFieldMps desired field-relative Y velocity (m/s)
   * @param omegaRadPerSec desired rotational velocity from the driver (rad/s)
   * @return adjusted (vx, vy, omega) with trench assist blended in
   */
  public Result calculate(
      Pose2d currentPose, double vxFieldMps, double vyFieldMps, double omegaRadPerSec) {

    if (!Constants.DriveConstants.TrenchAssist.ENABLED) {
      logInactive();
      return new Result(vxFieldMps, vyFieldMps, omegaRadPerSec);
    }

    // --- Convert to blue-origin for geometry queries ---
    Translation2d bluePos = currentPose.getTranslation();
    Rotation2d robotHeading = currentPose.getRotation();
    boolean isRed = isRedAlliance();
    if (isRed) {
      bluePos = FieldConstants.flipTranslation(bluePos);
      robotHeading = robotHeading.plus(Rotation2d.fromDegrees(180));
    }

    double buffer = Constants.DriveConstants.TrenchAssist.APPROACH_BUFFER;

    // --- Proximity blend (0 outside buffer, ramps to MAX_BLEND inside trench) ---
    double proximityBlend =
        FieldConstants.getTrenchBlendFactor(
            bluePos, Constants.DriveConstants.TrenchAssist.MAX_BLEND_FACTOR, buffer);

    // --- Soft speed gate: ramps from 0 at standstill to 1 at MIN_SPEED ---
    double speed = Math.hypot(vxFieldMps, vyFieldMps);
    double speedGate =
        MathUtil.clamp(speed / Constants.DriveConstants.TrenchAssist.MIN_SPEED_MPS, 0.0, 1.0);

    // --- Soft heading gate: 1.0 when velocity is along X, ramps to 0 at MAX_HEADING_ERROR ---
    double headingGate = computeHeadingGate(vxFieldMps, vyFieldMps, speed);

    // --- Combined strength: all three gates multiply together ---
    double strength = proximityBlend * speedGate * headingGate;

    // --- PID reset on activation edges ---
    boolean active = strength > 1e-3;
    if (active && !wasActive) {
      orientationPID.reset();
      lateralPID.reset();
    }
    wasActive = active;

    if (!active) {
      logInactive();
      return new Result(vxFieldMps, vyFieldMps, omegaRadPerSec);
    }

    // ===== 1. Orientation: blend between driver omega and PID omega =====
    boolean intakeDeployed = intakeDeployedSupplier.getAsBoolean();
    Rotation2d targetHeading =
        intakeDeployed
            ? FieldConstants.snapToHorizontal(robotHeading)
            : FieldConstants.snapToCardinal(robotHeading);

    double rawPidOmega =
        orientationPID.calculate(robotHeading.getRadians(), targetHeading.getRadians());
    double clampedPidOmega =
        MathUtil.clamp(
            rawPidOmega,
            -Constants.DriveConstants.TrenchAssist.MAX_ORIENTATION_OMEGA_RAD_PER_SEC,
            Constants.DriveConstants.TrenchAssist.MAX_ORIENTATION_OMEGA_RAD_PER_SEC);

    // Blend: at strength=0 keep driver omega; at strength=1 use full PID omega
    double adjustedOmega = MathUtil.interpolate(omegaRadPerSec, clampedPidOmega, strength);

    // ===== 2. Lateral centering: blend between driver vy and PID vy =====
    double adjustedVy = vyFieldMps; // default: no change
    double trenchCenterY = FieldConstants.getNearestTrenchCenterY(bluePos, buffer);
    double lateralCorrection = 0.0;
    if (!Double.isNaN(trenchCenterY)) {
      // PID output: positive = robot should move in +Y (blue-origin)
      double rawLateralPid = lateralPID.calculate(bluePos.getY(), trenchCenterY);
      lateralCorrection =
          MathUtil.clamp(
              rawLateralPid,
              -Constants.DriveConstants.TrenchAssist.MAX_LATERAL_CORRECTION_MPS,
              Constants.DriveConstants.TrenchAssist.MAX_LATERAL_CORRECTION_MPS);

      // Flip for red alliance: blue +Y correction = red -vy correction
      if (isRed) {
        lateralCorrection = -lateralCorrection;
      }

      // Blend: at strength=0 keep driver vy; at strength=1 use (driver vy + full correction)
      adjustedVy = vyFieldMps + lateralCorrection * strength;
    }

    // ===== Logging =====
    Logger.recordOutput("Drive/TrenchAssist/Active", true);
    Logger.recordOutput("Drive/TrenchAssist/Strength", strength);
    Logger.recordOutput("Drive/TrenchAssist/ProximityBlend", proximityBlend);
    Logger.recordOutput("Drive/TrenchAssist/SpeedGate", speedGate);
    Logger.recordOutput("Drive/TrenchAssist/HeadingGate", headingGate);
    Logger.recordOutput("Drive/TrenchAssist/TargetHeadingDeg", targetHeading.getDegrees());
    Logger.recordOutput("Drive/TrenchAssist/OrientationOmega", clampedPidOmega);
    Logger.recordOutput("Drive/TrenchAssist/LateralCorrection", lateralCorrection);
    if (!Double.isNaN(trenchCenterY)) {
      Logger.recordOutput("Drive/TrenchAssist/TrenchCenterY", trenchCenterY);
      Logger.recordOutput("Drive/TrenchAssist/RobotBlueY", bluePos.getY());
    }

    return new Result(vxFieldMps, adjustedVy, adjustedOmega);
  }

  // ===== Helpers =====

  /**
   * Soft heading gate. Returns 1.0 when the velocity vector points along the X axis (through the
   * trench) and ramps linearly to 0.0 at MAX_HEADING_ERROR_DEG from the X axis.
   */
  private static double computeHeadingGate(double vx, double vy, double speed) {
    if (speed < 0.01) return 0.0; // avoid atan2 noise at near-zero speed
    double velDeg = Math.toDegrees(Math.atan2(vy, vx));
    double errorFrom0 = Math.abs(velDeg);
    double errorFrom180 = 180.0 - Math.abs(velDeg);
    double errorFromXAxis = Math.min(errorFrom0, errorFrom180);
    double maxError = Constants.DriveConstants.TrenchAssist.MAX_HEADING_ERROR_DEG;
    // 1.0 at 0° error, linearly ramps to 0.0 at maxError
    return MathUtil.clamp(1.0 - errorFromXAxis / maxError, 0.0, 1.0);
  }

  /**
   * Log inactive state. Only logs the Active=false flag to avoid unnecessary NT overhead every 20ms
   * cycle when the robot is nowhere near a trench. The numeric fields are only interesting when the
   * assist is active or ramping.
   */
  private void logInactive() {
    Logger.recordOutput("Drive/TrenchAssist/Active", false);
  }

  private static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }
}
