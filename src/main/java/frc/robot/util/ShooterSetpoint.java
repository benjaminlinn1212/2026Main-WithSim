package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.RobotState;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * ShooterSetpoint represents a complete shooting solution combining: - 6328's physics-based
 * calculation with interpolation maps and motion compensation - 254's clean structure with getters
 * and feedforward support - Current project's turret offset accounting for accurate aiming
 *
 * <p>This class calculates shooter wheel speed, turret angle, hood angle, and feedforward
 * velocities needed to make a shot, accounting for robot motion, gravity, and projectile physics.
 */
public class ShooterSetpoint {
  // ===== Calculation Parameters (6328-style) =====
  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Shot calculation parameters
  private static double minDistance = 1.0; // meters
  private static double maxDistance = 6.0; // meters
  private static double phaseDelay = 0.03; // seconds - prediction lookahead

  // Filters for feedforward velocity calculation (6328-style)
  private static Rotation2d lastTurretAngle = null;
  private static double lastHoodAngle = Double.NaN;
  private static final LinearFilter turretAngleFilter = LinearFilter.movingAverage(10);
  private static final LinearFilter hoodAngleFilter = LinearFilter.movingAverage(10);

  // Override for testing
  public static Optional<Double> overrideShooterRPS = Optional.empty();

  // Latest calculated parameters (cached for reading)
  private static ShooterSetpoint latestParameters = null;

  // ===== Setpoint Data (254-style structure) =====
  private final double shooterRPS;
  private final double shooterStage1RPS; // For multi-stage shooters
  private final double turretRadiansFromCenter;
  private final double turretFeedforwardRadPerSec;
  private final double hoodRadians;
  private final double hoodFeedforwardRadPerSec;
  private final boolean isValid;

  // ===== Static Initialization Block (6328-style interpolation maps) =====
  static {
    // Configure shot parameters based on distance
    // These should be tuned based on your shooter's characteristics

    // Hood angle vs distance (interpolated)
    shotHoodAngleMap.put(1.0, Rotation2d.fromDegrees(50.0));
    shotHoodAngleMap.put(1.5, Rotation2d.fromDegrees(45.0));
    shotHoodAngleMap.put(2.0, Rotation2d.fromDegrees(40.0));
    shotHoodAngleMap.put(2.5, Rotation2d.fromDegrees(35.0));
    shotHoodAngleMap.put(3.0, Rotation2d.fromDegrees(32.0));
    shotHoodAngleMap.put(4.0, Rotation2d.fromDegrees(28.0));
    shotHoodAngleMap.put(5.0, Rotation2d.fromDegrees(25.0));
    shotHoodAngleMap.put(6.0, Rotation2d.fromDegrees(22.0));

    // Flywheel speed vs distance (RPS - rotations per second)
    shotFlywheelSpeedMap.put(1.0, 50.0);
    shotFlywheelSpeedMap.put(1.5, 55.0);
    shotFlywheelSpeedMap.put(2.0, 58.0);
    shotFlywheelSpeedMap.put(2.5, 60.0);
    shotFlywheelSpeedMap.put(3.0, 62.0);
    shotFlywheelSpeedMap.put(4.0, 65.0);
    shotFlywheelSpeedMap.put(5.0, 68.0);
    shotFlywheelSpeedMap.put(6.0, 70.0);

    // Time of flight vs distance (seconds) - for motion compensation
    timeOfFlightMap.put(1.0, 0.2);
    timeOfFlightMap.put(1.5, 0.3);
    timeOfFlightMap.put(2.0, 0.4);
    timeOfFlightMap.put(2.5, 0.5);
    timeOfFlightMap.put(3.0, 0.6);
    timeOfFlightMap.put(4.0, 0.75);
    timeOfFlightMap.put(5.0, 0.9);
    timeOfFlightMap.put(6.0, 1.0);
  }

  // ===== Constructors (254-style) =====

  /**
   * Create a shooter setpoint with all parameters and validity flag.
   *
   * @param shooterRPS Main shooter wheel speed in rotations per second
   * @param turretRadiansFromCenter Turret angle relative to robot (radians)
   * @param turretFeedforwardRadPerSec Turret angular velocity feedforward (rad/s)
   * @param hoodRadians Hood angle (radians)
   * @param hoodFeedforwardRadPerSec Hood angular velocity feedforward (rad/s)
   * @param isValid Whether this setpoint is physically achievable
   */
  public ShooterSetpoint(
      double shooterRPS,
      double turretRadiansFromCenter,
      double turretFeedforwardRadPerSec,
      double hoodRadians,
      double hoodFeedforwardRadPerSec,
      boolean isValid) {
    this.shooterRPS = shooterRPS;
    this.shooterStage1RPS = 14.4; // Default stage 1 speed (adjust as needed)
    this.turretRadiansFromCenter = turretRadiansFromCenter;
    this.turretFeedforwardRadPerSec = turretFeedforwardRadPerSec;
    this.hoodRadians = hoodRadians;
    this.hoodFeedforwardRadPerSec = hoodFeedforwardRadPerSec;
    this.isValid = isValid;
  }

  /** Create a shooter setpoint assuming it's valid. */
  public ShooterSetpoint(
      double shooterRPS,
      double turretRadiansFromCenter,
      double turretFeedforwardRadPerSec,
      double hoodRadians,
      double hoodFeedforwardRadPerSec) {
    this(
        shooterRPS,
        turretRadiansFromCenter,
        turretFeedforwardRadPerSec,
        hoodRadians,
        hoodFeedforwardRadPerSec,
        true);
  }

  // ===== Getters (254-style) =====

  public double getShooterRPS() {
    return shooterRPS;
  }

  public double getShooterStage1RPS() {
    return shooterStage1RPS;
  }

  public double getTurretRadiansFromCenter() {
    return turretRadiansFromCenter;
  }

  public double getTurretFeedforward() {
    return turretFeedforwardRadPerSec;
  }

  public double getHoodRadians() {
    return hoodRadians;
  }

  public double getHoodFeedforward() {
    return hoodFeedforwardRadPerSec;
  }

  public boolean getIsValid() {
    return isValid;
  }

  // ===== Static Factory Methods (254-style suppliers) =====

  /**
   * Create a supplier that calculates speaker shot setpoints. Uses current project's turret offset
   * accounting.
   */
  public static Supplier<ShooterSetpoint> speakerSetpointSupplier(RobotState robotState) {
    return () -> calculateSpeakerShot(robotState);
  }

  /** Get the most recently calculated setpoint (for efficiency). */
  public static ShooterSetpoint getLatestParameters() {
    return latestParameters;
  }

  /** Clear the cached shooting parameters. */
  public static void clearShootingParameters() {
    latestParameters = null;
    lastTurretAngle = null;
    lastHoodAngle = Double.NaN;
  }

  // ===== Calculation Methods (6328-style physics with current project's turret logic) =====

  /**
   * Calculate shooter setpoint for speaker shots using: - 6328's interpolation maps and motion
   * compensation - Current project's turret offset accounting - Physics-based trajectory
   * calculation
   */
  private static ShooterSetpoint calculateSpeakerShot(RobotState robotState) {
    // Get current robot state
    Pose2d robotPose = robotState.getLatestFieldToRobot().getValue();
    ChassisSpeeds robotVelocity = robotState.getLatestMeasuredFieldRelativeChassisSpeeds();

    // Account for system latency by predicting future pose (6328-style)
    Pose2d estimatedPose =
        robotPose.exp(
            new edu.wpi.first.math.geometry.Twist2d(
                robotVelocity.vxMetersPerSecond * phaseDelay,
                robotVelocity.vyMetersPerSecond * phaseDelay,
                robotVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate turret position accounting for offset from robot center
    // (Current project's approach)
    Translation2d robotPosition = estimatedPose.getTranslation();
    Rotation2d robotHeading = estimatedPose.getRotation();
    Translation2d turretOffset = Constants.TurretConstants.TURRET_OFFSET_FROM_ROBOT_CENTER;
    Translation2d turretOffsetRotated = turretOffset.rotateBy(robotHeading);
    Translation2d turretPosition = robotPosition.plus(turretOffsetRotated);

    // Get target position (speaker)
    // TODO: Add alliance-aware target selection
    Translation3d target = Constants.FieldPoses.BLUE_AIM_TARGET;

    // Calculate distance from turret to target
    Translation2d turretToTarget =
        new Translation2d(target.getX(), target.getY()).minus(turretPosition);
    double turretToTargetDistance = turretToTarget.getNorm();

    // Calculate field relative turret velocity for motion compensation (6328-style)
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (turretOffset.getY() * Math.cos(robotAngle)
                    - turretOffset.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (turretOffset.getX() * Math.cos(robotAngle)
                    - turretOffset.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot to game piece (6328-style)
    double timeOfFlight = timeOfFlightMap.get(turretToTargetDistance);
    double offsetX = -turretVelocityX * timeOfFlight;
    double offsetY = -turretVelocityY * timeOfFlight;

    // Calculate lookahead target position
    Translation2d lookaheadTarget =
        new Translation2d(target.getX() + offsetX, target.getY() + offsetY);
    Translation2d lookaheadTurretToTarget = lookaheadTarget.minus(turretPosition);
    double lookaheadDistance = lookaheadTurretToTarget.getNorm();

    // Calculate turret angle (field-relative angle to target)
    Rotation2d turretAngleFieldRelative = lookaheadTurretToTarget.getAngle();

    // Convert to robot-relative (current project's approach)
    Rotation2d turretAngle = turretAngleFieldRelative.minus(robotHeading);

    // Get hood angle from interpolation map (6328-style)
    double hoodAngle = shotHoodAngleMap.get(lookaheadDistance).getRadians();

    // Calculate feedforward velocities (6328-style with filters)
    double turretVelocity = 0.0;
    double hoodVelocity = 0.0;

    if (lastTurretAngle != null) {
      turretVelocity =
          turretAngleFilter.calculate(
              turretAngle.minus(lastTurretAngle).getRadians() / 0.02); // Assuming 20ms loop
    }
    if (!Double.isNaN(lastHoodAngle)) {
      hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / 0.02);
    }

    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;

    // Get shooter speed from interpolation map (6328-style)
    double shooterSpeed = shotFlywheelSpeedMap.get(lookaheadDistance);

    // Apply override if set
    if (overrideShooterRPS.isPresent()) {
      shooterSpeed = overrideShooterRPS.get();
    }

    // Check if shot is within valid range
    boolean isValid = lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance;

    // Create and cache the setpoint
    latestParameters =
        new ShooterSetpoint(
            shooterSpeed,
            turretAngle.getRadians(),
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            isValid);

    return latestParameters;
  }

  // ===== Configuration Methods =====

  /** Set shooter RPS override for testing. */
  public static void setOverrideRPS(double rps) {
    overrideShooterRPS = Optional.of(rps);
  }

  /** Clear shooter RPS override. */
  public static void clearOverrideRPS() {
    overrideShooterRPS = Optional.empty();
  }

  /** Configure the valid shot distance range. */
  public static void setDistanceRange(double min, double max) {
    minDistance = min;
    maxDistance = max;
  }

  /** Configure the prediction lookahead time (phase delay). */
  public static void setPhaseDelay(double delaySeconds) {
    phaseDelay = delaySeconds;
  }
}
