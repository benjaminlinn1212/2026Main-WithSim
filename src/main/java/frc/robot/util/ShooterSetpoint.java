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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.auto.dashboard.FieldConstants;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Complete shooting solution that calculates shooter speed, turret angle, hood angle, and
 * feedforward velocities for shoot-while-driving.
 *
 * <p>Combines:
 *
 * <ul>
 *   <li>Distance-based interpolation maps for hood angle, flywheel speed, and time of flight
 *   <li>Motion compensation: predicts future robot pose and compensates for projectile travel time
 *   <li>Turret offset accounting: aims from the turret's physical position, not robot center
 *   <li>Feedforward velocities: filtered angular rates for turret and hood tracking
 * </ul>
 *
 * <p>Usage: Create a supplier via {@link #createSupplier(RobotState)}, then pass it to each aiming
 * subsystem (shooter, turret, hood) so they share a single coordinated setpoint.
 */
public class ShooterSetpoint {

  // ===== Interpolation Maps =====
  // Keyed by distance from turret to target (meters)
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Feedforward filters (smooths angular velocity estimates)
  private static Rotation2d lastTurretAngle = null;
  private static double lastHoodAngle = Double.NaN;
  private static final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage(Constants.Aiming.FEEDFORWARD_FILTER_TAPS);
  private static final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage(Constants.Aiming.FEEDFORWARD_FILTER_TAPS);

  // ===== Per-cycle cache to avoid redundant calculateShot() calls =====
  private static ShooterSetpoint cachedSetpoint = null;
  private static long cachedCycleCount = -1;

  // ===== Setpoint Data =====
  private final double shooterRPS;
  private final double turretAngleRad;
  private final double turretFeedforwardRadPerSec;
  private final double hoodAngleRad;
  private final double hoodFeedforwardRadPerSec;
  private final boolean isValid;
  private final boolean isNeutralZoneShot;

  // ===== Interpolation Map Initialization =====
  static {
    // Hood angle vs distance (meters → radians from horizontal)
    hoodAngleMap.put(1.25, Rotation2d.fromDegrees(21.0));
    hoodAngleMap.put(1.5, Rotation2d.fromDegrees(21.0));
    hoodAngleMap.put(1.75, Rotation2d.fromDegrees(22.0));
    hoodAngleMap.put(2.0, Rotation2d.fromDegrees(23.0));
    hoodAngleMap.put(2.5, Rotation2d.fromDegrees(25.0));
    hoodAngleMap.put(3.0, Rotation2d.fromDegrees(29.0));
    hoodAngleMap.put(3.5, Rotation2d.fromDegrees(29.0));
    hoodAngleMap.put(4.0, Rotation2d.fromDegrees(30.0));
    hoodAngleMap.put(4.5, Rotation2d.fromDegrees(34.0));
    hoodAngleMap.put(4.83, Rotation2d.fromDegrees(37.0));

    // Flywheel speed vs distance (rotations per second)
    flywheelSpeedMap.put(1.25, 45.0);
    flywheelSpeedMap.put(1.5, 50.0);
    flywheelSpeedMap.put(1.75, 50.0);
    flywheelSpeedMap.put(2.0, 50.0);
    flywheelSpeedMap.put(2.5, 55.0);
    flywheelSpeedMap.put(3.0, 55.0);
    flywheelSpeedMap.put(3.5, 60.0);
    flywheelSpeedMap.put(4.0, 65.0);
    flywheelSpeedMap.put(4.5, 65.0);
    flywheelSpeedMap.put(4.83, 65.0);

    // Time of flight vs distance (seconds) — for motion compensation
    timeOfFlightMap.put(1.25, 1.0);
    timeOfFlightMap.put(1.5, 1.02);
    timeOfFlightMap.put(1.75, 1.08);
    timeOfFlightMap.put(2.0, 1.13);
    timeOfFlightMap.put(2.5, 1.13);
    timeOfFlightMap.put(3.0, 1.18);
    timeOfFlightMap.put(3.5, 1.34);
    timeOfFlightMap.put(4.0, 1.34);
    timeOfFlightMap.put(4.5, 1.29);
    timeOfFlightMap.put(4.83, 1.33);
  }

  // ===== Constructors =====

  /**
   * Create a shooter setpoint with all parameters.
   *
   * @param shooterRPS Flywheel speed (rotations per second)
   * @param turretAngleRad Turret angle relative to robot center (radians, 0 = forward)
   * @param turretFeedforwardRadPerSec Turret angular velocity feedforward (rad/s)
   * @param hoodAngleRad Hood angle from horizontal (radians)
   * @param hoodFeedforwardRadPerSec Hood angular velocity feedforward (rad/s)
   * @param isValid Whether this setpoint is physically achievable (distance in range)
   * @param isNeutralZoneShot Whether this is a neutral-zone shoot-back shot
   */
  public ShooterSetpoint(
      double shooterRPS,
      double turretAngleRad,
      double turretFeedforwardRadPerSec,
      double hoodAngleRad,
      double hoodFeedforwardRadPerSec,
      boolean isValid,
      boolean isNeutralZoneShot) {
    this.shooterRPS = shooterRPS;
    this.turretAngleRad = turretAngleRad;
    this.turretFeedforwardRadPerSec = turretFeedforwardRadPerSec;
    this.hoodAngleRad = hoodAngleRad;
    this.hoodFeedforwardRadPerSec = hoodFeedforwardRadPerSec;
    this.isValid = isValid;
    this.isNeutralZoneShot = isNeutralZoneShot;
  }

  /** Create a setpoint with validity flag (assumes not a neutral-zone shot). */
  public ShooterSetpoint(
      double shooterRPS,
      double turretAngleRad,
      double turretFeedforwardRadPerSec,
      double hoodAngleRad,
      double hoodFeedforwardRadPerSec,
      boolean isValid) {
    this(
        shooterRPS,
        turretAngleRad,
        turretFeedforwardRadPerSec,
        hoodAngleRad,
        hoodFeedforwardRadPerSec,
        isValid,
        false);
  }

  /** Cached invalid/default setpoint singleton to avoid repeated allocations. */
  private static final ShooterSetpoint INVALID = new ShooterSetpoint(0, 0, 0, 0, 0, false);

  /** Create an invalid/default setpoint (used as supplier default before aiming starts). */
  public static ShooterSetpoint invalid() {
    return INVALID;
  }

  // ===== Getters =====

  /** Flywheel speed in rotations per second. */
  public double getShooterRPS() {
    return shooterRPS;
  }

  /** Turret angle relative to robot center in radians (0 = forward, CCW positive). */
  public double getTurretAngleRad() {
    return turretAngleRad;
  }

  /** Turret angular velocity feedforward in rad/s (for shoot-while-driving tracking). */
  public double getTurretFeedforwardRadPerSec() {
    return turretFeedforwardRadPerSec;
  }

  /** Hood angle from horizontal in radians. */
  public double getHoodAngleRad() {
    return hoodAngleRad;
  }

  /** Hood angular velocity feedforward in rad/s (for shoot-while-driving tracking). */
  public double getHoodFeedforwardRadPerSec() {
    return hoodFeedforwardRadPerSec;
  }

  /** Whether this setpoint is physically achievable (target within valid distance range). */
  public boolean isValid() {
    return isValid;
  }

  /** Whether this is a neutral-zone shoot-back shot (fixed speed/angle, no motion compensation). */
  public boolean isNeutralZoneShot() {
    return isNeutralZoneShot;
  }

  // ===== Factory =====

  /**
   * Create a supplier that calculates hub shot setpoints each call.
   *
   * @param robotState Robot state providing pose and velocity data
   * @return Supplier that computes a fresh ShooterSetpoint each invocation
   */
  public static Supplier<ShooterSetpoint> createSupplier(RobotState robotState) {
    return () -> {
      // Cache the result so multiple .get() calls in the same 20ms cycle don't recompute.
      // Uses FPGA timestamp truncated to cycle count as a cheap "cycle ID".
      long cycleId = (long) (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() * 1000.0 / 20.0);
      if (cycleId != cachedCycleCount || cachedSetpoint == null) {
        cachedSetpoint = calculateShot(robotState);
        cachedCycleCount = cycleId;
      }
      return cachedSetpoint;
    };
  }

  // ===== Calculation Methods (6328-style physics with current project's turret logic) =====

  /**
   * Determines if robot is outside the aiming zone (past the HUB zone boundary). Returns true if
   * the robot should shoot back towards the alliance wall instead of at the hub.
   *
   * <p>Uses the same X=5.5m threshold (NEUTRAL_ZONE.minX) as AutoTuning's AIMING_ZONE_MAX_X. The
   * robot's blue-origin X is compared against this boundary — if beyond it, the robot is in the
   * neutral zone and should do a feedback shot towards the alliance wall.
   */
  private static boolean isInNeutralZone(Pose2d robotPose, boolean isBlueAlliance) {
    // Convert to blue-origin X for consistent zone checking
    double blueX =
        isBlueAlliance ? robotPose.getX() : FieldConstants.FIELD_LENGTH - robotPose.getX();

    // Robot is in neutral zone if past X=5.50m (NEUTRAL_ZONE.minX)
    return blueX > FieldConstants.Zone.NEUTRAL_ZONE.minX;
  }

  /**
   * Get the appropriate target based on alliance and robot position. Returns hub target normally,
   * or neutral zone aim target if in neutral zone (upper/lower based on robot Y).
   */
  private static Translation3d getSmartTarget(
      Pose2d robotPose, boolean isBlueAlliance, boolean isNeutralZone) {
    Translation3d hubTarget =
        isBlueAlliance
            ? FieldConstants.BLUE_HUB_TRANSLATION3D
            : FieldConstants.RED_HUB_POSE_TRANSLATION3D;

    // Check if in neutral zone
    if (isNeutralZone) {
      // Aim at upper or lower neutral zone target based on robot Y position
      double robotY = robotPose.getY();
      if (isBlueAlliance) {
        return robotY > FieldConstants.FIELD_WIDTH / 2.0
            ? FieldConstants.BLUE_NEUTRAL_ZONE_UPPER_AIM
            : FieldConstants.BLUE_NEUTRAL_ZONE_LOWER_AIM;
      } else {
        return robotY > FieldConstants.FIELD_WIDTH / 2.0
            ? FieldConstants.RED_NEUTRAL_ZONE_UPPER_AIM
            : FieldConstants.RED_NEUTRAL_ZONE_LOWER_AIM;
      }
    }

    // Normal hub shot
    return hubTarget;
  }

  /**
   * Calculate shooter setpoint using interpolation maps, motion compensation, and turret offset
   * accounting. Handles both normal hub shots and neutral-zone shoot-back.
   */
  private static ShooterSetpoint calculateShot(RobotState robotState) {
    if (robotState.getLatestFieldToRobot() == null) {
      return ShooterSetpoint.invalid();
    }

    Pose2d robotPose = robotState.getLatestFieldToRobot().getValue();
    ChassisSpeeds fieldRelativeVelocity = robotState.getLatestMeasuredFieldRelativeChassisSpeeds();
    ChassisSpeeds robotRelativeVelocity = robotState.getLatestRobotRelativeChassisSpeed();

    // Cache alliance once per calculation to avoid repeated DriverStation lookups
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isBlueAlliance = alliance.isEmpty() || alliance.get() == Alliance.Blue;

    // Predict future pose to compensate for system latency
    // IMPORTANT: Pose2d.exp() expects a robot-relative Twist2d (dx = forward, dy = left),
    // NOT field-relative velocities. Using field-relative here caused the predicted heading
    // to diverge at high angular velocities, leading to aim errors.
    double phaseDelay = Constants.Aiming.PHASE_DELAY;
    Pose2d estimatedPose =
        robotPose.exp(
            new edu.wpi.first.math.geometry.Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate turret position in field frame (robot center + rotated turret offset)
    Translation2d robotPosition = estimatedPose.getTranslation();
    Rotation2d robotHeading = estimatedPose.getRotation();
    Translation2d turretOffset = Constants.TurretConstants.TURRET_OFFSET_FROM_ROBOT_CENTER;
    Translation2d turretOffsetRotated = turretOffset.rotateBy(robotHeading);
    Translation2d turretPosition = robotPosition.plus(turretOffsetRotated);

    // Select target (hub or alliance wall if in neutral zone)
    boolean isNeutralZoneShot = isInNeutralZone(robotPose, isBlueAlliance);
    Translation3d target = getSmartTarget(robotPose, isBlueAlliance, isNeutralZoneShot);

    if (isNeutralZoneShot) {
      return calculateNeutralZoneShot(turretPosition, target, robotHeading);
    }

    return calculateHubShot(
        turretPosition, target, fieldRelativeVelocity, estimatedPose, turretOffset, robotHeading);
  }

  /**
   * Calculate a simple neutral-zone shot (fixed speed/angle, aim at alliance wall). No motion
   * compensation — just point and shoot.
   */
  private static ShooterSetpoint calculateNeutralZoneShot(
      Translation2d turretPosition, Translation3d target, Rotation2d robotHeading) {
    Translation2d turretToTarget =
        new Translation2d(target.getX(), target.getY()).minus(turretPosition);
    Rotation2d turretAngle = turretToTarget.getAngle().minus(robotHeading);

    double hoodAngleRad = Math.toRadians(Constants.Aiming.NEUTRAL_ZONE_HOOD_ANGLE_DEG);
    double shooterRPS = Constants.ShooterConstants.NEUTRAL_ZONE_SPEED;

    Logger.recordOutput("ShooterSetpoint/NeutralZone/Speed", shooterRPS);
    Logger.recordOutput("ShooterSetpoint/NeutralZone/HoodAngleDeg", Math.toDegrees(hoodAngleRad));

    return new ShooterSetpoint(
        shooterRPS, turretAngle.getRadians(), 0.0, hoodAngleRad, 0.0, true, true);
  }

  /**
   * Calculate a normal hub shot with full motion compensation.
   *
   * <p>Motion compensation flow:
   *
   * <ol>
   *   <li>Calculate turret velocity in field frame (translation + rotation cross product)
   *   <li>Look up time-of-flight for current distance
   *   <li>Offset target by -velocity * timeOfFlight (where the target will "appear" to be)
   *   <li>Compute turret angle and hood angle from corrected distance
   *   <li>Compute feedforward angular velocities via filtered finite differences
   * </ol>
   */
  private static ShooterSetpoint calculateHubShot(
      Translation2d turretPosition,
      Translation3d target,
      ChassisSpeeds robotVelocity,
      Pose2d estimatedPose,
      Translation2d turretOffset,
      Rotation2d robotHeading) {

    // Distance from turret to target (before motion compensation)
    Translation2d turretToTarget =
        new Translation2d(target.getX(), target.getY()).minus(turretPosition);
    double rawDistance = turretToTarget.getNorm();

    // ── Turret velocity in field frame ──
    // v_turret = v_robot + ω × R(θ) * turretOffset
    // For 2D cross product ω × (x, y) = (-ωy, ωx):
    //   rotated offset = R(θ) * turretOffset
    //   v_turret_x = vx_robot + ω * (-rotatedOffset.y)
    //   v_turret_y = vy_robot + ω * ( rotatedOffset.x)
    Translation2d rotatedOffset = turretOffset.rotateBy(robotHeading);
    double omega = robotVelocity.omegaRadiansPerSecond;
    double turretVelocityX = robotVelocity.vxMetersPerSecond - omega * rotatedOffset.getY();
    double turretVelocityY = robotVelocity.vyMetersPerSecond + omega * rotatedOffset.getX();

    // ── Motion compensation (iterative) ──
    // The game piece inherits the turret's velocity, so we offset the target
    // by -turretVelocity * timeOfFlight to aim where the target will effectively be.
    // We iterate because offsetting the target changes the distance, which changes the
    // time-of-flight. 3 iterations is sufficient for convergence (used by 254/6328).
    double compensatedDistance = rawDistance;
    Translation2d lookaheadTurretToTarget = turretToTarget;
    for (int i = 0; i < 3; i++) {
      double timeOfFlight = timeOfFlightMap.get(compensatedDistance);
      Translation2d lookaheadTarget =
          new Translation2d(
              target.getX() - turretVelocityX * timeOfFlight,
              target.getY() - turretVelocityY * timeOfFlight);
      lookaheadTurretToTarget = lookaheadTarget.minus(turretPosition);
      compensatedDistance = lookaheadTurretToTarget.getNorm();
    }

    // ── Turret angle (robot-relative) ──
    Rotation2d turretAngle = lookaheadTurretToTarget.getAngle().minus(robotHeading);

    // ── Hood angle and flywheel speed from interpolation maps ──
    double hoodAngleRad = hoodAngleMap.get(compensatedDistance).getRadians();
    double shooterRPS = flywheelSpeedMap.get(compensatedDistance);

    // ── Feedforward angular velocities (filtered finite differences) ──
    double turretFF = 0.0;
    double hoodFF = 0.0;
    if (lastTurretAngle != null) {
      turretFF =
          turretAngleFilter.calculate(turretAngle.minus(lastTurretAngle).getRadians() / 0.02);
    }
    if (!Double.isNaN(lastHoodAngle)) {
      hoodFF = hoodAngleFilter.calculate((hoodAngleRad - lastHoodAngle) / 0.02);
    }
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngleRad;

    // ── Logging ──
    Logger.recordOutput("ShooterSetpoint/Distance", compensatedDistance);
    Logger.recordOutput("ShooterSetpoint/ShooterRPS", shooterRPS);
    Logger.recordOutput("ShooterSetpoint/HoodAngleDeg", Math.toDegrees(hoodAngleRad));
    Logger.recordOutput("ShooterSetpoint/TurretAngleDeg", turretAngle.getDegrees());
    Logger.recordOutput("ShooterSetpoint/TurretFF", turretFF);
    Logger.recordOutput("ShooterSetpoint/HoodFF", hoodFF);

    // ── Validity check ──
    boolean isValid =
        compensatedDistance >= Constants.Aiming.MIN_SHOT_DISTANCE
            && compensatedDistance <= Constants.Aiming.MAX_SHOT_DISTANCE;

    return new ShooterSetpoint(
        shooterRPS, turretAngle.getRadians(), turretFF, hoodAngleRad, hoodFF, isValid);
  }
}
