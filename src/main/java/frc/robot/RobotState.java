package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

/**
 * RobotState keeps track of the robot's pose and velocity over time. This matches Team 254's
 * architecture with support for turret tracking.
 */
public class RobotState {
  private static final double kObservationBufferTime = 2.0; // seconds

  private final TreeMap<Double, Pose2d> fieldToRobot = new TreeMap<>();
  /** Stores raw turret angle in radians (unwrapped, NOT clamped to [-π,π]). */
  private final TreeMap<Double, Double> robotToTurretRad = new TreeMap<>();

  private final TreeMap<Double, Double> turretAngularVelocity = new TreeMap<>();
  private final TreeMap<Double, Double> driveAngularVelocity = new TreeMap<>();

  private ChassisSpeeds measuredFieldRelativeChassisSpeeds = new ChassisSpeeds();
  private ChassisSpeeds robotRelativeChassisSpeed = new ChassisSpeeds();

  /** Cleanup counter — runs trim every N additions instead of every call. */
  private int cleanupCounter = 0;

  private static final int CLEANUP_INTERVAL = 50;

  public RobotState() {
    // Seed with zero pose so getLatestFieldToRobot() is never null (matches 254).
    // The real pose will be set during disabled via the 254-style pre-seeding strategy.
    fieldToRobot.put(0.0, new Pose2d());
  }

  /** Add a new robot pose observation at the current timestamp */
  public synchronized void addFieldToRobot(Pose2d pose) {
    addFieldToRobot(Timer.getFPGATimestamp(), pose);
  }

  /** Add a new robot pose observation at a specific timestamp */
  public synchronized void addFieldToRobot(double timestamp, Pose2d pose) {
    fieldToRobot.put(timestamp, pose);
    maybeCleanUp();
  }

  /** Add turret rotation update (robot-relative, raw radians — not wrapped to [-π,π]). */
  public synchronized void addTurretUpdates(
      double timestamp, double turretAngleRad, double angularVelocityRadsPerS) {
    robotToTurretRad.put(timestamp, turretAngleRad);
    turretAngularVelocity.put(timestamp, angularVelocityRadsPerS);
    maybeCleanUp();
  }

  /**
   * Update the measured chassis speeds and record drive angular velocity for rejection filtering.
   */
  public void updateChassisSpeeds(
      ChassisSpeeds fieldRelativeSpeeds, ChassisSpeeds robotRelativeSpeeds) {
    this.measuredFieldRelativeChassisSpeeds = fieldRelativeSpeeds;
    this.robotRelativeChassisSpeed = robotRelativeSpeeds;
    synchronized (this) {
      driveAngularVelocity.put(Timer.getFPGATimestamp(), robotRelativeSpeeds.omegaRadiansPerSecond);
    }
  }

  /** Get the most recent robot pose */
  public synchronized Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
    return fieldToRobot.lastEntry();
  }

  /** Get robot pose at a specific timestamp */
  public synchronized Optional<Pose2d> getFieldToRobot(double timestamp) {
    var entry = fieldToRobot.floorEntry(timestamp);
    if (entry == null) {
      return Optional.empty();
    }
    return Optional.of(entry.getValue());
  }

  /** Get the most recent robot-to-turret rotation in raw radians (unwrapped). */
  public synchronized Map.Entry<Double, Double> getLatestRobotToTurret() {
    var entry = robotToTurretRad.lastEntry();
    if (entry == null) {
      return Map.entry(Timer.getFPGATimestamp(), 0.0);
    }
    return entry;
  }

  /** Get robot-to-turret rotation at a specific timestamp in raw radians (unwrapped). */
  public synchronized Optional<Double> getRobotToTurret(double timestamp) {
    var entry = robotToTurretRad.floorEntry(timestamp);
    if (entry == null) {
      return Optional.empty();
    }
    return Optional.of(entry.getValue());
  }

  /**
   * Interpolated robot-to-turret rotation at a timestamp (raw rad, unwrapped). Linearly
   * interpolates between bracketing entries. Falls back to floor entry if only one side available.
   */
  public synchronized Optional<Double> getInterpolatedRobotToTurret(double timestamp) {
    var floor = robotToTurretRad.floorEntry(timestamp);
    var ceiling = robotToTurretRad.ceilingEntry(timestamp);

    if (floor == null && ceiling == null) {
      return Optional.empty();
    }
    if (floor == null) {
      return Optional.of(ceiling.getValue());
    }
    if (ceiling == null || floor.getKey().equals(ceiling.getKey())) {
      return Optional.of(floor.getValue());
    }

    // Linear interpolation between the two bracketing entries
    double t = (timestamp - floor.getKey()) / (ceiling.getKey() - floor.getKey());
    return Optional.of(floor.getValue() + t * (ceiling.getValue() - floor.getValue()));
  }

  /** Get the latest turret angular velocity */
  public synchronized double getLatestTurretAngularVelocity() {
    var entry = turretAngularVelocity.lastEntry();
    if (entry == null) {
      return 0.0;
    }
    return entry.getValue();
  }

  /** Max abs turret angular velocity (rad/s) in a time range. For vision rejection. */
  public synchronized Optional<Double> getMaxAbsTurretAngularVelocityInRange(
      double minTime, double maxTime) {
    return getMaxAbsInRange(turretAngularVelocity, minTime, maxTime);
  }

  /** Max abs drive angular velocity (rad/s) in a time range. For vision rejection. */
  public synchronized Optional<Double> getMaxAbsDriveAngularVelocityInRange(
      double minTime, double maxTime) {
    return getMaxAbsInRange(driveAngularVelocity, minTime, maxTime);
  }

  /** Return the max-absolute value stored in a TreeMap within the given time window. */
  private static Optional<Double> getMaxAbsInRange(
      TreeMap<Double, Double> buffer, double minTime, double maxTime) {
    var submap = buffer.subMap(minTime, true, maxTime, true);
    if (submap.isEmpty()) {
      return Optional.empty();
    }
    double maxAbs = 0.0;
    for (double v : submap.values()) {
      double abs = Math.abs(v);
      if (abs > maxAbs) {
        maxAbs = abs;
      }
    }
    return Optional.of(maxAbs);
  }

  /** Cached 2D turret-to-camera transform (constant — computed once at class load). */
  private static final Transform2d TURRET_TO_CAMERA_2D;

  static {
    var transform3d = Constants.Vision.TURRET_TO_CAMERA;
    TURRET_TO_CAMERA_2D =
        new Transform2d(
            transform3d.getTranslation().toTranslation2d(),
            transform3d.getRotation().toRotation2d());
  }

  /** Get turret-to-camera 2D transform (projects the 3D TURRET_TO_CAMERA constant to 2D). */
  public Transform2d getTurretToCamera() {
    return TURRET_TO_CAMERA_2D;
  }

  /** Get the field-relative chassis speeds */
  public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
    return measuredFieldRelativeChassisSpeeds;
  }

  /** Get the robot-relative chassis speeds */
  public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
    return robotRelativeChassisSpeed;
  }

  /** Throttled cleanup — only runs the expensive TreeMap trimming every N additions. */
  private void maybeCleanUp() {
    if (++cleanupCounter >= CLEANUP_INTERVAL) {
      cleanupCounter = 0;
      cleanUpObservations();
    }
  }

  /** Remove old observations outside the buffer window */
  private void cleanUpObservations() {
    double currentTime = Timer.getFPGATimestamp();
    double cutoffTime = currentTime - kObservationBufferTime;
    // Use NavigableMap.headMap(exclusive=false) and poll to avoid ConcurrentModificationException.
    // headMap().clear() uses an iterator internally which can conflict with concurrent puts.
    while (!fieldToRobot.isEmpty() && fieldToRobot.firstKey() < cutoffTime) {
      fieldToRobot.pollFirstEntry();
    }
    while (!robotToTurretRad.isEmpty() && robotToTurretRad.firstKey() < cutoffTime) {
      robotToTurretRad.pollFirstEntry();
    }
    while (!turretAngularVelocity.isEmpty() && turretAngularVelocity.firstKey() < cutoffTime) {
      turretAngularVelocity.pollFirstEntry();
    }
    while (!driveAngularVelocity.isEmpty() && driveAngularVelocity.firstKey() < cutoffTime) {
      driveAngularVelocity.pollFirstEntry();
    }
  }

  /** Log robot state for debugging. Uses direct TreeMap access (already synchronized). */
  public synchronized void log() {
    var latestPose = fieldToRobot.lastEntry();
    if (latestPose != null) {
      Logger.recordOutput("RobotState/Pose", latestPose.getValue());
      Logger.recordOutput(
          "RobotState/FieldRelativeVelocityX",
          measuredFieldRelativeChassisSpeeds.vxMetersPerSecond);
      Logger.recordOutput(
          "RobotState/FieldRelativeVelocityY",
          measuredFieldRelativeChassisSpeeds.vyMetersPerSecond);
      Logger.recordOutput(
          "RobotState/AngularVelocity", robotRelativeChassisSpeed.omegaRadiansPerSecond);

      // Direct TreeMap access — no lock re-acquisition
      var turretEntry = robotToTurretRad.lastEntry();
      double turretRad = turretEntry != null ? turretEntry.getValue() : 0.0;
      Logger.recordOutput("RobotState/TurretRotationDeg", Math.toDegrees(turretRad));

      var angVelEntry = turretAngularVelocity.lastEntry();
      Logger.recordOutput(
          "RobotState/TurretAngularVelocity", angVelEntry != null ? angVelEntry.getValue() : 0.0);
    }
  }
}
