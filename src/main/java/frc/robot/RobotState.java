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

  private ChassisSpeeds measuredFieldRelativeChassisSpeeds = new ChassisSpeeds();
  private ChassisSpeeds robotRelativeChassisSpeed = new ChassisSpeeds();

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
    cleanUpObservations();
  }

  /** Add turret rotation update (robot-relative, raw radians — not wrapped to [-π,π]). */
  public synchronized void addTurretUpdates(
      double timestamp, double turretAngleRad, double angularVelocityRadsPerS) {
    robotToTurretRad.put(timestamp, turretAngleRad);
    turretAngularVelocity.put(timestamp, angularVelocityRadsPerS);
    cleanUpObservations();
  }

  /** Update the measured chassis speeds */
  public void updateChassisSpeeds(
      ChassisSpeeds fieldRelativeSpeeds, ChassisSpeeds robotRelativeSpeeds) {
    this.measuredFieldRelativeChassisSpeeds = fieldRelativeSpeeds;
    this.robotRelativeChassisSpeed = robotRelativeSpeeds;
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

  /** Get the latest turret angular velocity */
  public synchronized double getLatestTurretAngularVelocity() {
    var entry = turretAngularVelocity.lastEntry();
    if (entry == null) {
      return 0.0;
    }
    return entry.getValue();
  }

  /** Get turret-to-camera 2D transform (projects the 3D TURRET_TO_CAMERA constant to 2D). */
  public Transform2d getTurretToCamera() {
    var transform3d = Constants.Vision.TURRET_TO_CAMERA;
    return new Transform2d(
        transform3d.getTranslation().toTranslation2d(), transform3d.getRotation().toRotation2d());
  }

  /** Get the field-relative chassis speeds */
  public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
    return measuredFieldRelativeChassisSpeeds;
  }

  /** Get the robot-relative chassis speeds */
  public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
    return robotRelativeChassisSpeed;
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
  }

  /** Log the robot state for debugging */
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
      Logger.recordOutput("RobotState/TurretRotationRad", getLatestRobotToTurret().getValue());
      Logger.recordOutput(
          "RobotState/TurretRotationDeg", Math.toDegrees(getLatestRobotToTurret().getValue()));
      Logger.recordOutput("RobotState/TurretAngularVelocity", getLatestTurretAngularVelocity());
    }
  }
}
