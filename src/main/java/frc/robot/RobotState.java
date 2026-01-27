package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private final TreeMap<Double, Rotation2d> robotToTurret = new TreeMap<>();
  private final TreeMap<Double, Double> turretAngularVelocity = new TreeMap<>();

  private ChassisSpeeds measuredFieldRelativeChassisSpeeds = new ChassisSpeeds();
  private ChassisSpeeds robotRelativeChassisSpeed = new ChassisSpeeds();

  public RobotState() {}

  /** Add a new robot pose observation at the current timestamp */
  public void addFieldToRobot(Pose2d pose) {
    addFieldToRobot(Timer.getFPGATimestamp(), pose);
  }

  /** Add a new robot pose observation at a specific timestamp */
  public void addFieldToRobot(double timestamp, Pose2d pose) {
    fieldToRobot.put(timestamp, pose);
    cleanUpObservations();
  }

  /** Add turret rotation update (robot-relative) */
  public void addTurretUpdates(
      double timestamp, Rotation2d turretRotation, double angularVelocityRadsPerS) {
    robotToTurret.put(timestamp, turretRotation);
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
  public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
    return fieldToRobot.lastEntry();
  }

  /** Get robot pose at a specific timestamp */
  public Optional<Pose2d> getFieldToRobot(double timestamp) {
    var entry = fieldToRobot.floorEntry(timestamp);
    if (entry == null) {
      return Optional.empty();
    }
    return Optional.of(entry.getValue());
  }

  /** Get the most recent robot-to-turret rotation */
  public Map.Entry<Double, Rotation2d> getLatestRobotToTurret() {
    var entry = robotToTurret.lastEntry();
    if (entry == null) {
      return Map.entry(Timer.getFPGATimestamp(), new Rotation2d());
    }
    return entry;
  }

  /** Get robot-to-turret rotation at a specific timestamp */
  public Optional<Rotation2d> getRobotToTurret(double timestamp) {
    var entry = robotToTurret.floorEntry(timestamp);
    if (entry == null) {
      return Optional.empty();
    }
    return Optional.of(entry.getValue());
  }

  /** Get the latest turret angular velocity */
  public double getLatestTurretAngularVelocity() {
    var entry = turretAngularVelocity.lastEntry();
    if (entry == null) {
      return 0.0;
    }
    return entry.getValue();
  }

  /** Get turret-to-camera transform accounting for camera being on turret */
  public Transform2d getTurretToCamera(boolean isTurretCamera) {
    if (isTurretCamera) {
      // Camera is on turret - return the turret-to-camera transform
      var transform3d = Constants.Vision.TURRET_TO_CAMERA;
      return new Transform2d(
          transform3d.getTranslation().toTranslation2d(), transform3d.getRotation().toRotation2d());
    } else {
      // Camera is on robot body - no turret offset
      return new Transform2d();
    }
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
    fieldToRobot.headMap(cutoffTime).clear();
    robotToTurret.headMap(cutoffTime).clear();
    turretAngularVelocity.headMap(cutoffTime).clear();
  }

  /** Log the robot state for debugging */
  public void log() {
    if (getLatestFieldToRobot() != null) {
      Logger.recordOutput("RobotState/Pose", getLatestFieldToRobot().getValue());
      Logger.recordOutput(
          "RobotState/FieldRelativeVelocityX",
          measuredFieldRelativeChassisSpeeds.vxMetersPerSecond);
      Logger.recordOutput(
          "RobotState/FieldRelativeVelocityY",
          measuredFieldRelativeChassisSpeeds.vyMetersPerSecond);
      Logger.recordOutput(
          "RobotState/AngularVelocity", robotRelativeChassisSpeed.omegaRadiansPerSecond);
      Logger.recordOutput("RobotState/TurretRotation", getLatestRobotToTurret().getValue());
      Logger.recordOutput("RobotState/TurretAngularVelocity", getLatestTurretAngularVelocity());
    }
  }
}
