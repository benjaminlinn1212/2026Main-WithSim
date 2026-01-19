// Copyright (c) 2021-2026 Littleton Robotics
// Adapted from Team 254's 2025 code
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import java.util.Map;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

/**
 * RobotState keeps track of the robot's pose and velocity over time. This matches Team 254's
 * architecture.
 */
public class RobotState {
  private static final double kObservationBufferTime = 2.0; // seconds

  private final TreeMap<Double, Pose2d> fieldToRobot = new TreeMap<>();
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
    }
  }
}
