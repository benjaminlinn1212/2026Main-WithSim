// Copyright (c) 2021-2026 Littleton Robotics
// Adapted from Team 254's 2024 code
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;

public class MegatagPoseEstimate {
  /**
   * The pose returned by the Limelight's botpose estimator. For drivetrain cameras (with
   * camerapose_robotspace_set configured), this is field-to-robot. For the turret camera (with
   * SetRobotOrientation set to turret heading), this is the field-to-"turret-body" pose that needs
   * further transformation to get field-to-robot.
   */
  public Pose2d fieldPose;

  public double timestampSeconds;
  public double latency;
  public int tagCount;
  public int[] fiducialIds;
  public double avgTagArea;
  public double avgTagDist;

  public MegatagPoseEstimate() {}

  public static MegatagPoseEstimate fromLimelight(LimelightHelpers.PoseEstimate estimate) {
    if (estimate == null) {
      return null;
    }
    MegatagPoseEstimate rv = new MegatagPoseEstimate();
    rv.fieldPose = estimate.pose;
    rv.timestampSeconds = estimate.timestampSeconds;
    rv.latency = estimate.latency;
    rv.tagCount = estimate.tagCount;
    rv.avgTagArea = estimate.avgTagArea;
    rv.avgTagDist = estimate.avgTagDist;

    // Extract fiducial IDs from raw fiducials
    if (estimate.rawFiducials != null && estimate.rawFiducials.length > 0) {
      rv.fiducialIds = new int[estimate.rawFiducials.length];
      for (int i = 0; i < estimate.rawFiducials.length; i++) {
        rv.fiducialIds[i] = estimate.rawFiducials[i].id;
      }
    } else {
      rv.fiducialIds = new int[0];
    }

    return rv;
  }
}
