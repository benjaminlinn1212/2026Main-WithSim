package frc.robot.subsystems.vision;

public interface VisionIO {

  class VisionIOInputs {
    // Front camera (drivetrain)
    public boolean frontCameraSeesTarget;
    public MegatagPoseEstimate frontCameraMegatagPoseEstimate;
    public MegatagPoseEstimate frontCameraMegatag2PoseEstimate;

    // Turret camera
    public boolean turretCameraSeesTarget;
    public MegatagPoseEstimate turretCameraMegatagPoseEstimate;
    public MegatagPoseEstimate turretCameraMegatag2PoseEstimate;
  }

  /** Update inputs from hardware. */
  void updateInputs(VisionIOInputs inputs);
}
