package frc.robot.subsystems.vision;

public interface VisionIO {

  class VisionIOInputs {
    // Front camera (drivetrain)
    public boolean frontCameraSeesTarget;
    public FiducialObservation[] frontCameraFiducialObservations = new FiducialObservation[0];
    public MegatagPoseEstimate frontCameraMegatagPoseEstimate;
    public int frontCameraMegatagCount;
    public MegatagPoseEstimate frontCameraMegatag2PoseEstimate;

    // Back camera (drivetrain)
    public boolean backCameraSeesTarget;
    public FiducialObservation[] backCameraFiducialObservations = new FiducialObservation[0];
    public MegatagPoseEstimate backCameraMegatagPoseEstimate;
    public int backCameraMegatagCount;
    public MegatagPoseEstimate backCameraMegatag2PoseEstimate;

    // Turret camera
    public boolean turretCameraSeesTarget;
    public FiducialObservation[] turretCameraFiducialObservations = new FiducialObservation[0];
    public MegatagPoseEstimate turretCameraMegatagPoseEstimate;
    public int turretCameraMegatagCount;
    public MegatagPoseEstimate turretCameraMegatag2PoseEstimate;
  }

  /** Update inputs from hardware */
  void readInputs(VisionIOInputs inputs);

  /** Poll NetworkTables for latest vision data (can be called at higher frequency) */
  void pollNetworkTables();
}
