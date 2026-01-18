// LimelightHelpers v1.12 (Minimal version for MegaTag2 pose estimation)
// Full version: https://github.com/LimelightVision/limelightlib-wpijava

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Timer;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class LimelightHelpers {

  private static final Map<String, DoubleArrayEntry> doubleArrayEntries = new ConcurrentHashMap<>();

  public static class RawFiducial {
    public int id = 0;
    public double txnc = 0;
    public double tync = 0;
    public double ta = 0;
    public double distToCamera = 0;
    public double distToRobot = 0;
    public double ambiguity = 0;

    public RawFiducial(
        int id,
        double txnc,
        double tync,
        double ta,
        double distToCamera,
        double distToRobot,
        double ambiguity) {
      this.id = id;
      this.txnc = txnc;
      this.tync = tync;
      this.ta = ta;
      this.distToCamera = distToCamera;
      this.distToRobot = distToRobot;
      this.ambiguity = ambiguity;
    }
  }

  public static class PoseEstimate {
    public Pose2d pose;
    public double timestampSeconds;
    public double latency;
    public int tagCount;
    public double tagSpan;
    public double avgTagDist;
    public double avgTagArea;
    public RawFiducial[] rawFiducials;
    public boolean isMegaTag2;

    public PoseEstimate() {
      this.pose = new Pose2d();
      this.timestampSeconds = 0;
      this.latency = 0;
      this.tagCount = 0;
      this.tagSpan = 0;
      this.avgTagDist = 0;
      this.avgTagArea = 0;
      this.rawFiducials = new RawFiducial[] {};
      this.isMegaTag2 = false;
    }

    public PoseEstimate(
        Pose2d pose,
        double timestampSeconds,
        double latency,
        int tagCount,
        double tagSpan,
        double avgTagDist,
        double avgTagArea,
        RawFiducial[] rawFiducials,
        boolean isMegaTag2) {
      this.pose = pose;
      this.timestampSeconds = timestampSeconds;
      this.latency = latency;
      this.tagCount = tagCount;
      this.tagSpan = tagSpan;
      this.avgTagDist = avgTagDist;
      this.avgTagArea = avgTagArea;
      this.rawFiducials = rawFiducials;
      this.isMegaTag2 = isMegaTag2;
    }
  }

  static final String sanitizeName(String name) {
    if ("".equals(name) || name == null) {
      return "limelight";
    }
    return name;
  }

  public static Pose2d toPose2D(double[] inData) {
    if (inData.length < 6) {
      return new Pose2d();
    }
    Translation2d tran2d = new Translation2d(inData[0], inData[1]);
    Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
    return new Pose2d(tran2d, r2d);
  }

  private static double extractArrayEntry(double[] inData, int position) {
    if (inData.length < position + 1) {
      return 0;
    }
    return inData[position];
  }

  public static NetworkTable getLimelightNTTable(String tableName) {
    return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
  }

  public static DoubleArrayEntry getLimelightDoubleArrayEntry(String tableName, String entryName) {
    String key = tableName + "/" + entryName;
    return doubleArrayEntries.computeIfAbsent(
        key,
        k -> {
          NetworkTable table = getLimelightNTTable(tableName);
          return table.getDoubleArrayTopic(entryName).getEntry(new double[0]);
        });
  }

  private static PoseEstimate getBotPoseEstimate(
      String limelightName, String entryName, boolean isMegaTag2) {
    DoubleArrayEntry poseEntry = getLimelightDoubleArrayEntry(limelightName, entryName);

    TimestampedDoubleArray tsValue = poseEntry.getAtomic();
    double[] poseArray = tsValue.value;
    long timestamp = tsValue.timestamp;

    if (poseArray.length == 0) {
      return null;
    }

    var pose = toPose2D(poseArray);
    double latency = extractArrayEntry(poseArray, 6);
    int tagCount = (int) extractArrayEntry(poseArray, 7);
    double tagSpan = extractArrayEntry(poseArray, 8);
    double tagDist = extractArrayEntry(poseArray, 9);
    double tagArea = extractArrayEntry(poseArray, 10);

    // Convert server timestamp from microseconds to seconds and adjust for latency
    // Use FPGA timestamp as fallback if NT timestamp is not available
    double adjustedTimestamp;
    if (timestamp != 0) {
      adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);
    } else {
      adjustedTimestamp = Timer.getFPGATimestamp() - (latency / 1000.0);
    }

    RawFiducial[] rawFiducials = new RawFiducial[tagCount];
    int valsPerFiducial = 7;
    int expectedTotalVals = 11 + valsPerFiducial * tagCount;

    if (poseArray.length != expectedTotalVals) {
      // Don't populate fiducials
    } else {
      for (int i = 0; i < tagCount; i++) {
        int baseIndex = 11 + (i * valsPerFiducial);
        int id = (int) poseArray[baseIndex];
        double txnc = poseArray[baseIndex + 1];
        double tync = poseArray[baseIndex + 2];
        double ta = poseArray[baseIndex + 3];
        double distToCamera = poseArray[baseIndex + 4];
        double distToRobot = poseArray[baseIndex + 5];
        double ambiguity = poseArray[baseIndex + 6];
        rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
      }
    }

    return new PoseEstimate(
        pose,
        adjustedTimestamp,
        latency,
        tagCount,
        tagSpan,
        tagDist,
        tagArea,
        rawFiducials,
        isMegaTag2);
  }

  public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag1(String limelightName) {
    return getBotPoseEstimate(limelightName, "botpose_wpiblue", false);
  }

  public static PoseEstimate getBotPoseEstimate_wpiRed_MegaTag1(String limelightName) {
    return getBotPoseEstimate(limelightName, "botpose_wpired", false);
  }

  public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String limelightName) {
    return getBotPoseEstimate(limelightName, "botpose_orb_wpiblue", true);
  }

  public static PoseEstimate getBotPoseEstimate_wpiRed_MegaTag2(String limelightName) {
    return getBotPoseEstimate(limelightName, "botpose_orb_wpired", true);
  }

  public static Boolean validPoseEstimate(PoseEstimate pose) {
    return pose != null && pose.rawFiducials != null && pose.rawFiducials.length != 0;
  }

  public static void SetRobotOrientation(
      String limelightName,
      double yaw,
      double yawRate,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    double[] entries = new double[6];
    entries[0] = yaw;
    entries[1] = yawRate;
    entries[2] = pitch;
    entries[3] = pitchRate;
    entries[4] = roll;
    entries[5] = rollRate;
    getLimelightNTTable(limelightName).getEntry("robot_orientation_set").setDoubleArray(entries);
    // Removed flush() - NetworkTables sends automatically, flush() blocks for ~10-20ms
  }

  /**
   * Set the Limelight's LED mode
   *
   * @param limelightName Name of the limelight
   * @param mode 0=pipeline default, 1=off, 2=blink, 3=on
   */
  public static void setLEDMode(String limelightName, int mode) {
    getLimelightNTTable(limelightName).getEntry("ledMode").setNumber(mode);
  }

  /**
   * Set the Limelight's active pipeline
   *
   * @param limelightName Name of the limelight
   * @param pipeline Pipeline index (0-9)
   */
  public static void setPipelineIndex(String limelightName, int pipeline) {
    getLimelightNTTable(limelightName).getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Get current pipeline index
   *
   * @param limelightName Name of the limelight
   * @return Current pipeline index
   */
  public static int getPipelineIndex(String limelightName) {
    return (int) getLimelightNTTable(limelightName).getEntry("getpipe").getDouble(0);
  }

  /**
   * Check if limelight has a valid target
   *
   * @param limelightName Name of the limelight
   * @return true if target detected
   */
  public static boolean hasTarget(String limelightName) {
    return getLimelightNTTable(limelightName).getEntry("tv").getDouble(0) == 1.0;
  }

  /**
   * Get horizontal offset from crosshair to target (degrees)
   *
   * @param limelightName Name of the limelight
   * @return Horizontal offset in degrees
   */
  public static double getTX(String limelightName) {
    return getLimelightNTTable(limelightName).getEntry("tx").getDouble(0);
  }

  /**
   * Get vertical offset from crosshair to target (degrees)
   *
   * @param limelightName Name of the limelight
   * @return Vertical offset in degrees
   */
  public static double getTY(String limelightName) {
    return getLimelightNTTable(limelightName).getEntry("ty").getDouble(0);
  }

  /**
   * Get target area (0-100% of image)
   *
   * @param limelightName Name of the limelight
   * @return Target area percentage
   */
  public static double getTA(String limelightName) {
    return getLimelightNTTable(limelightName).getEntry("ta").getDouble(0);
  }

  /**
   * Get pipeline latency (ms)
   *
   * @param limelightName Name of the limelight
   * @return Pipeline latency in milliseconds
   */
  public static double getLatency(String limelightName) {
    return getLimelightNTTable(limelightName).getEntry("tl").getDouble(0);
  }

  /** Neural detector target data */
  public static class NeuralTarget {
    public final double tx; // Horizontal offset (degrees)
    public final double ty; // Vertical offset (degrees)
    public final double ta; // Area (0-100)
    public final int classId; // Class ID from neural network
    public final double confidence; // Detection confidence (0-1)

    public NeuralTarget(double tx, double ty, double ta, int classId, double confidence) {
      this.tx = tx;
      this.ty = ty;
      this.ta = ta;
      this.classId = classId;
      this.confidence = confidence;
    }
  }

  /**
   * Get all neural detector targets from the limelight Returns data from the "botpose_wpiblue"
   * array which contains neural detector results
   *
   * @param limelightName Name of the limelight
   * @return Array of detected neural targets (empty if none detected)
   */
  public static NeuralTarget[] getNeuralTargets(String limelightName) {
    NetworkTable table = getLimelightNTTable(limelightName);

    // Check if any targets detected
    if (!hasTarget(limelightName)) {
      return new NeuralTarget[0];
    }

    // Get the detector results from NetworkTables
    // Limelight publishes neural detector data in "tx0", "ty0", "ta0", "tclass0" arrays
    double[] txArray = table.getEntry("tx0").getDoubleArray(new double[0]);
    double[] tyArray = table.getEntry("ty0").getDoubleArray(new double[0]);
    double[] taArray = table.getEntry("ta0").getDoubleArray(new double[0]);
    double[] classArray = table.getEntry("tclass").getDoubleArray(new double[0]);

    // Find the minimum length (all arrays should be same length but be safe)
    int numTargets =
        Math.min(
            Math.min(txArray.length, tyArray.length), Math.min(taArray.length, classArray.length));

    if (numTargets == 0) {
      return new NeuralTarget[0];
    }

    // Build target array with confidence = 1.0 (Limelight doesn't expose confidence separately)
    NeuralTarget[] targets = new NeuralTarget[numTargets];
    for (int i = 0; i < numTargets; i++) {
      targets[i] =
          new NeuralTarget(
              txArray[i],
              tyArray[i],
              taArray[i],
              (int) classArray[i],
              1.0 // Limelight filters by confidence internally
              );
    }

    return targets;
  }
}
