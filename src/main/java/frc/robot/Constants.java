// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class FieldPoses {
    public static final double FIELD_LENGTH = 16.54; // meters
    public static final double FIELD_WIDTH = 8.07; // meters

    // Field center point (for ball search fallback)
    public static final Translation2d FIELD_CENTER =
        new Translation2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0);

    // Common field positions for drive-to-pose
    // Adjust these based on your field and desired positions
    public static final Pose2d TEST = new Pose2d(3.75, 5.3, Rotation2d.fromDegrees(-60));

    // Auto-aim target positions (e.g., Speaker)
    public static final Translation3d BLUE_AIM_TARGET = new Translation3d(4.625689, 4.040981, 0);
    public static final Translation3d RED_AIM_TARGET =
        new Translation3d(16.54175 - 4.625689, 4.040981, 0); // Mirrored across field
  }

  public static class Vision {
    // Limelight Configuration (supports one or multiple cameras)
    public static class LimelightCamera {
      public final String name;
      public final Transform3d robotToCamera;

      public LimelightCamera(String name, Transform3d robotToCamera) {
        this.name = name;
        this.robotToCamera = robotToCamera;
      }
    }

    // Add your Limelights here - example with front and back cameras
    public static final LimelightCamera[] LIMELIGHT_CAMERAS = {
      new LimelightCamera(
          "limelight-front",
          new Transform3d(
              new Translation3d(0.324339, 0, 0.1337), new Rotation3d(0, Math.toRadians(10), 0))),
    };

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect correction rate
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static class DriveConstants {
    // Maple-Sim Physics Simulation Configuration
    public static final boolean USE_MAPLE_SIM = true;
    public static final double ROBOT_WEIGHT_POUNDS = 150.0;
    public static final double BUMPER_LENGTH_INCHES = 35.625;
    public static final double BUMPER_WIDTH_INCHES = 35.625;
    public static final int DRIVE_MOTOR_COUNT = 1;
    public static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.2;

    public static final double JOYSTICK_DEADBAND = 0.03;
    public static final LinearVelocity JOYSTICK_POV_VELOCITY = MetersPerSecond.of(0.2);

    public static final PPHolonomicDriveController PP_HOLONOMIC_DRIVE_CONTROLLER =
        new PPHolonomicDriveController(
            // PPHolonomicController is the built in path following controller for holonomic drive
            // trains.
            // This does not affect DriveToPose.
            new PIDConstants(3.5, 0.0, 0.0), // Translation PID constants500
            new PIDConstants(2.8, 0.0, 0.0) // Rotation PID constants500
            );

    public static class DriveToPose {
      // Whether to use PPLib in the driveToPose() function.
      public static final boolean USE_PPLIB = false;
      // This constraint is used in the driveToPose() function by PPLib AND PIDControl.
      public static final PathConstraints CONSTRAINTS =
          new PathConstraints(4, 4, 360, 360, 12, false); // 2.2

      // These constraints are solely used in the driveToPose() function by PIDControl.
      public static final double TRANSLATION_KP = 3;
      public static final double TRANSLATION_KI = 0.0;
      public static final double TRANSLATION_KD = 0.1;
      public static final double TRANSLATION_TOLERANCE = 0.02;
      public static final double STATIC_FRICTION_CONSTANT = 0.02;
      public static final double MAX_VELOCITY = 3.0;

      public static final double ROTATION_KP = 3;
      public static final double ROTATION_KI = 0.0;
      public static final double ROTATION_KD = 0.1;
      public static final double ROTATION_TOLERANCE = Units.degreesToRadians(1);
    }

    public static class AutoAim {
      // PID constants for auto-aiming at a target while driving
      public static final double HEADING_KP = 5.0;
      public static final double HEADING_KI = 0.0;
      public static final double HEADING_KD = 0.1;

      // Feedforward to compensate for changing angle while driving
      // Higher value = more aggressive prediction of needed rotation
      public static final double HEADING_KV = 0.8;
    }
  }

  public static class ShooterConstants {
    // Motor CAN ID
    public static final int MOTOR_CAN_ID = 41;
    public static final String CAN_BUS = "Drivetrain";

    // Gear ratio (motor rotations to shooter wheel rotations)
    public static final double GEAR_RATIO = 3.0;

    // PID and feedforward constants
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;
    public static final double KP = 1.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;

    // Motion Magic constants
    public static final double CRUISE_VELOCITY = 80.0; // rotations per second
    public static final double ACCELERATION = 160.0; // rotations per second^2
    public static final double JERK = 1600.0; // rotations per second^3

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 80.0;
    public static final double SUPPLY_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Shooter speed presets (in rotations per second)
    public static final double IDLE_SPEED = 0.0;
    public static final double SPEAKER_SPEED = 60.0;
    public static final double AMP_SPEED = 30.0;

    // Velocity tolerance (for checking if at speed)
    public static final double VELOCITY_TOLERANCE = 2.0; // rotations per second
  }

  public static class IntakeConstants {
    // Motor CAN ID
    public static final int MOTOR_CAN_ID = 42;
    public static final String CAN_BUS = "Drivetrain";

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    // Intake voltage
    public static final double INTAKE_VOLTAGE = 8.0; // Volts (positive = intake)
  }

  public static class IntakePivotConstants {
    // Motor CAN ID
    public static final int MOTOR_CAN_ID = 43;
    public static final String CAN_BUS = "Drivetrain";

    // PID constants
    public static final double KP = 15.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;

    // Motion Magic constants
    public static final double CRUISE_VELOCITY = 30.0; // rotations per second
    public static final double ACCELERATION = 60.0; // rotations per second^2
    public static final double JERK = 600.0; // rotations per second^3

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    // Position setpoints (in rotations)
    public static final double RETRACTED_POSITION = 0.0; // Stowed/up position
    public static final double DEPLOYED_POSITION = 10.0; // Extended/down position for intaking

    // Position tolerance
    public static final double POSITION_TOLERANCE = 0.5; // rotations
  }

  public static class ConveyorConstants {
    // Motor CAN ID
    public static final int MOTOR_CAN_ID = 44;
    public static final String CAN_BUS = "Drivetrain";

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    // Conveyor voltages
    public static final double TO_SHOOTER_VOLTAGE = 6.0; // Volts (positive = toward shooter)
    public static final double TO_BUCKET_VOLTAGE = -6.0; // Volts (negative = toward bucket/amp)
  }

  public static class ClimbConstants {
    // Motor CAN ID
    public static final int MOTOR_CAN_ID = 45;
    public static final String CAN_BUS = "rio";

    // PID constants
    public static final double KP = 10.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;

    // Motion Magic constants
    public static final double CRUISE_VELOCITY = 40.0; // rotations per second
    public static final double ACCELERATION = 80.0; // rotations per second^2
    public static final double JERK = 800.0; // rotations per second^3

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 80.0;
    public static final double SUPPLY_CURRENT_LIMIT = 60.0;

    // Position setpoints (in rotations)
    public static final double STOWED_POSITION = 0.0; // Starting position
    public static final double EXTENDED_POSITION = 50.0; // Fully extended
    public static final double RETRACTED_POSITION = 5.0; // Pulled up on chain

    // Position tolerance
    public static final double POSITION_TOLERANCE = 1.0; // rotations
  }

  /** Ball Vision Constants Configuration for Limelight-based ball detection */
  public static class BallVision {
    // Ball physical properties
    public static final double BALL_DIAMETER_METERS = 0.15; // 15cm diameter ball

    // Camera calibration
    public static final double AREA_CALIBRATION_CONSTANT = 10.0; // Adjust based on camera testing

    // Ball detection thresholds
    public static final double MIN_CONFIDENCE = 0.5; // Minimum confidence to consider a detection
    public static final double MAX_BALL_DISTANCE = 8.0; // Maximum distance to detect balls (meters)
  }

  /** Navigation Constants Configuration for ball intake navigation and path planning */
  public static class NavigationConstants {
    // Slew rate limits for smooth acceleration
    public static final double TRANSLATION_RATE_LIMIT = 3.0; // m/s^2
    public static final double ROTATION_RATE_LIMIT = 5.0; // rad/s^2

    // Search behavior when no balls detected
    public static final double SEARCH_DRIVE_SPEED = 1.0; // m/s towards field center
    public static final double SEARCH_HEADING_TOLERANCE =
        Math.toRadians(30); // Only drive when facing target
    public static final double FIELD_CENTER_STOP_DISTANCE =
        1.0; // Stop when within 1m of field center

    // Target persistence (prevents losing ball when out of view)
    public static final int BALL_PERSISTENCE_CYCLES =
        25; // Keep driving to last position for 0.5s @ 50Hz

    // Target switching threshold
    public static final double PRIORITY_SWITCH_THRESHOLD =
        50.0; // Only switch if new priority is 50+ points better
    public static final double SAME_BALL_TOLERANCE =
        0.1; // Distance to consider balls the same (meters)

    // PID blending factors
    public static final double PID_TRANSLATION_BLEND =
        0.3; // How much PID correction to blend with APF (30%)
    public static final double PID_ROTATION_BLEND =
        0.2; // How much PID correction to blend with APF (20%)
    public static final double SECONDARY_ATTRACTION_FACTOR =
        0.3; // Influence of secondary balls (30%)
  }

  /**
   * APF (Artificial Potential Field) Constants public static final double BALL_DIAMETER_METERS =
   * 0.15; // 15cm diameter (7.5cm radius)
   *
   * <p>// Camera configuration // Example: camera mounted at front of robot, 0.5m forward, 0.3m
   * high, tilted 20 degrees down public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
   * new Translation3d(0.5, 0.0, 0.3), // 0.5m forward, 0.3m high new Rotation3d(0,
   * Math.toRadians(-20), 0) // 20 degrees down );
   *
   * <p>// Detection thresholds public static final double MIN_CONFIDENCE_THRESHOLD = 0.5; //
   * Minimum detection confidence (0-1) public static final double MAX_DETECTION_DISTANCE = 5.0; //
   * Maximum distance to detect balls (meters)
   *
   * <p>// Area calibration // This constant relates ball area to distance // Tune this based on
   * real measurements: measure ball area % at known distances // Formula: area_at_1m =
   * (focal_length * ball_diameter / sensor_size)^2 public static final double
   * AREA_CALIBRATION_CONSTANT = 10.0; // Adjust based on calibration
   *
   * <p>// Ball class IDs (depends on your neural network training) public static final int
   * RED_BALL_CLASS = 0; public static final int BLUE_BALL_CLASS = 1; }
   *
   * <p>/** APF (Artificial Potential Field) Constants Configuration for autonomous ball intake
   * navigation
   */
  /**
   * APF (Artificial Potential Field) Constants Configuration for weighted ball priority and force
   * field navigation
   */
  public static class APFConstants {
    // Priority weights
    public static final double DISTANCE_WEIGHT = 100.0; // Weight for distance-based priority
    public static final double DISTANCE_DECAY_FACTOR =
        2.0; // Exponential decay for distance (meters)
    public static final double CLUSTER_WEIGHT = 50.0; // Bonus per additional ball in cluster
    public static final double ALLIANCE_BONUS = 30.0; // Bonus for our alliance balls
    public static final double CONFIDENCE_EXPONENT = 2.0; // Confidence penalty exponent

    // Distance thresholds
    public static final double FAR_BALL_THRESHOLD =
        4.0; // Distance where far penalty starts (meters)
    public static final double FAR_BALL_PENALTY = 20.0; // Penalty per meter beyond threshold

    // Clustering parameters
    public static final double CLUSTER_RADIUS =
        0.8; // Maximum distance between balls in cluster (meters)
    public static final int MIN_CLUSTER_SIZE = 2; // Minimum balls to form a cluster

    // Force field parameters
    public static final double ATTRACTIVE_GAIN = 1.5; // Gain for attractive force toward target
    public static final double MAX_ATTRACTIVE_FORCE = 3.0; // Maximum attractive force magnitude

    // Secondary ball attraction (for strategic clustering flow)
    public static final double SECONDARY_BALL_ATTRACTION =
        0.8; // Weak attraction from non-target balls
    public static final double SECONDARY_BALL_DECAY =
        1.5; // Rapid decay for secondary attractions (meters)

    // Wall repulsion
    public static final double WALL_REPULSION_DISTANCE =
        1.0; // Distance from wall where repulsion starts (meters)
    public static final double WALL_REPULSION_GAIN = 0.5; // Gain for repulsive force from walls

    // Velocity limits
    public static final double MAX_LINEAR_VELOCITY = 3.0; // Maximum linear velocity (m/s)
    public static final double MAX_ANGULAR_VELOCITY = 2.0; // Maximum angular velocity (rad/s)
    public static final double HEADING_GAIN = 3.0; // Proportional gain for heading control
  }
}
