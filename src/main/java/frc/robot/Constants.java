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
    public static final Pose2d TEST = new Pose2d(2.3, 4.5, Rotation2d.fromDegrees(0));

    // Auto-aim target positions (e.g., Hub)
    public static final Translation3d BLUE_AIM_TARGET = new Translation3d(4.625689, 4.040981, 0);
    public static final Translation3d RED_AIM_TARGET =
        new Translation3d(16.54175 - 4.625689, 4.040981, 0); // Mirrored across field
  }

  public static class Vision {
    // Limelight names
    public static final String FRONT_LIMELIGHT_NAME = "limelight-front";
    public static final String BACK_LIMELIGHT_NAME = "limelight-back";
    public static final String TURRET_LIMELIGHT_NAME = "limelight-turret";

    // Camera positions relative to robot center (Translation3d: x forward, y left, z up)
    // Front camera (on drivetrain, front of robot)
    public static final Transform3d FRONT_CAMERA_TO_ROBOT =
        new Transform3d(
            new Translation3d(0.25, 0.0, 0.25), // 25cm forward, 25cm up
            new Rotation3d(0, Units.degreesToRadians(-20), 0)); // Pitched down 20 degrees

    // Back camera (on drivetrain, back of robot)
    public static final Transform3d BACK_CAMERA_TO_ROBOT =
        new Transform3d(
            new Translation3d(-0.25, 0.0, 0.25), // 25cm backward, 25cm up
            new Rotation3d(
                0,
                Units.degreesToRadians(-20),
                Units.degreesToRadians(180))); // Pitched down, facing back

    // Turret camera (on turret, offset from turret center)
    // This is the transform from TURRET CENTER to camera
    public static final Transform3d TURRET_TO_CAMERA =
        new Transform3d(
            new Translation3d(0.15, 0.0, 0.30), // 15cm forward from turret axis, 30cm up
            new Rotation3d(0, Units.degreesToRadians(-15), 0)); // Pitched down 15 degrees

    // Turret position relative to robot center
    // This allows us to account for turret not being at robot center
    public static final Translation2d TURRET_TO_ROBOT_CENTER =
        new Translation2d(0.0, 0.0); // Adjust if turret is offset

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
      new LimelightCamera(FRONT_LIMELIGHT_NAME, FRONT_CAMERA_TO_ROBOT),
      new LimelightCamera(BACK_LIMELIGHT_NAME, BACK_CAMERA_TO_ROBOT),
    };

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (X, Y, Theta) - smaller values = trust vision more
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static class DriveConstants {
    // Maple-Sim Physics Simulation Configuration
    public static final boolean USE_MAPLE_SIM = true;
    public static final double ROBOT_WEIGHT_KILOGRAMS = 30.0;
    public static final double BUMPER_LENGTH_INCHES = 32.0;
    public static final double BUMPER_WIDTH_INCHES = 32.0;
    public static final int DRIVE_MOTOR_COUNT = 1;
    public static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.2;

    public static final double JOYSTICK_DEADBAND = 0.05;
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
      // This constraint is used in the driveToPose() function by PPLib AND PIDControl.
      public static final PathConstraints CONSTRAINTS =
          new PathConstraints(4, 4, 360, 360, 12, false); // 2.2

      // These constraints are solely used in the driveToPose() function by PIDControl.
      public static final double TRANSLATION_KP = 2.0; // Reduced from 5 to reduce oscillation
      public static final double TRANSLATION_KI = 0.0;
      public static final double TRANSLATION_KD = 0.2; // Increased to dampen oscillation
      public static final double TRANSLATION_TOLERANCE = 0.05;
      public static final double STATIC_FRICTION_CONSTANT = 1.2;
      public static final double MAX_VELOCITY = 3.0;

      public static final double ROTATION_KP = 5;
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
    public static final int MOTOR_CAN_ID = 60;
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
    public static final double HUB_SPEED = 60.0;
    public static final double PASS_SPEED = 30.0;

    // Velocity tolerance (for checking if at speed)
    public static final double VELOCITY_TOLERANCE = 2.0; // rotations per second
  }

  public static class IntakeConstants {
    // Motor CAN IDs
    public static final int UPPER_MOTOR_CAN_ID = 41;
    public static final int LOWER_MOTOR_CAN_ID = 42;
    public static final String CAN_BUS = "Superstructure";

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    // Intake percent output (0.0 to 1.0)
    public static final double INTAKE_PERCENT = 0.45; // 45% speed for intaking
    public static final double OUTTAKE_PERCENT = -0.5; // 50% speed for outtaking
  }

  public static class IntakePivotConstants {
    // Motor CAN ID
    public static final int MOTOR_CAN_ID = 43;
    public static final String CAN_BUS = "Superstructure";

    // Motor inversion (true = clockwise positive)
    public static final boolean MOTOR_INVERTED = true;

    // Motor encoder offset (in rotations)
    // Set this to the current position when the mechanism is at its zero/reference position
    public static final double MOTOR_ROTOR_OFFSET = 0.2;

    // Soft limits (in rotations)
    public static final double SOFT_LIMIT_REVERSE = 0.0; // Reverse soft limit (minimum position)
    public static final double SOFT_LIMIT_FORWARD = 28.0; // Forward soft limit (maximum position)

    // PID constants
    public static final double KP = 1.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;
    public static final double KG = 0.0;

    // Motion Magic constants
    public static final double CRUISE_VELOCITY = 100.0; // rotations per second
    public static final double ACCELERATION = 200.0; // rotations per second^2
    public static final double JERK = 2000.0; // rotations per second^3

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    // Position setpoints (in rotations)
    public static final double STOWED_POSITION = 0.0; // Stowed/up position
    public static final double DEPLOYED_POSITION = 27.6; // Extended/down position for intaking

    // Position tolerance
    public static final double POSITION_TOLERANCE = 0.5; // rotations
  }

  public static class ConveyorConstants {
    // Motor CAN ID
    public static final int MOTOR_CAN_ID = 45;
    public static final String CAN_BUS = "Drivetrain";

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    // Conveyor voltages
    public static final double TO_SHOOTER_VOLTAGE = 6.0; // Volts (positive = toward shooter)
    public static final double TO_BUCKET_VOLTAGE = -6.0; // Volts (negative = toward bucket)
  }

  public static class IndexerConstants {
    // Motor CAN ID
    public static final int MOTOR_CAN_ID = 47;
    public static final String CAN_BUS = "Drivetrain";

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    // Indexer voltages
    public static final double TO_SHOOTER_VOLTAGE = 8.0; // Volts (toward shooter)
  }

  public static class ClimbConstants {
    // Motor CAN ID
    public static final int MOTOR_CAN_ID = 46;
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

  public static class TurretConstants {
    // Motor CAN IDs
    public static final int MOTOR_CAN_ID = 50;
    public static final int CANCODER_CAN_ID = 51;
    public static final String CAN_BUS = "Superstructure";

    // Gear ratio (motor rotations to turret rotations)
    public static final double GEAR_RATIO = 100.0; // Example: 100:1 reduction

    // CANCoder configuration
    public static final double CANCODER_OFFSET = 0.0; // Adjust based on physical alignment

    // Position limits (radians)
    public static final double MIN_POSITION_RAD = -Math.PI; // -180 degrees
    public static final double MAX_POSITION_RAD = Math.PI; // 180 degrees

    // PID constants (tuned for position control)
    public static final double KP = 20.0;
    public static final double KI = 0.0;
    public static final double KD = 0.2;
    public static final double KS = 0.18;
    public static final double KV = 0.12;
    public static final double KA = 0.02;

    // Motion Magic constants
    public static final double CRUISE_VELOCITY = 120.0;
    public static final double ACCELERATION = 1200.0;
    public static final double JERK = 9000.0;

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 150.0;
    public static final double SUPPLY_CURRENT_LIMIT = 80.0;

    // Position tolerance for aiming
    public static final double AIMING_TOLERANCE_RAD = Units.degreesToRadians(2.0);

    // Latency compensation for aiming (seconds)
    // Accounts for control loop delay + motor response time
    // Start low and increase only if you see consistent lag
    public static final double AIMING_LATENCY_COMPENSATION =
        0.15; // Increased to 120ms for better tracking

    // Position setpoints (radians)
    public static final double STOW_POSITION = 0.0; // Forward
    public static final double SHOOT_BACK_BLUE_POSITION =
        Math.PI; // Backward (180°) for blue alliance
    public static final double SHOOT_BACK_RED_POSITION = 0.0; // Forward (0°) for red alliance
  }

  public static class HoodConstants {
    // Motor CAN ID
    public static final int MOTOR_CAN_ID = 52;
    public static final String CAN_BUS = "Superstructure";

    // Gear ratio (motor rotations to hood rotations)
    public static final double GEAR_RATIO = 60.0; // Example: 60:1 reduction

    // Position limits (radians from horizontal)
    public static final double MIN_POSITION_RAD = Units.degreesToRadians(0.0); // Flat
    public static final double MAX_POSITION_RAD = Units.degreesToRadians(60.0); // Max angle

    // PID constants
    public static final double KP = 5.0;
    public static final double KI = 0.0;
    public static final double KD = 0.1;
    public static final double KS = 0.1;
    public static final double KV = 0.12;
    public static final double KA = 0.01;
    public static final double KG = 0.2; // Gravity compensation

    // Motion Magic constants
    public static final double CRUISE_VELOCITY = 80.0; // rotations per second
    public static final double ACCELERATION = 160.0; // rotations per second^2
    public static final double JERK = 1600.0; // rotations per second^3

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    // Position tolerance for aiming
    public static final double AIMING_TOLERANCE_RAD = Units.degreesToRadians(1.0);

    // Position setpoints (radians)
    public static final double STOW_POSITION = Units.degreesToRadians(20.0); // Safe stow angle
    public static final double SHOOT_BACK_POSITION = Units.degreesToRadians(45.0); // Mid-field shot
    public static final double MIN_AIM_ANGLE = Units.degreesToRadians(15.0); // Minimum angle
  }

  /** APF (Artificial Potential Field) Constants for autonomous ball intake navigation */
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
