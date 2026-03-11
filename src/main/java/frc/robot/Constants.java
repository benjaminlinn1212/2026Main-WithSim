// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/** AdvantageKit runtime mode. Change simMode to switch between SIM and REPLAY. */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /** Whether to write .wpilog files to USB/roboRIO. */
  public static final boolean ENABLE_FILE_LOGGING = false;

  public static final CANBus SUPERSTRUCTURE_CAN_BUS = new CANBus("Superstructure");

  // Odometry proximity check thresholds (meters, degrees)
  public static final double ODOMETRY_CLOSE_TRANSLATION_METERS = 0.25;
  public static final double ODOMETRY_CLOSE_ROTATION_DEGREES = 8.0;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Vision {
    public static final String FRONT_LIMELIGHT_NAME = "limelight-right";
    public static final String TURRET_LIMELIGHT_NAME = "limelight-turret";

    // Front camera (on drivetrain)
    public static final Transform3d RIGHT_CAMERA_TO_ROBOT =
        new Transform3d(
            new Translation3d(0.318704, 0.183753, 0.292216),
            new Rotation3d(0, Units.degreesToRadians(20), 0));

    // Turret camera (offset from turret center)
    public static final Transform3d TURRET_TO_CAMERA =
        new Transform3d(
            new Translation3d(0.185643, 0.0, 0.52803),
            new Rotation3d(0, Units.degreesToRadians(15), 0));

    public static final Translation2d ROBOT_TO_TURRET =
        TurretConstants.TURRET_OFFSET_FROM_ROBOT_CENTER;

    // Angular velocity rejection thresholds (rad/s) for turret camera vision.
    // Frames captured while turret or chassis spins faster than this are rejected.
    public static final double MAX_TURRET_ANGULAR_VELOCITY_FOR_VISION =
        Units.degreesToRadians(200.0);
    public static final double MAX_DRIVE_ANGULAR_VELOCITY_FOR_VISION =
        Units.degreesToRadians(200.0);
    /** Lookback window (seconds) for peak angular velocity check. */
    public static final double VELOCITY_REJECTION_LOOKBACK = 0.10;
  }

  public static class AutoConstants {
    // Path following PID controllers
    public static final double PATH_FOLLOWING_TRANSLATION_KP = 3.0;
    public static final double PATH_FOLLOWING_ROTATION_KP = 8.0;

    // PathPlanner pathfinding constraints
    public static final double PATHFINDING_MAX_VELOCITY_MPS = 3.0;
    public static final double PATHFINDING_MAX_ACCELERATION_MPS2 = 4.0;
    public static final double PATHFINDING_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.toRadians(720);
    public static final double PATHFINDING_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC2 =
        Math.toRadians(900);

    // Climb straight-line approach
    /** Distance (m) from climb target to switch from pathfinding to straight-line drive. */
    public static final double CLIMB_APPROACH_DISTANCE_M = 0.2;
    /** Max velocity (m/s) for straight-line approach PID. */
    public static final double CLIMB_APPROACH_MAX_VELOCITY_MPS = 3.0;
    /** Position tolerance (m). */
    public static final double CLIMB_APPROACH_TOLERANCE_M = 0.01;
    /** Heading tolerance (rad). */
    public static final double CLIMB_APPROACH_THETA_TOLERANCE_RAD = Math.toRadians(1.0);

    // DriveToPoint PID gains (auto vs teleop)
    public static final double DRIVE_TO_POINT_AUTO_KP = 3.0;
    public static final double DRIVE_TO_POINT_AUTO_KD = 0.1;
    public static final double DRIVE_TO_POINT_TELEOP_KP = 3.0;
    public static final double DRIVE_TO_POINT_TELEOP_KD = 0.1;
    /** Heading kP for CTRE FieldCentricFacingAngle (runs at 250 Hz in odometry thread). */
    public static final double DRIVE_TO_POINT_HEADING_KP = 5.0;
    /** Static friction FF multiplier -- minimum velocity to overcome drivetrain friction. */
    public static final double DRIVE_TO_POINT_FRICTION_FF = 0.02;
    /** Default max velocity (m/s). */
    public static final double DRIVE_TO_POINT_DEFAULT_MAX_VELOCITY_MPS = 3.0;
    /** Translation tolerance (m). */
    public static final double DRIVE_TO_POINT_POSITION_TOLERANCE_M = Units.inchesToMeters(1.0);
    /** Heading tolerance (rad). */
    public static final double DRIVE_TO_POINT_HEADING_TOLERANCE_RAD = Math.toRadians(2.0);

    /**
     * Path distance derating (0-1). AD* paths are ~11% longer than straight-line due to curves.
     * Trapezoidal model handles accel/decel, so this only covers curvature.
     */
    public static final double PATH_DISTANCE_DERATING = 0.9;

    public static final edu.wpi.first.math.geometry.Pose2d DEFAULT_RESET_POSE =
        new edu.wpi.first.math.geometry.Pose2d(
            0.3, 0.3, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));

    /**
     * Auto trench approach buffer (meters). Tighter than teleop's buffer to avoid premature
     * snapping that interferes with PathPlanner's planned heading.
     */
    public static final double TRENCH_APPROACH_BUFFER = 0.5;
  }

  public static class DriveConstants {
    // Maple-Sim Configuration
    public static final boolean USE_MAPLE_SIM = true;
    public static final double ROBOT_WEIGHT_KILOGRAMS = 30.0;
    public static final double BUMPER_LENGTH_INCHES = 32.0;
    public static final double BUMPER_WIDTH_INCHES = 35.055;
    public static final int DRIVE_MOTOR_COUNT = 1;
    public static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.2;

    // Teleop Speed Limits
    public static final double MAX_TELEOP_SPEED_MPS = 5.0;
    public static final double MAX_TELEOP_ANGULAR_SPEED_RAD_PER_SEC = Math.PI * 2;

    public static final double JOYSTICK_DEADBAND = 0.0;

    /**
     * Trench Teleop Assist tuning. When the robot is near a trench, two effects activate:
     *
     * <ol>
     *   <li><b>Chassis orientation alignment</b> -- injects omega (rotational velocity) to LERP the
     *       robot's heading toward the nearest cardinal direction, so the bumpers fit through the
     *       22.25in tunnel. This modifies {@code omega}, not the velocity direction.
     *   <li><b>Lateral centering</b> -- deflects the velocity vector toward the trench's center Y
     *       line, guiding the travel path toward the middle of the 48in corridor. Speed magnitude
     *       is preserved.
     * </ol>
     *
     * Both effects ramp from 0 at the buffer edge to full strength inside the trench.
     */
    public static class TrenchAssist {

      public static final boolean ENABLED = true;

      /** Max blend factor (0-1). */
      public static final double MAX_BLEND_FACTOR = 0.6;

      /** How far outside trench walls (meters) the assist begins ramping. */
      public static final double APPROACH_BUFFER = 1.5;

      /** Min speed (m/s) below which assist is inactive. */
      public static final double MIN_SPEED_MPS = 2.0;

      /** Max angle (degrees) between velocity vector and X axis for assist to activate. */
      public static final double MAX_HEADING_ERROR_DEG = 30.0;

      // Orientation PID (heading -> omega)
      public static final double ORIENTATION_KP = 10.0;
      public static final double ORIENTATION_KI = 0.0;
      public static final double ORIENTATION_KD = 0.15;

      /** Max omega correction (rad/s). */
      public static final double MAX_ORIENTATION_OMEGA_RAD_PER_SEC = 8.0;

      // Lateral Centering PID (Y-offset -> vy correction)

      public static final double LATERAL_KP = 3.0;
      public static final double LATERAL_KI = 0.0;
      public static final double LATERAL_KD = 0.1;
      /** Max lateral vy correction (m/s). */
      public static final double MAX_LATERAL_CORRECTION_MPS = 1.5;

      /** Half bumper width (meters). */
      public static final double ROBOT_HALF_WIDTH_M =
          Units.inchesToMeters(BUMPER_WIDTH_INCHES / 2.0);

      /** Hood stow buffer (meters) -- tighter than assist buffer. */
      public static final double HOOD_STOW_BUFFER = 0.5;
    }
  }

  public static class IntakeConstants {
    // Hardware Configuration
    public static final int UPPER_MOTOR_CAN_ID = 41;
    public static final int LOWER_MOTOR_CAN_ID = 42;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
    public static final double GEAR_RATIO = 1.0;
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

    // Upper Roller PID and Feedforward
    public static final double UPPER_KP = 0.1;
    public static final double UPPER_KI = 0.0;
    public static final double UPPER_KD = 0.0;
    public static final double UPPER_KS = 0.0;
    public static final double UPPER_KV = 0.12;
    public static final double UPPER_KA = 0.0;

    // Lower Roller PID and Feedforward
    public static final double LOWER_KP = 0.1;
    public static final double LOWER_KI = 0.0;
    public static final double LOWER_KD = 0.0;
    public static final double LOWER_KS = 0.0;
    public static final double LOWER_KV = 0.12;
    public static final double LOWER_KA = 0.0;

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Velocity Setpoints (rotations per second)
    public static final double UPPER_INTAKE_VELOCITY_RPS = 67.0;
    public static final double LOWER_INTAKE_VELOCITY_RPS = 15.0;
    public static final double LOWER_INTAKE_EJECT_RPS = -20.0;
  }

  public static class IntakePivotConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 43;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
    public static final double GEAR_RATIO = 1.0;
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

    // Soft Limits (rotations)
    public static final double SOFT_LIMIT_REVERSE = 0.0;
    public static final double SOFT_LIMIT_FORWARD = 28.0;

    // PID and Feedforward
    public static final double KP = 1.3;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.05;
    public static final double KV = 0.0;
    public static final double KA = 0.0;

    // Motion Magic Constants
    public static final double CRUISE_VELOCITY = 100.0;
    public static final double ACCELERATION = 300.0;
    public static final double JERK = 4000.0;

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Position Setpoints (rotations)
    public static final double STOWED_POSITION = 0.0;
    public static final double OUTPOST_POSITION = 7.784;
    public static final double OUTPOST_JIGGLE_POSITION = 4.784;
    public static final double DEPLOYED_POSITION = 27.0;

    // Jiggle positions -- pivot alternates between these two during feeding to dislodge FUEL
    public static final double JIGGLE_POSITION_A = 23.0;
    public static final double JIGGLE_POSITION_B = 26.0;
    /** Seconds per half-cycle of the jiggle oscillation. */
    public static final double JIGGLE_PERIOD_SECONDS = 0.5;

    // Position Tolerance
    public static final double POSITION_TOLERANCE = 0.1;
  }

  public static class ConveyorConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 45;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
    public static final double GEAR_RATIO = 1.0;
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

    // PID and Feedforward (Slot 0 -- VelocityVoltage)
    public static final double KP = 0.5;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.15;
    public static final double KV = 0.12;
    public static final double KA = 0.0;

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Feed velocity setpoint (rotations per second)
    public static final double FEED_VELOCITY_RPS = 40.0;
  }

  public static class IndexerConstants {
    // Hardware Configuration
    public static final int LEADER_MOTOR_CAN_ID = 47;
    public static final int FOLLOWER_MOTOR_CAN_ID = 48;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
    public static final double GEAR_RATIO = 1.0;
    public static final InvertedValue LEADER_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue FOLLOWER_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

    // PID and Feedforward (unused for duty cycle control)
    public static final double KP = 0.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;
    public static final double KG = 0.0;

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Indexer Percent Output
    public static final double TO_SHOOTER_PERCENT = 0.75;
  }

  public static class Aiming {
    public static final double MIN_SHOT_DISTANCE = 1.0; // meters
    public static final double MAX_SHOT_DISTANCE = 6.0; // meters
    public static final double PHASE_DELAY = 0.03; // seconds

    public static final int FEEDFORWARD_FILTER_TAPS = 5;

    // Neutral Zone Feed Shot -- physics-based (no interp maps)
    /** Hood angle (degrees) for neutral zone lob shots. */
    public static final double NEUTRAL_ZONE_HOOD_ANGLE_DEG = 45.0;

    /** Launch height above ground (meters). */
    public static final double NEUTRAL_ZONE_LAUNCH_HEIGHT_M = 0.55;

    /**
     * Flywheel RPS per m/s of required launch speed. Increase if balls fall short, decrease if
     * overshoot.
     */
    public static final double NEUTRAL_ZONE_RPS_PER_MPS = 8.0;
  }

  public static class TurretConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 44;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
    public static final double GEAR_RATIO = 1.0 / 26.812; // mechanism rotations per motor rotation
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

    // Turret offset from robot center (m)
    public static final Translation2d TURRET_OFFSET_FROM_ROBOT_CENTER =
        new Translation2d(0.1909, 0.0);

    // Position limits (rad)
    public static final double MIN_POSITION_RAD = Units.degreesToRadians(-300.0);
    public static final double MAX_POSITION_RAD = Units.degreesToRadians(110.0);

    // PID and Feedforward (MotionMagicVoltage)
    public static final double KP = 10.0;
    public static final double KI = 0.0;
    public static final double KD = 0.3;
    public static final double KS = 0.25;
    public static final double KV = 0.12;
    public static final double KA = 0.0;
    public static final double KG = 0.0;

    // Motion Magic Constants (motor rotations per second)
    public static final double CRUISE_VELOCITY = 90;
    public static final double ACCELERATION = 360;
    public static final double JERK = 2000;

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 150.0;
    public static final double SUPPLY_CURRENT_LIMIT = 80.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Position Tolerance (radians)
    public static final double AIMING_TOLERANCE_RAD = Units.degreesToRadians(1.0);

    // Boot position (rad) -- encoder seeded to this at boot via motor.setPosition()
    public static final double BOOT_POSITION_RAD = Units.degreesToRadians(-90.0);

    // Position Setpoints (radians)
    public static final double STOW_POSITION = Units.degreesToRadians(-90.0);
  }

  public static class ShooterConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 49;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
    public static final double GEAR_RATIO = 1.0;
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

    // PID and Feedforward (VelocityVoltage)
    public static final double KP = 0.5;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.25;
    public static final double KV = 0.12;
    public static final double KA = 0.01;

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 80.0;
    public static final double SUPPLY_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Velocity tolerance (rot/s)
    public static final double VELOCITY_TOLERANCE = 2.0;
  }

  public static class HoodConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 50;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
    public static final double GEAR_RATIO = 1.0 / 12.6; // mechanism rotations per motor rotation
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

    // Position limits (rad from horizontal)
    public static final double MIN_POSITION_RAD = Units.degreesToRadians(21.0);
    public static final double MAX_POSITION_RAD = Units.degreesToRadians(45.0);

    // PID and Feedforward Constants
    public static final double KP = 25.0;
    public static final double KI = 0.0;
    public static final double KD = 0.1;
    public static final double KS = 0.1;
    public static final double KV = 0.12;
    public static final double KA = 0.0;
    public static final double KG = 0.23;
    public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Elevator_Static;

    // Motion Magic Constants (motor rotations per second)
    public static final double CRUISE_VELOCITY = 80.0;
    public static final double ACCELERATION = 160.0;
    public static final double JERK = 1600.0;

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    // Position Tolerance
    public static final double AIMING_TOLERANCE_RAD = Units.degreesToRadians(1.0);

    // Position Setpoints (radians)
    public static final double STOW_POSITION = Units.degreesToRadians(21.0);
  }

  public static class ClimbConstants {
    // Hardware Configuration - 4 Winch Motors
    public static final int RIGHT_FRONT_MOTOR_CAN_ID = 51;
    public static final int RIGHT_BACK_MOTOR_CAN_ID = 52;
    public static final int LEFT_FRONT_MOTOR_CAN_ID = 53;
    public static final int LEFT_BACK_MOTOR_CAN_ID = 54;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;

    // ==================== Secondary Hook Servo Configuration ====================
    // Angle servos: 180deg, 500-2500us. Hardstop servos: 100deg, 1000-2000us.
    // Right angle: not inverted. Right hardstop: inverted.
    // Left angle: inverted. Left hardstop: not inverted.

    /** Angle servo config: 180deg range, 500-2500us pulse. */
    public static class AngleServo {
      public static final double FULL_RANGE_DEG = 180.0;
      public static final int PULSE_MIN_US = 500;
      public static final int PULSE_MAX_US = 2500;

      // Per-servo stowed/released positions (0.0-1.0, before inversion)
      public static final double LEFT_STOWED_POSITION = 5.0 / FULL_RANGE_DEG;
      public static final double LEFT_RELEASED_POSITION = 155.0 / FULL_RANGE_DEG;
      public static final double RIGHT_STOWED_POSITION = 6.0 / FULL_RANGE_DEG;
      public static final double RIGHT_RELEASED_POSITION = 156.0 / FULL_RANGE_DEG;

      /** Time (seconds) for the angle servo to travel its full range. */
      public static final double TRAVEL_TIME_SEC = 1.2;
    }

    /** Hardstop servo config: 100deg range, 1000-2000us pulse. */
    public static class HardstopServo {
      public static final double FULL_RANGE_DEG = 180.0;
      public static final int PULSE_MIN_US = 500;
      public static final int PULSE_MAX_US = 2500;

      // Per-servo stowed/released positions (0.0-1.0, before inversion)
      public static final double LEFT_STOWED_POSITION = 25.0 / FULL_RANGE_DEG;
      public static final double LEFT_RELEASED_POSITION = 82.0 / FULL_RANGE_DEG;
      public static final double RIGHT_STOWED_POSITION = 26.0 / FULL_RANGE_DEG;
      public static final double RIGHT_RELEASED_POSITION = 85.0 / FULL_RANGE_DEG;

      /** Time (seconds) for the hardstop servo to travel its full range. */
      public static final double TRAVEL_TIME_SEC = 0.6;
    }

    // PWM Channels
    public static final int RIGHT_ANGLE_SERVO_PWM = 6;
    public static final int RIGHT_HARDSTOP_SERVO_PWM = 7;
    public static final int LEFT_ANGLE_SERVO_PWM = 8;
    public static final int LEFT_HARDSTOP_SERVO_PWM = 9;

    // Per-servo inversion flags (true = CW-positive, maxdeg at set(0.0))
    public static final boolean RIGHT_ANGLE_SERVO_INVERTED = false;
    public static final boolean RIGHT_HARDSTOP_SERVO_INVERTED = true;
    public static final boolean LEFT_ANGLE_SERVO_INVERTED = true;
    public static final boolean LEFT_HARDSTOP_SERVO_INVERTED = false;

    // Gear ratios (mechanism per motor rotation)
    // Front: 1/48, Back: 1/27
    public static final double FRONT_GEAR_RATIO = 1.0 / 48.0;
    public static final double BACK_GEAR_RATIO = 1.0 / 27.0;

    // Motor Inversion - Configure each motor independently
    public static final InvertedValue RIGHT_FRONT_MOTOR_INVERTED =
        InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue RIGHT_BACK_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue LEFT_FRONT_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue LEFT_BACK_MOTOR_INVERTED =
        InvertedValue.CounterClockwise_Positive;

    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

    // PID and Feedforward Constants (Slot 0 -- MotionMagicVoltage position control)
    public static final double KP = 1.0;
    public static final double KI = 0.0;
    public static final double KD = 0.05;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;
    public static final double KG = 0.0;

    // Velocity PID and Feedforward Constants (Slot 1 -- VelocityVoltage path following)
    // Matched to ShooterConstants velocity control gains
    public static final double VELOCITY_KP = 0.06;
    public static final double VELOCITY_KI = 0.0;
    public static final double VELOCITY_KD = 0.0;
    public static final double VELOCITY_KS = 0.15;
    public static final double VELOCITY_KV = 0.115;
    public static final double VELOCITY_KA = 0.0;

    // Motion Magic Constants (mechanism/drum rotations per second)
    public static final double MOTOR_FREE_SPEED_RPS = 100.0;
    public static final double SPEED_UTILIZATION = 0.90;
    // Cruise velocity (drum rot/s) -- front (slower) ratio is the bottleneck
    public static final double CRUISE_VELOCITY =
        MOTOR_FREE_SPEED_RPS * FRONT_GEAR_RATIO * SPEED_UTILIZATION;
    public static final double BACK_CRUISE_VELOCITY =
        MOTOR_FREE_SPEED_RPS * BACK_GEAR_RATIO * SPEED_UTILIZATION;
    public static final double ACCELERATION = CRUISE_VELOCITY * 3.75;
    public static final double BACK_ACCELERATION = BACK_CRUISE_VELOCITY * 3.75;
    public static final double JERK = ACCELERATION * 4.0;
    public static final double BACK_JERK = BACK_ACCELERATION * 4.0;

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 80.0;
    public static final double SUPPLY_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Calibration Mode Current Limits (lower limits for safe manual motor jogging)
    public static final double CALIBRATION_FRONT_STATOR_CURRENT_LIMIT = 80.0;
    public static final double CALIBRATION_FRONT_SUPPLY_CURRENT_LIMIT = 60.0;
    public static final double CALIBRATION_BACK_STATOR_CURRENT_LIMIT = 80.0;
    public static final double CALIBRATION_BACK_SUPPLY_CURRENT_LIMIT = 60.0;

    // Position Setpoints (rotations) - Legacy direct motor control
    public static final double STOWED_POSITION = 0.0;

    // Position Tolerance
    public static final double POSITION_TOLERANCE = 1.0;

    // ==================== Inverse Kinematics Configuration ====================

    // Fixed Points (all relative to back winch at origin)
    public static final double FRONT_WINCH_X_METERS = 0.341;
    public static final double FRONT_WINCH_Y_METERS = 0.13208;
    public static final double SHOULDER_X_METERS = 0.3048;
    public static final double SHOULDER_Y_METERS = 0.0;

    // Link Lengths
    public static final double LINK_1_LENGTH_METERS = 0.32;
    public static final double LINK_2_LENGTH_METERS = 0.42547;

    // Cable Attachment Offsets (distance along link from moving end)
    public static final double BACK_CABLE_ATTACH_ON_LINK1_METERS = 0.02684;
    public static final double FRONT_CABLE_ATTACH_ON_LINK2_METERS = 0.15267;

    // Cable Drum
    public static final double CABLE_DRUM_CIRCUMFERENCE_METERS = 0.0439941;

    // Cable Layer Buildup -- effective circumference grows as cable stacks on the drum
    public static final double ROTATIONS_PER_LAYER = 4.0;
    public static final double CIRCUMFERENCE_PER_LAYER_METERS = 0.012;

    // Absolute rotations wound on the drum at the STOW pose
    public static final double STOW_ABSOLUTE_ROTATIONS = 10.0;

    // Mechanism Position Limits (drum rotations)
    public static final double MIN_MECHANISM_POSITION = 0.0;
    public static final double MAX_MECHANISM_POSITION = 100.0;

    // Joint Angle Limits (safety limits in radians)
    public static final double MIN_SHOULDER_ANGLE_RAD = Math.toRadians(60);
    public static final double MAX_SHOULDER_ANGLE_RAD = Math.toRadians(175);
    public static final double MIN_ELBOW_ANGLE_RAD = Math.toRadians(-160);
    public static final double MAX_ELBOW_ANGLE_RAD = Math.toRadians(-10);
    // Set false for testing paths outside joint limits
    public static final boolean ENABLE_JOINT_LIMITS = false;

    // Workspace Limits (for end effector reachability checks)
    public static final double WORKSPACE_MIN_X_METERS = -5.0;
    public static final double WORKSPACE_MAX_X_METERS = 5.0;
    public static final double WORKSPACE_MIN_Y_METERS = -5.0;
    public static final double WORKSPACE_MAX_Y_METERS = 5.0;

    // Starting Position (end effector at power-on, used to compute initial cable lengths via IK)
    public static final double START_POSITION_X_METERS = 0.33;
    public static final double START_POSITION_Y_METERS = 0.44;

    // IK Solver Tolerance
    public static final double IK_POSITION_TOLERANCE_METERS = 0.005;
    public static final double IK_ANGLE_TOLERANCE_RAD = 0.01;

    // ==================== Path Planning Constraints ====================

    // Cartesian path generation constraints (derived from cruise velocity and drum geometry)
    private static final double MID_LAYER_CIRCUMFERENCE_M =
        CABLE_DRUM_CIRCUMFERENCE_METERS + CIRCUMFERENCE_PER_LAYER_METERS;
    public static final double PATH_MAX_VELOCITY_MPS = CRUISE_VELOCITY * MID_LAYER_CIRCUMFERENCE_M;
    public static final double PATH_MAX_ACCELERATION_MPS2 =
        ACCELERATION * MID_LAYER_CIRCUMFERENCE_M;

    // ==================== Path Following Feedforward ====================
    // Additional Cartesian-space FF applied via Jacobian transpose on top of Slot 1 (kS+kV+kP).
    // EXTEND uses arm-only gravity comp; RETRACT uses full robot weight + spring comp.

    // Feedforward voltages (V) -- mapped through J^T to per-motor voltages
    public static final double EXTEND_GRAVITY_FF_VOLTS = 0.8; // arm-only gravity (extend)
    public static final double RETRACT_GRAVITY_FF_VOLTS = 3.0; // full robot weight (retract)
    public static final double SPRING_FF_VOLTS = 1.0; // extension spring comp (retract)

    // Position correction kP ((m/s) per (m) error). 0 to disable.
    public static final double PATH_POSITION_CORRECTION_KP = 2.0;

    // ==================== IMU Climb Assist (Auto-Level) ====================
    // Uses IMU roll to adjust L/R velocity during RETRACT, keeping robot level.

    /** IMU-based auto-level config for climb retract. */
    public static class ImuAssist {
      /** Master enable -- set true after tuning on real robot. */
      public static final boolean ENABLED = false;

      // Velocity PID (input: roll degrees, output: velocity correction m/s)
      public static final double KP = 0.003;
      public static final double KI = 0.0;
      public static final double KD = 0.001;
      public static final double MAX_VEL_CORRECTION_MPS = 0.015;
      public static final double DEADBAND_DEGREES = 1.0;
    }
  }

  public static class OrchestraConstants {
    /** CHRP file in src/main/deploy/. Convert MIDI via Phoenix Tuner X. */
    public static final String CHRP_FILE = "10trackCruelAngel.chrp";

    /** Allow Orchestra playback while robot is disabled. */
    public static final boolean ALLOW_MUSIC_DURING_DISABLE = true;

    /** Tracks in the CHRP file. Motors are distributed across tracks via modulo. */
    public static final int NUM_TRACKS = 10;
  }

  // ==================== Maple-Sim Tuning Constants ====================
  public static class SimConstants {

    // --- Intake Simulation ---
    /** Intake zone width (m). */
    public static final double INTAKE_WIDTH_METERS = 0.67;

    /** Intake extension beyond chassis when deployed (m). */
    public static final double INTAKE_EXTENSION_METERS =
        MechanismVisualization.INTAKE_EXTENSION_X_M;

    /** Max fuel the intake can hold. */
    public static final int INTAKE_CAPACITY = 20;

    /** Min motor output to consider intake running. */
    public static final double INTAKE_MOTOR_THRESHOLD = 0.05;

    // --- Shooter / Projectile ---

    /** Fuel launch height above floor (m). */
    public static final double PROJECTILE_INITIAL_HEIGHT_METERS =
        MechanismVisualization.TURRET_HEIGHT_M + MechanismVisualization.HOOD_Z_ABOVE_TURRET_M;

    /** Launch speed = flywheel RPS / 100 * scale. */
    public static final double LAUNCH_SPEED_SCALE = 10.0;

    /** Compression efficiency (0-1). Tune to match real shot distances. */
    public static final double FUEL_SPEED_EFFICIENCY = 1.15;

    /** Min flywheel velocity (rot/s) to launch. */
    public static final double MIN_LAUNCH_VELOCITY_RPS = 10.0;

    /** Min launch angle (deg). */
    public static final double MIN_LAUNCH_ANGLE_DEG = 15.0;

    /** Max launch angle (deg). */
    public static final double MAX_LAUNCH_ANGLE_DEG = 75.0;

    // --- Conveyor / Fuel Transfer ---
    /** Conveyor velocity threshold (RPS) to consider feeding. */
    public static final double CONVEYOR_FEED_THRESHOLD = 0.5;

    /** Cooldown ticks (x20ms) between fuel transfers. */
    public static final int FEED_COOLDOWN_TICKS = 7;
  }

  // ==================== 3D Mechanism Visualization (AdvantageScope) ====================
  // Robot-relative positions (m) for turret, hood, and intake pivot rendering.
  public static class MechanismVisualization {
    /** Number of articulated components. */
    public static final int NUM_COMPONENTS = 3;

    // --- Component 0: Turret ---
    public static final double TURRET_HEIGHT_M = 0.3887; // pivot height above floor (m)
    public static final double TURRET_X_M = 0.1909; // forward offset from robot center (m)
    public static final double TURRET_Y_M = 0; // lateral offset from robot center (m)

    // --- Component 1: Hood ---
    public static final double HOOD_Z_ABOVE_TURRET_M = 0.065; // above turret base (m)
    /** Forward offset of hood pivot from turret axis (m). */
    public static final double HOOD_X_FROM_TURRET_M = 0.105;
    /** Pitch offset (rad) for visualization alignment. */
    public static final double HOOD_PITCH_OFFSET_RAD = Units.degreesToRadians(-48.0);

    // --- Component 2: Intake (rack-and-pinion) ---
    /** Total linear travel when fully extended (m). */
    public static final double INTAKE_FULL_TRAVEL_M = 0.28;
    /** Tilt angle of intake slide rail from horizontal (rad). */
    public static final double INTAKE_TILT_RAD = Units.degreesToRadians(6.278);
    /** Motor rotations at full extension. */
    public static final double INTAKE_FULL_TRAVEL_ROTATIONS =
        IntakePivotConstants.SOFT_LIMIT_FORWARD;

    /** X component of full extension (m). */
    public static final double INTAKE_EXTENSION_X_M =
        INTAKE_FULL_TRAVEL_M * Math.cos(INTAKE_TILT_RAD);
    /** Z component of full extension (m). */
    public static final double INTAKE_EXTENSION_Z_M =
        INTAKE_FULL_TRAVEL_M * Math.sin(INTAKE_TILT_RAD);
  }
}
