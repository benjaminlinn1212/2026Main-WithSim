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

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // Shared CAN bus used by all superstructure subsystems
  public static final CANBus SUPERSTRUCTURE_CAN_BUS = new CANBus("Superstructure");

  // Odometry proximity check thresholds (used in RobotContainer.odometryCloseToPose)
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
  }

  public static class AutoConstants {
    // Path following PID controllers
    public static final double PATH_FOLLOWING_TRANSLATION_KP = 3.0;
    public static final double PATH_FOLLOWING_ROTATION_KP = 5.0;

    // PathPlanner pathfinding constraints (single source of truth)
    public static final double PATHFINDING_MAX_VELOCITY_MPS = 3.0;
    public static final double PATHFINDING_MAX_ACCELERATION_MPS2 = 5.0;
    public static final double PATHFINDING_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.toRadians(360);
    public static final double PATHFINDING_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC2 =
        Math.toRadians(540);

    // Climb straight-line approach (final segment after pathfinding)
    /**
     * Distance (m) from the climb target at which the robot stops pathfinding and drives straight
     */
    public static final double CLIMB_APPROACH_DISTANCE_M = 0.3;
    /** Max velocity (m/s) cap for the PID-controlled straight-line drive into the tower */
    public static final double CLIMB_APPROACH_MAX_VELOCITY_MPS = 3.0;
    /** Position tolerance (m) to consider the robot arrived at the climb target */
    public static final double CLIMB_APPROACH_TOLERANCE_M = 0.01;
    /** Heading tolerance (rad) to consider the robot arrived at the climb target */
    public static final double CLIMB_APPROACH_THETA_TOLERANCE_RAD = Math.toRadians(3.0);

    // DriveToPose PID gains (2910-style: PID on linear distance + heading PID)
    /** Proportional gain for linear distance-to-target PID controller */
    public static final double DRIVE_TO_POSE_KP = 3.5;
    /** Derivative gain for linear distance-to-target PID controller */
    public static final double DRIVE_TO_POSE_KD = 0.1;
    /** Proportional gain for heading PID controller */
    public static final double DRIVE_TO_POSE_THETA_KP = 5.0;
    /**
     * Static friction feedforward constant. Multiplied by max velocity to produce a minimum
     * velocity that overcomes drivetrain friction when distance > 0.5 inches.
     */
    public static final double DRIVE_TO_POSE_FRICTION_FF = 0.02;

    /**
     * Derating factor for time estimation. AD* paths are longer than straight-line due to curves
     * and obstacle avoidance, so we inflate the distance by 1/derating. Since the trapezoidal
     * motion profile already accounts for acceleration and deceleration phases physically, this
     * factor only needs to cover path curvature (~10-15% longer than straight-line). 0.9 means we
     * assume the actual path is ~11% longer than the straight-line distance.
     *
     * <p>Note: The old flat-speed model used AVG_SPEED_DERATING=0.7 which covered BOTH curvature
     * AND accel/decel. With the trapezoidal model handling accel/decel, 0.9 produces comparable
     * time estimates.
     */
    public static final double PATH_DISTANCE_DERATING = 0.9;

    public static final edu.wpi.first.math.geometry.Pose2d DEFAULT_RESET_POSE =
        new edu.wpi.first.math.geometry.Pose2d(
            0.0, 0.0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
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

    public static final double JOYSTICK_DEADBAND = 0.05;

    public static class DriveToPose {
      // This constraint is used in the driveToPose() function by PPLib AND PIDControl.
      public static final double TRANSLATION_KP = 2.0;
      public static final double TRANSLATION_KI = 0.0;
      public static final double TRANSLATION_KD = 0.2;
      public static final double TRANSLATION_TOLERANCE = 0.05;
      public static final double STATIC_FRICTION_CONSTANT = 1.2;
      public static final double MAX_VELOCITY = 3.0;

      public static final double ROTATION_KP = 7;
      public static final double ROTATION_KI = 0.0;
      public static final double ROTATION_KD = 0;
      public static final double ROTATION_TOLERANCE = Units.degreesToRadians(1);
    }

    /**
     * Trench Teleop Assist tuning. When the robot is near a trench, two effects activate:
     *
     * <ol>
     *   <li><b>Chassis orientation alignment</b> Ã¢â‚¬â€ injects omega (rotational velocity) to
     *       LERP the robot's heading toward the nearest cardinal direction, so the bumpers fit
     *       through the 22.25in tunnel. This modifies {@code omega}, not the velocity direction.
     *   <li><b>Lateral centering</b> Ã¢â‚¬â€ deflects the velocity vector toward the trench's
     *       center Y line, guiding the travel path toward the middle of the 48in corridor. Speed
     *       magnitude is preserved.
     * </ol>
     *
     * Both effects ramp from 0 at the buffer edge to full strength inside the trench.
     */
    public static class TrenchAssist {

      public static final double MAX_BLEND_FACTOR = 0.9;

      /**
       * Teleop approach buffer (meters). How far outside the trench walls the assist begins
       * ramping. Independent from {@link
       * frc.robot.auto.dashboard.FieldConstants#TRENCH_APPROACH_BUFFER} which is used by auto
       * heading snap and Superstructure hood stow. Larger = assist starts earlier, more time to
       * center.
       */
      public static final double APPROACH_BUFFER = 2.5;

      /**
       * Minimum speed (m/s) below which the assist is inactive. Prevents the assist from
       * interfering with fine positioning / stationary rotation near the trench.
       */
      public static final double MIN_SPEED_MPS = 0.5;

      /**
       * Maximum angular error (degrees) between the velocity vector and the nearest cardinal before
       * the assist activates. If the driver is traveling perpendicular to the trench, they clearly
       * don't intend to go through it Ã¢â‚¬â€ don't fight them.
       */
      public static final double MAX_HEADING_ERROR_DEG = 50.0;

      /**
       * Lateral centering strength (deg per meter of offset). The trench half-width is ~0.61m, so
       * 35Ã‚Â°/m gives ~21Ã‚Â° at the wall Ã¢â‚¬â€ a noticeable but gentle nudge toward center.
       * Lower values feel more like a suggestion, higher values feel like rails.
       */
      public static final double CENTERING_DEG_PER_METER = 45.0;

      /**
       * Maximum centering deflection angle (degrees). At 25Ã‚Â° the lateral component is ~42% of
       * speed Ã¢â‚¬â€ enough to guide you toward center without stealing all your forward
       * momentum.
       */
      public static final double MAX_CENTERING_DEG = 30.0;

      /**
       * Maximum omega correction (rad/s) for orientation alignment. Caps the injected rotational
       * rate so the trench assist doesn't spin the robot violently. The heading P-gain itself is
       * reused from {@link DriveToPose#ROTATION_KP} Ã¢â‚¬â€ no separate KP needed.
       */
      public static final double MAX_ORIENTATION_OMEGA_RAD_PER_SEC = 4.0;

      // === Wall Avoidance ===

      /**
       * Half the robot's bumper width in meters. Used to compute the position of the robot's bumper
       * edges relative to trench walls. 32in bumper Ã¢â€ â€™ 16in Ã¢â€ â€™ 0.406m.
       */
      public static final double ROBOT_HALF_WIDTH_M =
          Units.inchesToMeters(BUMPER_WIDTH_INCHES / 2.0);

      /**
       * Wall repulsion gain (m/s of lateral push per meter of encroachment into the danger zone).
       * Higher = harder virtual wall. At 4.0, a 0.1m encroachment produces 0.4 m/s of push-back
       * Ã¢â‚¬â€ firm enough to prevent wall contact without feeling like a spring.
       */
      public static final double WALL_REPULSION_MPS_PER_METER = 6.0;

      /**
       * Distance (meters) from a trench wall at which the repulsion force begins, measured from the
       * robot's <b>bumper edge</b> (the distance calculation already accounts for {@link
       * #ROBOT_HALF_WIDTH_M}). The trench has only ~0.20m of clearance per side, so this should be
       * small Ã¢â‚¬â€ just enough to catch the robot before contact. At 0.12m the repulsion
       * activates when the bumper is ~4.7in from the wall.
       */
      public static final double WALL_DANGER_ZONE_M = 0.12;
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

    // PID and Feedforward Constants (not used for duty cycle control)
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

    // Intake Percent Output (0.0 to 1.0)
    public static final double UPPER_INTAKE_PERCENT = 0.45;
    public static final double LOWER_INTAKE_PERCENT = 0.75;
    public static final double UPPER_OUTTAKE_PERCENT = -0.5;
    public static final double LOWER_OUTTAKE_PERCENT = -0.5;
  }

  public static class IntakePivotConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 43;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
    public static final double GEAR_RATIO = 1.0;
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final double MOTOR_ROTOR_OFFSET = 0.048;

    // Soft Limits (rotations)
    public static final double SOFT_LIMIT_REVERSE = 0.0;
    public static final double SOFT_LIMIT_FORWARD = 28.0;

    // PID and Feedforward Constants
    public static final double KP = 1.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;

    // Motion Magic Constants
    public static final double CRUISE_VELOCITY = 100.0;
    public static final double ACCELERATION = 300.0;
    public static final double JERK = 2000.0;

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Position Setpoints (rotations)
    public static final double STOWED_POSITION = 0.0;
    public static final double HALF_DEPLOYED_POSITION = 22.0;
    public static final double DEPLOYED_POSITION = 27.4;

    // Position Tolerance
    public static final double POSITION_TOLERANCE = 0.5;
  }

  public static class ConveyorConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 45;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
    public static final double GEAR_RATIO = 1.0;
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

    // PID and Feedforward Constants (not used for duty cycle control)
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

    // Conveyor Duty Cycle Percentages
    public static final double TO_SHOOTER_PERCENT = -0.5;
    public static final double TO_BUCKET_PERCENT = 0.5;
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

    // PID and Feedforward Constants (not used for duty cycle control)
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
    public static final double TO_SHOOTER_PERCENT = 0.7;
  }

  public static class Aiming {
    // Shot Calculation Parameters
    public static final double MIN_SHOT_DISTANCE = 1.0;
    public static final double MAX_SHOT_DISTANCE = 6.0;
    public static final double PHASE_DELAY = 0.03;

    public static final int FEEDFORWARD_FILTER_TAPS = 5;

    // Neutral Zone Shot Settings
    public static final double NEUTRAL_ZONE_HOOD_ANGLE_DEG = 35.0;
  }

  public static class TurretConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 44;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
    public static final double GEAR_RATIO =
        1.0 / 26.812; // mechanism rotations per motor rotation (reduction)
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

    // Turret Position Offset from Robot Center (meters)
    public static final Translation2d TURRET_OFFSET_FROM_ROBOT_CENTER =
        new Translation2d(0.1909, 0.0);

    // Position Limits (radians - mechanism angle limits)
    public static final double MIN_POSITION_RAD = Units.rotationsToRadians(-1.0);
    public static final double MAX_POSITION_RAD = Units.rotationsToRadians(0.2777);

    // PID and Feedforward Constants (MotionMagicVoltage mode)
    public static final double KP = 10.0;
    public static final double KI = 0.0;
    public static final double KD = 0.3;
    public static final double KS = 0.25;
    public static final double KV = 0.12;
    public static final double KA = 0.0;
    public static final double KG = 0.0;

    // Motion Magic Constants (motor rotations per second)
    public static final double CRUISE_VELOCITY = 2.0 / GEAR_RATIO;
    public static final double ACCELERATION = 10.0 / GEAR_RATIO;
    public static final double JERK = 100.0 / GEAR_RATIO;

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 150.0;
    public static final double SUPPLY_CURRENT_LIMIT = 80.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Position Tolerance (radians)
    public static final double AIMING_TOLERANCE_RAD = Units.degreesToRadians(2.0);

    // Boot Position (radians) Ã¢â‚¬â€ the mechanism angle when the robot powers on.
    // The turret physically starts at -90Ã‚Â° (facing right when viewed from above).
    // TurretIOTalonFX seeds the encoder to this value via motor.setPosition() at boot
    // because FeedbackRotorOffset is limited to [0,1) motor rotations and cannot
    // represent the multi-rotation offset required by the 26.8:1 gear ratio.
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

    // PID and Feedforward Constants (VelocityVoltage control)
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

    // Shooter Speed Presets (rotations per second)
    public static final double NEUTRAL_ZONE_SPEED = 80.0;

    // Velocity Tolerance
    public static final double VELOCITY_TOLERANCE = 2.0;
  }

  public static class HoodConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 50;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
    public static final double GEAR_RATIO =
        1.0 / 12.6; // mechanism rotations per motor rotation (reduction)
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final double ROTOR_OFFSET = 0.228027;

    // Mechanism Zero Angle (degrees from horizontal)
    public static final double MECHANISM_ZERO_ANGLE_DEG = 20.0;

    // Position Limits (radians from horizontal)
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
    public static final double CRUISE_VELOCITY = 80.0 / GEAR_RATIO;
    public static final double ACCELERATION = 160.0 / GEAR_RATIO;
    public static final double JERK = 1600.0 / GEAR_RATIO;

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
    //
    // Two servo types:
    //   Angle servos   Ã¢â‚¬â€ 180Ã‚Â° travel, 500Ã‚ÂµsÃ¢â‚¬â€œ2500Ã‚Âµs pulse range
    //   Hardstop servos Ã¢â‚¬â€ 100Ã‚Â° travel, 1000Ã‚ÂµsÃ¢â‚¬â€œ2000Ã‚Âµs pulse range
    //
    // Inversion convention (all servos are CCW-positive when non-inverted):
    //   Non-inverted: set(0.0) = 0Ã‚Â°,   requesting +NÃ‚Â° moves 0Ã‚Â° Ã¢â€ â€™ NÃ‚Â°
    //   Inverted:     set(0.0) = maxÃ‚Â°, requesting +NÃ‚Â° moves maxÃ‚Â° Ã¢â€ â€™
    // (maxÃ¢Ë†â€™N)Ã‚Â°
    //   In code: pwmValue = inverted ? (1.0 Ã¢Ë†â€™ input) : input
    //
    // Per-side inversion:
    //   Right angle   Ã¢â€ â€™ NOT inverted   (CCW-positive, 0Ã‚Â°Ã¢â€ â€™180Ã‚Â°)
    //   Right hardstop Ã¢â€ â€™ INVERTED      (CW-positive,  100Ã‚Â°Ã¢â€ â€™0Ã‚Â°)
    //   Left angle    Ã¢â€ â€™ INVERTED       (CW-positive,  180Ã‚Â°Ã¢â€ â€™0Ã‚Â°)
    //   Left hardstop Ã¢â€ â€™ NOT inverted   (CCW-positive, 0Ã‚Â°Ã¢â€ â€™100Ã‚Â°)

    /** Angle servo config: 180Ã‚Â° range, 500Ã‚ÂµsÃ¢â‚¬â€œ2500Ã‚Âµs pulse. */
    public static class AngleServo {
      public static final double FULL_RANGE_DEG = 180.0;
      public static final int PULSE_MIN_US = 500;
      public static final int PULSE_MAX_US = 2500;

      // Per-servo stowed/released positions (0.0-1.0, before inversion)
      public static final double LEFT_STOWED_POSITION = 0.0 / 180.0;
      public static final double LEFT_RELEASED_POSITION = 140.0 / 180.0;
      public static final double RIGHT_STOWED_POSITION = 0.0 / 180.0;
      public static final double RIGHT_RELEASED_POSITION = 140.0 / 180.0;

      /** Time (seconds) for the angle servo to travel its full range. */
      public static final double TRAVEL_TIME_SEC = 1.0;
    }

    /** Hardstop servo config: 100Ã‚Â° range, 1000Ã‚ÂµsÃ¢â‚¬â€œ2000Ã‚Âµs pulse. */
    public static class HardstopServo {
      public static final double FULL_RANGE_DEG = 100.0;
      public static final int PULSE_MIN_US = 1000;
      public static final int PULSE_MAX_US = 2000;

      // Per-servo stowed/released positions (0.0-1.0, before inversion)
      public static final double LEFT_STOWED_POSITION = 0.0 / 100.0;
      public static final double LEFT_RELEASED_POSITION = 57.0 / 100.0;
      public static final double RIGHT_STOWED_POSITION = 0.0 / 100.0;
      public static final double RIGHT_RELEASED_POSITION = 57.0 / 100.0;

      /** Time (seconds) for the hardstop servo to travel its full range. */
      public static final double TRAVEL_TIME_SEC = 0.7;
    }

    // PWM Channels
    public static final int RIGHT_ANGLE_SERVO_PWM = 6;
    public static final int RIGHT_HARDSTOP_SERVO_PWM = 7;
    public static final int LEFT_ANGLE_SERVO_PWM = 8;
    public static final int LEFT_HARDSTOP_SERVO_PWM = 9;

    // Per-servo inversion flags (true = CW-positive, maxÃ‚Â° at set(0.0))
    public static final boolean RIGHT_ANGLE_SERVO_INVERTED = false;
    public static final boolean RIGHT_HARDSTOP_SERVO_INVERTED = true;
    public static final boolean LEFT_ANGLE_SERVO_INVERTED = true;
    public static final boolean LEFT_HARDSTOP_SERVO_INVERTED = false;

    // Gear Ratios: Mechanism rotations per motor rotation (speed reduction)
    // Front motors: 100:1 reduction Ã¢â€ â€™ 1 motor rotation = 1/100 drum rotation
    // Back motors: 80:1 reduction Ã¢â€ â€™ 1 motor rotation = 1/80 drum rotation
    // Used in both Phoenix SensorToMechanismRatio and IK calculations
    public static final double FRONT_GEAR_RATIO = 1.0 / 60.0;
    public static final double BACK_GEAR_RATIO = 1.0 / 48.0;

    // Motor Inversion - Configure each motor independently
    public static final InvertedValue RIGHT_FRONT_MOTOR_INVERTED =
        InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue RIGHT_BACK_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue LEFT_FRONT_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue LEFT_BACK_MOTOR_INVERTED =
        InvertedValue.CounterClockwise_Positive;

    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

    // PID and Feedforward Constants (Slot 0 Ã¢â‚¬â€ MotionMagicVoltage position control)
    public static final double KP = 2.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;
    public static final double KG = 0.0;

    // Velocity PID and Feedforward Constants (Slot 1 Ã¢â‚¬â€ VelocityVoltage path following)
    // Matched to ShooterConstants velocity control gains
    public static final double VELOCITY_KP = 0.06;
    public static final double VELOCITY_KI = 0.0;
    public static final double VELOCITY_KD = 0.0;
    public static final double VELOCITY_KS = 0.0;
    public static final double VELOCITY_KV = 0.115;
    public static final double VELOCITY_KA = 0.0;

    // Motion Magic Constants (mechanism/drum rotations per second)
    public static final double MOTOR_FREE_SPEED_RPS = 100.0;
    public static final double SPEED_UTILIZATION = 0.90;
    public static final double CRUISE_VELOCITY =
        MOTOR_FREE_SPEED_RPS * FRONT_GEAR_RATIO * SPEED_UTILIZATION;
    public static final double ACCELERATION = CRUISE_VELOCITY * 3.75;
    public static final double JERK = ACCELERATION * 4.0;

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 80.0;
    public static final double SUPPLY_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Calibration Mode Current Limits (lower limits for safe manual motor jogging)
    public static final double CALIBRATION_FRONT_STATOR_CURRENT_LIMIT = 40.0;
    public static final double CALIBRATION_FRONT_SUPPLY_CURRENT_LIMIT = 30.0;
    public static final double CALIBRATION_BACK_STATOR_CURRENT_LIMIT = 40.0;
    public static final double CALIBRATION_BACK_SUPPLY_CURRENT_LIMIT = 30.0;

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

    // Cable Layer Buildup — effective circumference grows as cable stacks on the drum
    public static final double ROTATIONS_PER_LAYER = 4.0;
    public static final double CIRCUMFERENCE_PER_LAYER_METERS = 0.012;

    // Absolute rotations wound on the drum at the STOW pose
    public static final double STOW_ABSOLUTE_ROTATIONS = 10.0;

    // Mechanism Position Limits (drum rotations)
    public static final double MIN_MECHANISM_POSITION = 0.0;
    public static final double MAX_MECHANISM_POSITION = 100.0;

    // Joint Angle Limits (safety limits in radians)
    public static final double MIN_SHOULDER_ANGLE_RAD = Math.toRadians(85);
    public static final double MAX_SHOULDER_ANGLE_RAD = Math.toRadians(160);
    public static final double MIN_ELBOW_ANGLE_RAD = Math.toRadians(-140);
    public static final double MAX_ELBOW_ANGLE_RAD = Math.toRadians(-30);
    // Set false for testing paths outside joint limits
    public static final boolean ENABLE_JOINT_LIMITS = true;

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

    public static final double VELOCITY_KG_PULLING = 0.15;

    // ==================== IMU Climb Assist (Auto-Level) ====================
    // Uses IMU roll to differentially adjust left/right end-effector velocities
    // during RETRACT paths, keeping the robot level while pulling up.
    // See docs/AUTO_LEVEL_CLIMB_PLAN.md for full design.

    /** IMU-based auto-level config for climb retract paths. */
    public static class ImuAssist {
      /** Master enable â€” set true after tuning on real robot. */
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
    /**
     * Name of the .chrp file to play. Place the file in {@code src/main/deploy/} â€” it will be
     * automatically deployed to the roboRIO. Convert MIDI â†’ CHRP using Phoenix Tuner X's CHRP
     * Converter (Tools â†’ CHRP Converter).
     */
    public static final String CHRP_FILE = "4trackCruelAngel.chrp";

    /**
     * When true, applies the {@code AllowMusicDurDisable} audio config so Orchestra can play even
     * when the robot is disabled. Useful for pre-match entertainment.
     */
    public static final boolean ALLOW_MUSIC_DURING_DISABLE = true;

    /**
     * Number of tracks in the .chrp file. Motors are distributed across tracks using modulo â€” if
     * you have 8 motors and 2 tracks, 4 motors play track 0 and 4 motors play track 1. Set to 1 if
     * your MIDI was exported as a single-track file (all motors play the same part).
     */
    public static final int NUM_TRACKS = 4;
  }

  // ==================== Maple-Sim Tuning Constants ====================
  // All tunable parameters for the maple-sim integration (intake, shooter projectiles, conveyor
  // fuel transfer). Tweak these to dial in simulation behavior without hunting through IO files.
  public static class SimConstants {

    // --- Intake Simulation ---
    /** Width of the intake zone on the chassis (meters). */
    public static final double INTAKE_WIDTH_METERS = 0.67;

    /**
     * How far the intake extends beyond the chassis frame when deployed (meters). Derived from the
     * X component of the rack-and-pinion linear extension.
     */
    public static final double INTAKE_EXTENSION_METERS =
        MechanismVisualization.INTAKE_EXTENSION_X_M;

    /** Maximum number of fuel the intake can hold at once. */
    public static final int INTAKE_CAPACITY = 20;

    /** Minimum motor percent-output to consider the intake "running". */
    public static final double INTAKE_MOTOR_THRESHOLD = 0.05;

    // --- Shooter / Projectile ---
    // Motor physics (acceleration, kV, current) come from ShooterConstants + WPILib DCMotorSim.
    // Only projectile-specific tuning values live here.
    // XY offset and height are derived from the real mechanism geometry
    // (TurretConstants + MechanismVisualization) â€” no need to duplicate values here.

    /**
     * Height above floor at which the fuel leaves the shooter (meters). Derived from turret height
     * + hood offset above turret.
     */
    public static final double PROJECTILE_INITIAL_HEIGHT_METERS =
        MechanismVisualization.TURRET_HEIGHT_M + MechanismVisualization.HOOD_Z_ABOVE_TURRET_M;

    /**
     * Launch-speed scaling factor. Actual launch speed (m/s) = flywheel RPS / 100 *
     * LAUNCH_SPEED_SCALE. Example: 50 RPS â†’ 5 m/s at scale 10, 80 RPS â†’ 8 m/s.
     */
    public static final double LAUNCH_SPEED_SCALE = 10.0;

    /**
     * Compression / efficiency multiplier (0â€“1). On a real robot the ball compresses against the
     * flywheel and hood, so exit speed is lower than ideal surface speed. 1.0 = no loss, 0.7 = 30%
     * speed loss. Tune this until sim trajectories match real-robot shot distances.
     */
    public static final double FUEL_SPEED_EFFICIENCY = 1.15;

    /** Minimum flywheel velocity (rot/s) before the sim will launch a projectile. */
    public static final double MIN_LAUNCH_VELOCITY_RPS = 10.0;

    /** Minimum allowed launch angle (degrees) â€” clamp for hood safety. */
    public static final double MIN_LAUNCH_ANGLE_DEG = 15.0;

    /** Maximum allowed launch angle (degrees) â€” clamp for hood safety. */
    public static final double MAX_LAUNCH_ANGLE_DEG = 75.0;

    // --- Conveyor / Fuel Transfer ---
    /**
     * Conveyor percent-output threshold to trigger feeding toward the shooter (negative = feed).
     */
    public static final double CONVEYOR_FEED_THRESHOLD = -0.05;

    /** Cooldown ticks (Ã— 20 ms) between consecutive fuel transfers from intake â†’ shooter. */
    public static final int FEED_COOLDOWN_TICKS = 7;
  }

  // ==================== 3D Mechanism Visualization (AdvantageScope) ====================
  // Component layout for articulated 3D rendering in AdvantageScope's 3D Field tab.
  // Components (indexed 0â€“2 in the robot model config):
  //   0 = Turret   â€” yaw rotation, mounted on frame
  //   1 = Hood     â€” pitch rotation, mounted on turret
  //   2 = IntakePivot â€” pitch rotation, mounted on frame rear
  //
  // All positions are robot-relative (origin = robot center at floor level).
  // Units: meters for positions, used with Pose3d / Rotation3d.
  public static class MechanismVisualization {
    /** Number of articulated components logged to AdvantageScope. */
    public static final int NUM_COMPONENTS = 3;

    // --- Component 0: Turret ---
    /** Height of the turret rotation axis above the floor (meters). */
    public static final double TURRET_HEIGHT_M = 0.3887;
    /** X offset of turret pivot from robot center (forward-positive, meters). */
    public static final double TURRET_X_M = 0.1909;
    /** Y offset of turret pivot from robot center (left-positive, meters). */
    public static final double TURRET_Y_M = 0;

    // --- Component 1: Hood ---
    /** Height of the hood pivot above the turret base (meters). Sits on top of turret. */
    public static final double HOOD_Z_ABOVE_TURRET_M = 0.065;
    /**
     * Forward offset of the hood pivot from the turret axis (meters). The hood pitches at this
     * point.
     */
    public static final double HOOD_X_FROM_TURRET_M = 0.105;
    /**
     * Pitch offset (radians) added to the hood angle for visualization. Use this to align the 3D
     * model with the actual mechanism â€” positive tilts the model further up.
     */
    public static final double HOOD_PITCH_OFFSET_RAD = Units.degreesToRadians(-48.0);

    // --- Component 2: Intake (rack-and-pinion linear extension) ---
    /** Total linear travel of the intake rack when fully extended (meters). */
    public static final double INTAKE_FULL_TRAVEL_M = 0.28;
    /**
     * Tilt angle of the intake slide rail from horizontal (radians). 6.278Â° â†’ mostly rearward,
     * slightly downward.
     */
    public static final double INTAKE_TILT_RAD = Units.degreesToRadians(6.278);
    /** Motor rotations at full extension (soft-limit forward). */
    public static final double INTAKE_FULL_TRAVEL_ROTATIONS =
        IntakePivotConstants.SOFT_LIMIT_FORWARD;

    /**
     * X component of full intake extension (meters). 6.278Â° from horizontal â†’ cos is the large X
     * part.
     */
    public static final double INTAKE_EXTENSION_X_M =
        INTAKE_FULL_TRAVEL_M * Math.cos(INTAKE_TILT_RAD);
    /**
     * Z component of full intake extension (meters). 6.278Â° from horizontal â†’ sin is the small Z
     * part.
     */
    public static final double INTAKE_EXTENSION_Z_M =
        INTAKE_FULL_TRAVEL_M * Math.sin(INTAKE_TILT_RAD);
  }
}
