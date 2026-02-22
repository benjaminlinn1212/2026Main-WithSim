// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
            new Translation3d(0.177, 0.0, 0.53), // 17.7cm forward from turret axis, 53cm up
            new Rotation3d(0, Units.degreesToRadians(15), 0)); // Pitched up 15 degrees

    // Turret position relative to robot center
    // Vector from robot center TO turret center (positive X = forward, meters).
    public static final Translation2d ROBOT_TO_TURRET =
        TurretConstants.TURRET_OFFSET_FROM_ROBOT_CENTER;
  }

  public static class AutoConstants {
    // Path following PID controllers (inspired by Team 254's tuning approach)
    // Note: Standard PathPlanner uses 2 controllers (translation + rotation)
    // We use 254's translation value as our baseline
    public static final double kPLTEController = 3.0; // Translation (along/cross track combined)
    public static final double kPThetaController = 5.0; // Rotation error

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

    // Default pose for gyro/pose reset
    public static final edu.wpi.first.math.geometry.Pose2d DEFAULT_RESET_POSE =
        new edu.wpi.first.math.geometry.Pose2d(
            0.33, 0.33, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
  }

  public static class DriveConstants {
    // Maple-Sim Physics Simulation Configuration
    public static final boolean USE_MAPLE_SIM = true;
    public static final double ROBOT_WEIGHT_KILOGRAMS = 30.0;
    public static final double BUMPER_LENGTH_INCHES = 32.0;
    public static final double BUMPER_WIDTH_INCHES = 32.0;
    public static final int DRIVE_MOTOR_COUNT = 1;
    public static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.2;

    // Teleop Speed Limits
    public static final double MAX_TELEOP_SPEED_MPS = 5.0; // meters per second
    public static final double MAX_TELEOP_ANGULAR_SPEED_RAD_PER_SEC = Math.PI * 2; // rad/s

    public static final double JOYSTICK_DEADBAND = 0.05;
    public static final LinearVelocity JOYSTICK_POV_VELOCITY = MetersPerSecond.of(0.2);

    public static final PPHolonomicDriveController PP_HOLONOMIC_DRIVE_CONTROLLER =
        new PPHolonomicDriveController(
            // PPHolonomicController is the built in path following controller for holonomic drive
            // trains.
            new PIDConstants(3.5, 0.0, 0.0), // Translation PID constants
            new PIDConstants(2.8, 0.0, 0.0) // Rotation PID constants
            );

    public static class DriveToPose {
      // This constraint is used in the driveToPose() function by PPLib AND PIDControl.
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

  public static class IntakeConstants {
    // Hardware Configuration
    public static final int UPPER_MOTOR_CAN_ID = 41;
    public static final int LOWER_MOTOR_CAN_ID = 42;
    public static final CANBus CAN_BUS = new CANBus("Superstructure");
    public static final double GEAR_RATIO = 1.0; // Direct drive
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

    // Combined (for backward compatibility)
    public static final double INTAKE_PERCENT = 0.45; // 45% speed for intaking
    public static final double OUTTAKE_PERCENT = -0.5; // 50% speed for outtaking
  }

  public static class IntakePivotConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 43;
    public static final CANBus CAN_BUS = new CANBus("Superstructure");
    public static final double GEAR_RATIO = 1.0; // Direct drive or specify actual ratio
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final double MOTOR_ROTOR_OFFSET = 0.048; // Motor encoder offset (rotations)

    // Soft Limits (rotations)
    public static final double SOFT_LIMIT_REVERSE = 0.0; // Minimum position
    public static final double SOFT_LIMIT_FORWARD = 28.0; // Maximum position

    // PID and Feedforward Constants
    public static final double KP = 1.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;
    public static final double KG = 0.0;

    // Motion Magic Constants
    public static final double CRUISE_VELOCITY = 100.0; // rotations per second
    public static final double ACCELERATION = 200.0; // rotations per second^2
    public static final double JERK = 2000.0; // rotations per second^3

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Position Setpoints (rotations)
    public static final double STOWED_POSITION = 0.0; // Stowed/up position
    public static final double HALF_DEPLOYED_POSITION = 14.0; // Halfway between stowed and deployed
    public static final double DEPLOYED_POSITION = 27.8; // Extended/down position for intaking

    // Position Tolerance
    public static final double POSITION_TOLERANCE = 0.5; // rotations
  }

  public static class ConveyorConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 45;
    public static final CANBus CAN_BUS = new CANBus("Superstructure");
    public static final double GEAR_RATIO = 1.0; // Direct drive
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
    public static final CANBus CAN_BUS = new CANBus("Superstructure");
    public static final double GEAR_RATIO = 1.0; // Direct drive
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

    // Indexer Duty Cycle
    public static final double TO_SHOOTER_DUTY_CYCLE = 0.7;
  }

  public static class Aiming {
    // Shot Calculation Parameters
    public static final double MIN_SHOT_DISTANCE = 1.0; // meters - minimum valid shot distance
    public static final double MAX_SHOT_DISTANCE = 6.0; // meters - maximum valid shot distance
    public static final double PHASE_DELAY =
        0.03; // seconds - prediction lookahead for motion compensation

    // Feedforward Filter
    // Lower tap count = less smoothing but less phase lag.
    // 5 taps at 20ms loop = ~50ms group delay (was 10 taps / ~100ms).
    public static final int FEEDFORWARD_FILTER_TAPS = 5; // Moving average filter size

    // Neutral Zone Shot Settings
    public static final double NEUTRAL_ZONE_HOOD_ANGLE_DEG =
        35.0; // Hood angle for neutral zone shots
  }

  public static class TurretConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 44;
    public static final CANBus CAN_BUS = new CANBus("Superstructure");
    public static final double GEAR_RATIO =
        1.0 / 26.812; // mechanism rotations per motor rotation (reduction)
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final double ROTOR_OFFSET = -0.03; // Motor rotations

    // Turret Position Offset from Robot Center
    // Positive X = forward, Positive Y = left from robot center (meters)
    public static final Translation2d TURRET_OFFSET_FROM_ROBOT_CENTER =
        new Translation2d(0.1909, 0.0);

    // Position Limits (radians - mechanism angle limits)
    public static final double MIN_POSITION_RAD = Units.rotationsToRadians(-1.0);
    public static final double MAX_POSITION_RAD = Units.rotationsToRadians(0.2777);

    // PID and Feedforward Constants (MotionMagicVoltage mode)
    public static final double KP = 10.0;
    public static final double KI = 0.0;
    public static final double KD = 0.3; // Dampens jitter from vision-induced setpoint changes
    public static final double KS = 0.25;
    public static final double KV = 0.12;
    public static final double KA = 0.0;
    public static final double KG = 0.0;

    // Motion Magic Constants (motor rotations per second)
    // Note: These are in motor units since SensorToMechanismRatio = 1.0
    public static final double CRUISE_VELOCITY = 2.0 / GEAR_RATIO; // motor rotations/sec
    public static final double ACCELERATION = 10.0 / GEAR_RATIO; // motor rotations/sec²
    public static final double JERK = 100.0 / GEAR_RATIO; // motor rotations/sec³

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 150.0;
    public static final double SUPPLY_CURRENT_LIMIT = 80.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Position Tolerance (radians)
    public static final double AIMING_TOLERANCE_RAD = Units.degreesToRadians(2.0);

    // Position Setpoints (radians)
    public static final double STOW_POSITION = 0.0; // Forward
  }

  public static class ShooterConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 49;
    public static final CANBus CAN_BUS = new CANBus("Superstructure");
    public static final double GEAR_RATIO =
        1.0; // 1 mechanism rotation per 1 motor rotation (direct drive)
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

    // PID and Feedforward Constants (VelocityVoltage control)
    public static final double KP = 0.5; // Proportional gain
    public static final double KI = 0.0; // Integral gain
    public static final double KD = 0.0; // Derivative gain
    public static final double KS = 0.25; // Static friction
    public static final double KV = 0.12; // Velocity feedforward
    public static final double KA = 0.01; // Acceleration feedforward

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 80.0;
    public static final double SUPPLY_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Shooter Speed Presets (rotations per second)
    public static final double NEUTRAL_ZONE_SPEED = 80.0; // Speed for shooting back in neutral zone

    // Velocity Tolerance
    public static final double VELOCITY_TOLERANCE = 2.0; // rotations per second

    // Test Mode Tuning
    public static final double TEST_MODE_RPS_INCREMENT = 5.0; // RPS increment per button press
  }

  public static class HoodConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 50;
    public static final CANBus CAN_BUS = new CANBus("Superstructure");
    public static final double GEAR_RATIO =
        1.0 / 12.6; // mechanism rotations per motor rotation (reduction)
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final double ROTOR_OFFSET = 0.228027; // Motor rotations

    // Mechanism Zero Angle
    // The actual mechanism angle when rotor sensor reads 0 (degrees from horizontal)
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
    // Note: These are in motor units since SensorToMechanismRatio = 1.0
    public static final double CRUISE_VELOCITY = 80.0 / GEAR_RATIO; // motor rotations per second
    public static final double ACCELERATION = 160.0 / GEAR_RATIO; // motor rotations per second^2
    public static final double JERK = 1600.0 / GEAR_RATIO; // motor rotations per second^3

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    // Position Tolerance
    public static final double AIMING_TOLERANCE_RAD = Units.degreesToRadians(1.0);

    // Position Setpoints (radians)
    public static final double STOW_POSITION = Units.degreesToRadians(21.0); // Safe stow angle
    public static final double MIN_AIM_ANGLE = Units.degreesToRadians(21.0); // Minimum angle

    // Test Mode Tuning
    public static final double TEST_MODE_ANGLE_INCREMENT =
        Units.degreesToRadians(1.0); // angle increment per button press
  }

  public static class ClimbConstants {
    // Hardware Configuration - 4 Winch Motors
    public static final int RIGHT_FRONT_MOTOR_CAN_ID = 51;
    public static final int RIGHT_BACK_MOTOR_CAN_ID = 52;
    public static final int LEFT_FRONT_MOTOR_CAN_ID = 53;
    public static final int LEFT_BACK_MOTOR_CAN_ID = 54;
    public static final CANBus CAN_BUS = new CANBus("Superstructure");

    // Passive Hook Release Servos (REV Smart Robot Servo, REV-41-1097)
    // Without an SRS Programmer the servo acts as a 270° standard servo
    // with a 500µs–2500µs pulse range.  PWM bounds are overridden in the IO
    // layer so set(0.0) = 0° and set(1.0) = 270°.
    public static final int RIGHT_HOOK_SERVO_PWM = 9;
    public static final int LEFT_HOOK_SERVO_PWM = 1;
    public static final double SRS_FULL_RANGE_DEG = 270.0; // degrees of travel
    // Servo positions as fraction of full range (0.0 = 0°, 1.0 = 270°)
    // Tune these to the actual stowed / released angles of your mechanism.
    public static final double HOOK_STOWED_POSITION = 0.0; // 0° — hooks locked
    public static final double HOOK_RELEASED_POSITION = 1.0; // 270° — hooks released

    // Gear Ratios: Mechanism rotations per motor rotation (speed reduction)
    // Front motors: 100:1 reduction → 1 motor rotation = 1/100 drum rotation
    // Back motors: 80:1 reduction → 1 motor rotation = 1/80 drum rotation
    // Used in both Phoenix SensorToMechanismRatio and IK calculations
    public static final double FRONT_GEAR_RATIO = 1.0 / 100.0; // 0.01
    public static final double BACK_GEAR_RATIO = 1.0 / 80.0; // 0.0125

    // Motor Inversion - Configure each motor independently
    public static final InvertedValue RIGHT_FRONT_MOTOR_INVERTED =
        InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue RIGHT_BACK_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue LEFT_FRONT_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue LEFT_BACK_MOTOR_INVERTED =
        InvertedValue.CounterClockwise_Positive;

    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

    // PID and Feedforward Constants
    public static final double KP = 10.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;
    public static final double KG = 0.0;

    // Motion Magic Constants (in MOTOR rotations per second)
    // Note: These are in motor units since SensorToMechanismRatio = 1.0 per CTRE recommendation
    // Front motors (100:1): 3.0 drum rot/s = 300 motor rot/s
    // Back motors (80:1): 3.0 drum rot/s = 240 motor rot/s
    // We'll use average for shared base config, specific conversions handled in IO layer
    public static final double CRUISE_VELOCITY = 3.0; // mechanism (drum) rotations per second
    public static final double ACCELERATION = 10.0; // mechanism (drum) rotations per second^2
    public static final double JERK = 40.0; // mechanism (drum) rotations per second^3

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 80.0;
    public static final double SUPPLY_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Position Setpoints (rotations) - Legacy direct motor control
    public static final double STOWED_POSITION = 0.0; // Starting position

    // Position Tolerance
    public static final double POSITION_TOLERANCE = 1.0; // rotations

    // ==================== Inverse Kinematics Configuration ====================

    // Fixed Points (all relative to back winch at origin)
    // Back winch (W_back) = (0, 0) — origin, drives the back cable
    // Front winch (W_front) — drives the front cable
    public static final double FRONT_WINCH_X_METERS = 0.345;
    public static final double FRONT_WINCH_Y_METERS = 0.145;
    // Shoulder joint (S) — fixed pivot where link 1 attaches to the frame
    public static final double SHOULDER_X_METERS = 0.305;
    public static final double SHOULDER_Y_METERS = 0.0;

    // Link Lengths
    public static final double LINK_1_LENGTH_METERS = 0.32; // Shoulder to elbow
    public static final double LINK_2_LENGTH_METERS = 0.45; // Elbow to end effector

    // Cable Attachment Offsets (distance along the link from the moving end)
    // Back cable attaches to link 1, BACK_CABLE_OFFSET meters from elbow toward shoulder
    public static final double BACK_CABLE_ATTACH_ON_LINK1_METERS = 0.03;
    // Front cable attaches to link 2, FRONT_CABLE_OFFSET meters from end effector toward elbow
    public static final double FRONT_CABLE_ATTACH_ON_LINK2_METERS = 0.18;

    // Cable Drum (on motor shaft that winds up cable)
    public static final double CABLE_DRUM_CIRCUMFERENCE_METERS =
        0.043992; // Circumference of cable drum (meters per rotation)

    // Mechanism Position Limits (safety limits in drum/mechanism rotations, not motor rotations)
    // Note: With SensorToMechanismRatio configured, position control uses mechanism rotations
    public static final double MIN_MECHANISM_POSITION = 0.0; // Minimum safe drum position
    public static final double MAX_MECHANISM_POSITION = 100.0; // Maximum safe drum position

    // Joint Angle Limits (safety limits in radians)
    public static final double MIN_SHOULDER_ANGLE_RAD = -Math.PI / 2; // -90 degrees
    public static final double MAX_SHOULDER_ANGLE_RAD = Math.PI / 2; // 90 degrees
    public static final double MIN_ELBOW_ANGLE_RAD = -Math.PI; // -180 degrees
    public static final double MAX_ELBOW_ANGLE_RAD = Math.PI; // 180 degrees
    // Set false for testing paths outside joint limits
    public static final boolean ENABLE_JOINT_LIMITS = false;

    // Workspace Limits (for end effector reachability checks)
    public static final double WORKSPACE_MIN_X_METERS = -0.2; // Minimum X position
    public static final double WORKSPACE_MAX_X_METERS = 1.0; // Maximum X position
    public static final double WORKSPACE_MIN_Y_METERS = 0.0; // Minimum Y position (ground)
    public static final double WORKSPACE_MAX_Y_METERS = 1.2; // Maximum Y position

    // Starting Position (cable has some initial extension, not fully retracted)
    // This represents where the end effector is when the robot powers on
    // Measure this position with the climb mechanism in its physical starting state
    // Initial cable lengths are computed automatically from this position by ClimbIK.
    public static final double START_POSITION_X_METERS = 0.37; // Forward offset from winch base
    public static final double START_POSITION_Y_METERS = 0.46; // Height above winch base

    // IK Solver Tolerance
    public static final double IK_POSITION_TOLERANCE_METERS = 0.005; // 5mm tolerance
    public static final double IK_ANGLE_TOLERANCE_RAD = 0.01; // ~0.57 degrees

    // ==================== Path Planning Constraints ====================

    // Cartesian path generation constraints (for ClimbPathPlanner)
    public static final double PATH_MAX_VELOCITY_MPS = 1.0; // Maximum end effector velocity (m/s)
    public static final double PATH_MAX_ACCELERATION_MPS2 =
        2.0; // Maximum end effector acceleration (m/s^2)

    // Velocity control feedforward (for pulling paths under load)
    public static final double VELOCITY_KG_PULLING =
        0.15; // Extra feedforward voltage when pulling robot (0-1.0)

    // Test Mode Tuning
    public static final double TEST_MODE_POSITION_INCREMENT =
        1.0; // rotations increment per button press
    public static final double TEST_MODE_VOLTAGE = 3.0; // volts for manual control
  }
}
