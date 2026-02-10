// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

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

  public static class FieldPoses {
    public static final double FIELD_LENGTH = 16.54; // meters
    public static final double FIELD_WIDTH = 8.07; // meters
    public static final Translation2d FIELD_CENTER =
        new Translation2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0);
    public static final Translation3d BLUE_HUB_TRANSLATION3D =
        new Translation3d(4.625689, 4.040981, 0);
    public static final Translation3d RED_HUB_POSE_TRANSLATION3D =
        new Translation3d(16.54175 - 4.625689, 4.040981, 0);
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

    // Turret position relative to robot center (should match TurretConstants offset)
    // Positive X = forward, Positive Y = left (meters)
    public static final Translation2d TURRET_TO_ROBOT_CENTER =
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
    public static final double PATHFINDING_MAX_ACCELERATION_MPS2 = 3.0;
    public static final double PATHFINDING_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.toRadians(360);
    public static final double PATHFINDING_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC2 =
        Math.toRadians(540);

    // Default pose for gyro/pose reset
    public static final edu.wpi.first.math.geometry.Pose2d DEFAULT_RESET_POSE =
        new edu.wpi.first.math.geometry.Pose2d(
            0.0, 0.0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
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
    public static final String CAN_BUS = "Superstructure";
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
    public static final String CAN_BUS = "Superstructure";
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
    public static final double DEPLOYED_POSITION = 27.6; // Extended/down position for intaking

    // Position Tolerance
    public static final double POSITION_TOLERANCE = 0.5; // rotations
  }

  public static class ConveyorConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 45;
    public static final String CAN_BUS = "Superstructure";
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
    public static final double TO_SHOOTER_PERCENT = 0.5;
    public static final double TO_BUCKET_PERCENT = -0.5;
  }

  public static class IndexerConstants {
    // Hardware Configuration
    public static final int LEADER_MOTOR_CAN_ID = 47;
    public static final int FOLLOWER_MOTOR_CAN_ID = 48;
    public static final String CAN_BUS = "Superstructure";
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
    public static final int FEEDFORWARD_FILTER_TAPS = 10; // Moving average filter size

    // Neutral Zone Shot Settings
    public static final double NEUTRAL_ZONE_HOOD_ANGLE_DEG =
        35.0; // Hood angle for neutral zone shots
  }

  public static class TurretConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 44;
    public static final String CAN_BUS = "Superstructure";
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
    public static final double MIN_POSITION_RAD = Units.rotationsToRadians(-0.5); // -π radians
    public static final double MAX_POSITION_RAD = Units.rotationsToRadians(0.5); // +π radians

    // PID and Feedforward Constants (MotionMagicVoltage mode)
    public static final double KP = 10.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
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
    public static final String CAN_BUS = "Superstructure";
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
    public static final double TEST_MODE_RPS_INCREMENT = 1.0; // RPS increment per button press
  }

  public static class HoodConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 50;
    public static final String CAN_BUS = "Superstructure";
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
    public static final String CAN_BUS = "Superstructure";

    // Passive Hook Release Servos
    public static final int LEFT_HOOK_SERVO_PWM = 0;
    public static final int RIGHT_HOOK_SERVO_PWM = 1;
    public static final double HOOK_STOWED_POSITION = 0.0; // Hooks locked
    public static final double HOOK_RELEASED_POSITION = 1.0; // Hooks released

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
    public static final double EXTENDED_POSITION = 50.0; // Fully extended
    public static final double RETRACTED_POSITION = 5.0; // Pulled up on chain

    // Position Tolerance
    public static final double POSITION_TOLERANCE = 1.0; // rotations

    // ==================== Inverse Kinematics Configuration ====================

    // Winch Separation (distance between W1 and W2)
    public static final double WINCH_SEPARATION_METERS =
        0.304818; // Distance between front and back winches

    // Link Lengths (adjust these to match your actual mechanism)
    public static final double LINK_1_LENGTH_METERS =
        0.32; // Length of first link (shoulder to elbow)
    public static final double LINK_2_LENGTH_METERS =
        0.445; // Length of second link (elbow to end effector)

    // Cable Attachment Offsets (on links)
    public static final double CABLE_1_OFFSET_METERS =
        0.171; // Cable 1 attachment point on link 1 (p)
    public static final double CABLE_2_OFFSET_METERS =
        0.03; // Cable 2 attachment point on link 2 (q)

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

    // Initial Cable Length Calibration
    // CRITICAL: These must be the actual cable lengths when the mechanism is at START_POSITION.
    // If these don't match, the IK will output non-zero rotations at startup, causing the
    // motors to move immediately. Calculate using ClimbIK.calculateCableLengths(START_X, START_Y).
    // With current geometry (a=0.304818, b=0.32, c=0.445, p=0.171, q=0.03):
    //   At START_POSITION (0.39, 0.43): l1 ≈ 0.1935, l2 ≈ 0.4167
    public static final double FRONT_CABLE_INITIAL_LENGTH_METERS = 0.1935;
    public static final double BACK_CABLE_INITIAL_LENGTH_METERS = 0.4167;

    // Starting Position (cable has some initial extension, not fully retracted)
    // This represents where the end effector is when the robot powers on
    // Measure this position with the climb mechanism in its physical starting state
    public static final double START_POSITION_X_METERS = 0.39; // Forward offset from winch base
    public static final double START_POSITION_Y_METERS = 0.43; // Height above winch base

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
