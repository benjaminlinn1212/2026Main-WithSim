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
    public static final String FRONT_LIMELIGHT_NAME = "limelight-right";
    public static final String BACK_LIMELIGHT_NAME = "limelight-left";
    public static final String TURRET_LIMELIGHT_NAME = "limelight-turret";

    // Camera positions relative to robot center (Translation3d: x forward, y left, z up)
    // Front camera (on drivetrain, front of robot)
    public static final Transform3d RIGHT_CAMERA_TO_ROBOT =
        new Transform3d(
            new Translation3d(0.318704, 0.183753, 0.292216),
            new Rotation3d(0, Units.degreesToRadians(20), 0));

    // Back camera (on drivetrain, back of robot)
    public static final Transform3d LEFT_CAMERA_TO_ROBOT =
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
    public static final double BUMPER_WIDTH_INCHES = 35.055;
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

    /**
     * Trench Teleop Assist tuning. When the robot is near a trench, two effects activate:
     *
     * <ol>
     *   <li><b>Chassis orientation alignment</b> — injects omega (rotational velocity) to LERP the
     *       robot's heading toward the nearest cardinal direction, so the bumpers fit through the
     *       22.25in tunnel. This modifies {@code omega}, not the velocity direction.
     *   <li><b>Lateral centering</b> — deflects the velocity vector toward the trench's center Y
     *       line, guiding the travel path toward the middle of the 48in corridor. Speed magnitude
     *       is preserved.
     * </ol>
     *
     * Both effects ramp from 0 at the buffer edge to full strength inside the trench.
     */
    public static class TrenchAssist {
      /** Maximum blend factor (0–1). 0.7 = the driver always retains at least 30% authority. */
      public static final double MAX_BLEND_FACTOR = 0.8;

      /**
       * Teleop approach buffer (meters). How far outside the trench walls the assist begins
       * ramping. Independent from {@link
       * frc.robot.auto.dashboard.FieldConstants#TRENCH_APPROACH_BUFFER} which is used by auto
       * heading snap and Superstructure hood stow. Larger = assist starts earlier, more time to
       * center.
       */
      public static final double APPROACH_BUFFER = 2.0;

      /**
       * Minimum speed (m/s) below which the assist is inactive. Prevents the assist from
       * interfering with fine positioning / stationary rotation near the trench.
       */
      public static final double MIN_SPEED_MPS = 0.5;

      /**
       * Maximum angular error (degrees) between the velocity vector and the nearest cardinal before
       * the assist activates. If the driver is traveling perpendicular to the trench, they clearly
       * don't intend to go through it — don't fight them.
       */
      public static final double MAX_HEADING_ERROR_DEG = 50.0;

      /**
       * Lateral centering strength (deg per meter of offset). The trench half-width is ~0.61m, so
       * 35°/m gives ~21° at the wall — a noticeable but gentle nudge toward center. Lower values
       * feel more like a suggestion, higher values feel like rails.
       */
      public static final double CENTERING_DEG_PER_METER = 45.0;

      /**
       * Maximum centering deflection angle (degrees). At 25° the lateral component is ~42% of speed
       * — enough to guide you toward center without stealing all your forward momentum.
       */
      public static final double MAX_CENTERING_DEG = 30.0;

      /**
       * Maximum omega correction (rad/s) for orientation alignment. Caps the injected rotational
       * rate so the trench assist doesn't spin the robot violently. The heading P-gain itself is
       * reused from {@link DriveToPose#ROTATION_KP} — no separate KP needed.
       */
      public static final double MAX_ORIENTATION_OMEGA_RAD_PER_SEC = 2.0;

      // === Wall Avoidance ===

      /**
       * Half the robot's bumper width in meters. Used to compute the position of the robot's bumper
       * edges relative to trench walls. 32in bumper → 16in → 0.406m.
       */
      public static final double ROBOT_HALF_WIDTH_M =
          Units.inchesToMeters(BUMPER_WIDTH_INCHES / 2.0);

      /**
       * Wall repulsion gain (m/s of lateral push per meter of encroachment into the danger zone).
       * Higher = harder virtual wall. At 4.0, a 0.1m encroachment produces 0.4 m/s of push-back —
       * firm enough to prevent wall contact without feeling like a spring.
       */
      public static final double WALL_REPULSION_MPS_PER_METER = 5.0;

      /**
       * Distance (meters) from a trench wall at which the repulsion force begins, measured from the
       * robot's <b>bumper edge</b> (the distance calculation already accounts for {@link
       * #ROBOT_HALF_WIDTH_M}). The trench has only ~0.20m of clearance per side, so this should be
       * small — just enough to catch the robot before contact. At 0.12m the repulsion activates
       * when the bumper is ~4.7in from the wall.
       */
      public static final double WALL_DANGER_ZONE_M = 0.12;
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
    // Note: KG is no longer used — gravity FF is computed by IntakePivotFF via linkage kinematics

    /** When true, use the IntakePivotFF linkage-based gravity feedforward. When false, no FF. */
    public static final boolean USE_CALCULATED_FF = false;

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

    // ─── Gravity Feedforward Linkage Geometry (IntakePivotFF) ───
    // All values in meters / radians / kg — MEASURE FROM CAD AND FILL IN.
    // See IntakePivotFF.java Javadoc for coordinate frame definitions.

    /** Pinion pitch radius (meters). Measure from CAD or datasheet. */
    public static final double FF_PINION_RADIUS_M = 0.012; // TODO: measure from CAD

    /** Motor-to-pinion gear ratio (motor rotations per pinion rotation). */
    public static final double FF_RACK_GEAR_RATIO = GEAR_RATIO;

    /**
     * Rack axis angle θ from +x (radians). 0 = horizontal rightward, π/2 = vertical upward. Measure
     * the rack's travel direction in the mechanism coordinate frame.
     */
    public static final double FF_RACK_THETA_RAD = 0.0; // TODO: measure from CAD

    /** Rack attachment point A₀ (meters) when rack displacement s = 0. */
    public static final double FF_A0X_M = 0.0; // TODO: measure from CAD

    public static final double FF_A0Y_M = 0.0; // TODO: measure from CAD

    /** V-link fixed pivot O (meters). */
    public static final double FF_OX_M = 0.0; // TODO: measure from CAD

    public static final double FF_OY_M = 0.0; // TODO: measure from CAD

    /** Distance from pivot O to elbow point E (meters). */
    public static final double FF_ELBOW_RADIUS_M = 0.0; // TODO: measure from CAD

    /** Coupler link length — between rack attachment A and elbow E (meters). */
    public static final double FF_COUPLER_LENGTH_M = 0.0; // TODO: measure from CAD

    /** Total moving mass hanging from the V-link (kg). */
    public static final double FF_MASS_KG = 0.0; // TODO: weigh or estimate from CAD

    /** Distance from pivot O to lumped center-of-mass (meters). */
    public static final double FF_COM_RADIUS_M = 0.0; // TODO: estimate from CAD

    /** COM angle offset δ relative to the elbow ray (radians). 0 if COM is on same ray as E. */
    public static final double FF_COM_ANGLE_OFFSET_RAD = 0.0; // TODO: estimate from CAD

    /** Motor torque constant Kt (N·m/A). Kraken X60 ≈ 0.0194, NEO ≈ 0.025. */
    public static final double FF_MOTOR_KT = 0.025; // TODO: set for your motor

    /** Motor winding resistance (Ω). NEO ≈ 0.114, Kraken X60 ≈ 0.025. */
    public static final double FF_MOTOR_R_OHM = 0.114; // TODO: set for your motor

    /** Efficiency fudge factor (0..1). Start at 0.85, tune on the real mechanism. */
    public static final double FF_EFFICIENCY = 0.85;
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

    // All climb servos are 180° with 500µs–2500µs pulse range.
    // PWM bounds are overridden in the IO layer so set(0.0) = 0° and set(1.0) = 180°.
    public static final double SERVO_FULL_RANGE_DEG = 180.0; // degrees of travel

    // Secondary Hook Angle Servos
    public static final int LEFT_SECONDARY_HOOK_ANGLE_SERVO_PWM = 2;
    public static final int RIGHT_SECONDARY_HOOK_ANGLE_SERVO_PWM = 6;
    public static final double SECONDARY_HOOK_ANGLE_STOWED_POSITION = 0.0;

    // Secondary Hook Hardstop Servos
    public static final int LEFT_SECONDARY_HOOK_HARDSTOP_SERVO_PWM = 4;
    public static final int RIGHT_SECONDARY_HOOK_HARDSTOP_SERVO_PWM = 7;
    public static final double SECONDARY_HOOK_HARDSTOP_STOWED_POSITION = 0.0;

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

    // Motion Magic Constants (in mechanism/drum rotations per second)
    // IO layer converts to motor units: motorVel = drumVel / GEAR_RATIO
    // Kraken/Falcon free speed ≈ 100 motor rot/s
    // Front motors (100:1): 0.8 drum rot/s = 80 motor rot/s (80% of free speed)
    // Back motors (80:1): 0.8 drum rot/s = 64 motor rot/s (64% of free speed)
    public static final double CRUISE_VELOCITY = 0.8; // mechanism (drum) rotations per second
    public static final double ACCELERATION = 3.0; // mechanism (drum) rotations per second^2
    public static final double JERK = 12.0; // mechanism (drum) rotations per second^3

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
    public static final double FRONT_WINCH_X_METERS = 0.341;
    public static final double FRONT_WINCH_Y_METERS = 0.13208;
    // Shoulder joint (S) — fixed pivot where link 1 attaches to the frame
    public static final double SHOULDER_X_METERS = 0.3048;
    public static final double SHOULDER_Y_METERS = 0.0;

    // Link Lengths
    public static final double LINK_1_LENGTH_METERS = 0.32; // Shoulder to elbow
    public static final double LINK_2_LENGTH_METERS = 0.42547; // Elbow to end effector

    // Cable Attachment Offsets (distance along the link from the moving end)
    // Back cable attaches to link 1, BACK_CABLE_OFFSET meters from elbow toward shoulder
    public static final double BACK_CABLE_ATTACH_ON_LINK1_METERS = 0.02684;
    // Front cable attaches to link 2, FRONT_CABLE_OFFSET meters from end effector toward elbow
    public static final double FRONT_CABLE_ATTACH_ON_LINK2_METERS = 0.15267;

    // Cable Drum (on motor shaft that winds up cable)
    public static final double CABLE_DRUM_CIRCUMFERENCE_METERS =
        0.0439941; // Base circumference of bare cable drum (meters per rotation, layer 0)

    // Cable Layer Buildup — as cable stacks on the drum, effective circumference grows.
    // One full layer = ROTATIONS_PER_LAYER rotations. When a new layer starts, the
    // circumference increases by CIRCUMFERENCE_PER_LAYER_METERS (step function, not linear).
    public static final double ROTATIONS_PER_LAYER = 4.0; // rotations to fill one layer
    public static final double CIRCUMFERENCE_PER_LAYER_METERS =
        0.012; // +1.2 cm circumference per layer

    // At the STOW pose, the drum already has cable wound on it.
    // Layer 0 (4 rot) + Layer 1 (4 rot) + 2 rot into Layer 2 = 10 total absolute rotations.
    // Effective circumference at stow = C0 + 2 * 0.012 = C0 + 2.4 cm (layer 2).
    public static final double STOW_ABSOLUTE_ROTATIONS = 10.0;

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
    // NOTE: Opened wide for testing — tighten these after validating real mechanism range
    public static final double WORKSPACE_MIN_X_METERS = -5.0; // Minimum X position
    public static final double WORKSPACE_MAX_X_METERS = 5.0; // Maximum X position
    public static final double WORKSPACE_MIN_Y_METERS = -5.0; // Minimum Y position
    public static final double WORKSPACE_MAX_Y_METERS = 5.0; // Maximum Y position

    // Starting Position (cable has some initial extension, not fully retracted)
    // This represents where the end effector is when the robot powers on
    // Measure this position with the climb mechanism in its physical starting state
    // Initial cable lengths are computed automatically from this position by ClimbIK.
    public static final double START_POSITION_X_METERS = 0.356; // Forward offset from winch base
    public static final double START_POSITION_Y_METERS = 0.43; // Height above winch base

    // IK Solver Tolerance
    public static final double IK_POSITION_TOLERANCE_METERS = 0.005; // 5mm tolerance
    public static final double IK_ANGLE_TOLERANCE_RAD = 0.01; // ~0.57 degrees

    // ==================== Path Planning Constraints ====================

    // Cartesian path generation constraints (for ClimbPathPlanner)
    public static final double PATH_MAX_VELOCITY_MPS = 10.0; // Maximum end effector velocity (m/s)
    public static final double PATH_MAX_ACCELERATION_MPS2 =
        10.0; // Maximum end effector acceleration (m/s^2)

    // Velocity control feedforward (for pulling paths under load)
    public static final double VELOCITY_KG_PULLING =
        0.15; // Extra feedforward voltage when pulling robot (0-1.0)

    // Test Mode Tuning
    public static final double TEST_MODE_POSITION_INCREMENT =
        1.0; // rotations increment per button press
    public static final double TEST_MODE_VOLTAGE = 3.0; // volts for manual control
  }
}
