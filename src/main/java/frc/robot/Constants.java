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
    // Limelight names
    public static final String FRONT_LIMELIGHT_NAME = "limelight-right";
    public static final String TURRET_LIMELIGHT_NAME = "limelight-turret";

    // Camera positions relative to robot center (Translation3d: x forward, y left, z up)
    // Front camera (on drivetrain, front of robot)
    public static final Transform3d RIGHT_CAMERA_TO_ROBOT =
        new Transform3d(
            new Translation3d(0.318704, 0.183753, 0.292216),
            new Rotation3d(0, Units.degreesToRadians(20), 0));

    // Turret camera (on turret, offset from turret center)
    // This is the transform from TURRET CENTER to camera
    public static final Transform3d TURRET_TO_CAMERA =
        new Transform3d(
            new Translation3d(
                0.185643, 0.0, 0.52803), // 18.5643cm forward from turret axis, 53cm up
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
    public static final double PATH_FOLLOWING_TRANSLATION_KP =
        3.0; // Translation (along/cross track combined)
    public static final double PATH_FOLLOWING_ROTATION_KP = 5.0; // Rotation error

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
            0.0, 0.0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
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

    public static class DriveToPose {
      // This constraint is used in the driveToPose() function by PPLib AND PIDControl.
      public static final double TRANSLATION_KP = 2.0; // Reduced from 5 to reduce oscillation
      public static final double TRANSLATION_KI = 0.0;
      public static final double TRANSLATION_KD = 0.2; // Increased to dampen oscillation
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
     *   <li><b>Chassis orientation alignment</b> â€” injects omega (rotational velocity) to LERP
     *       the robot's heading toward the nearest cardinal direction, so the bumpers fit through
     *       the 22.25in tunnel. This modifies {@code omega}, not the velocity direction.
     *   <li><b>Lateral centering</b> â€” deflects the velocity vector toward the trench's center Y
     *       line, guiding the travel path toward the middle of the 48in corridor. Speed magnitude
     *       is preserved.
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
       * don't intend to go through it â€” don't fight them.
       */
      public static final double MAX_HEADING_ERROR_DEG = 50.0;

      /**
       * Lateral centering strength (deg per meter of offset). The trench half-width is ~0.61m, so
       * 35Â°/m gives ~21Â° at the wall â€” a noticeable but gentle nudge toward center. Lower
       * values feel more like a suggestion, higher values feel like rails.
       */
      public static final double CENTERING_DEG_PER_METER = 45.0;

      /**
       * Maximum centering deflection angle (degrees). At 25Â° the lateral component is ~42% of
       * speed â€” enough to guide you toward center without stealing all your forward momentum.
       */
      public static final double MAX_CENTERING_DEG = 30.0;

      /**
       * Maximum omega correction (rad/s) for orientation alignment. Caps the injected rotational
       * rate so the trench assist doesn't spin the robot violently. The heading P-gain itself is
       * reused from {@link DriveToPose#ROTATION_KP} â€” no separate KP needed.
       */
      public static final double MAX_ORIENTATION_OMEGA_RAD_PER_SEC = 4.0;

      // === Wall Avoidance ===

      /**
       * Half the robot's bumper width in meters. Used to compute the position of the robot's bumper
       * edges relative to trench walls. 32in bumper â†’ 16in â†’ 0.406m.
       */
      public static final double ROBOT_HALF_WIDTH_M =
          Units.inchesToMeters(BUMPER_WIDTH_INCHES / 2.0);

      /**
       * Wall repulsion gain (m/s of lateral push per meter of encroachment into the danger zone).
       * Higher = harder virtual wall. At 4.0, a 0.1m encroachment produces 0.4 m/s of push-back â€”
       * firm enough to prevent wall contact without feeling like a spring.
       */
      public static final double WALL_REPULSION_MPS_PER_METER = 6.0;

      /**
       * Distance (meters) from a trench wall at which the repulsion force begins, measured from the
       * robot's <b>bumper edge</b> (the distance calculation already accounts for {@link
       * #ROBOT_HALF_WIDTH_M}). The trench has only ~0.20m of clearance per side, so this should be
       * small â€” just enough to catch the robot before contact. At 0.12m the repulsion activates
       * when the bumper is ~4.7in from the wall.
       */
      public static final double WALL_DANGER_ZONE_M = 0.12;
    }
  }

  public static class IntakeConstants {
    // Hardware Configuration
    public static final int UPPER_MOTOR_CAN_ID = 41;
    public static final int LOWER_MOTOR_CAN_ID = 42;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
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
  }

  public static class IntakePivotConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 43;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
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
    // Note: KG is no longer used â€” gravity FF is computed by IntakePivotFF via linkage kinematics

    /** When true, use the IntakePivotFF linkage-based gravity feedforward. When false, no FF. */
    public static final boolean USE_CALCULATED_FF = false;

    // Motion Magic Constants
    public static final double CRUISE_VELOCITY = 100.0; // rotations per second
    public static final double ACCELERATION = 300.0; // rotations per second^2
    public static final double JERK = 2000.0; // rotations per second^3

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Position Setpoints (rotations)
    public static final double STOWED_POSITION = 0.0; // Stowed/up position
    public static final double HALF_DEPLOYED_POSITION = 22.0; // Halfway between stowed and deployed
    public static final double DEPLOYED_POSITION = 27.4; // Extended/down position for intaking

    // Position Tolerance
    public static final double POSITION_TOLERANCE = 0.5; // rotations

    // â”€â”€â”€ Gravity Feedforward Linkage Geometry (IntakePivotFF) â”€â”€â”€
    // All values in meters / radians / kg â€” MEASURE FROM CAD AND FILL IN.
    // See IntakePivotFF.java Javadoc for coordinate frame definitions.

    /** Number of teeth on the pinion gear. Read directly from the gear datasheet or CAD. */
    public static final int FF_PINION_NUM_TEETH = 16;

    /** Number of teeth on the rack. Determines total linear travel. Count from CAD or datasheet. */
    public static final int FF_RACK_NUM_TEETH = 50; // TODO: count from rack

    /**
     * Rack tooth-to-tooth distance (meters). Measure the linear distance between adjacent tooth
     * centers on the rack. This equals the circular pitch of the meshing gear pair.
     */
    public static final double FF_RACK_TOOTH_DISTANCE_M = 0.00643196; // TODO: measure from rack

    /**
     * Total rack travel (meters), derived from rack tooth count and tooth distance. Do not set
     * directly â€” change RACK_NUM_TEETH and RACK_TOOTH_DISTANCE instead.
     */
    public static final double FF_RACK_TRAVEL_M = FF_RACK_NUM_TEETH * FF_RACK_TOOTH_DISTANCE_M;

    /**
     * Pinion pitch radius (meters), derived from tooth count and rack tooth distance. r = N * d /
     * (2Ï€). Do not set directly â€” change NUM_TEETH and RACK_TOOTH_DISTANCE instead.
     */
    public static final double FF_PINION_RADIUS_M =
        FF_PINION_NUM_TEETH * FF_RACK_TOOTH_DISTANCE_M / (2.0 * Math.PI);

    /** Motor-to-pinion gear ratio (motor rotations per pinion rotation). */
    public static final double FF_RACK_GEAR_RATIO = GEAR_RATIO;

    /**
     * Rack axis angle Î¸ from +x (radians). 0 = horizontal rightward, Ï€/2 = vertical upward.
     * Measure the rack's travel direction in the mechanism coordinate frame.
     */
    public static final double FF_RACK_THETA_RAD = 0.0; // TODO: measure from CAD

    /** Rack attachment point Aâ‚€ (meters) when rack displacement s = 0. */
    public static final double FF_A0X_M = 0.0; // TODO: measure from CAD

    public static final double FF_A0Y_M = 0.0; // TODO: measure from CAD

    /** V-link fixed pivot O (meters). */
    public static final double FF_OX_M = 0.0; // TODO: measure from CAD

    public static final double FF_OY_M = 0.0; // TODO: measure from CAD

    /** Distance from pivot O to elbow point E (meters). */
    public static final double FF_ELBOW_RADIUS_M = 0.0; // TODO: measure from CAD

    /** Coupler link length â€” between rack attachment A and elbow E (meters). */
    public static final double FF_COUPLER_LENGTH_M = 0.0; // TODO: measure from CAD

    /** Total moving mass hanging from the V-link (kg). */
    public static final double FF_MASS_KG = 0.0; // TODO: weigh or estimate from CAD

    /** Distance from pivot O to lumped center-of-mass (meters). */
    public static final double FF_COM_RADIUS_M = 0.0; // TODO: estimate from CAD

    /** COM angle offset Î´ relative to the elbow ray (radians). 0 if COM is on same ray as E. */
    public static final double FF_COM_ANGLE_OFFSET_RAD = 0.0; // TODO: estimate from CAD

    /** Motor torque constant Kt (NÂ·m/A). Kraken X60 â‰ˆ 0.0194, NEO â‰ˆ 0.025. */
    public static final double FF_MOTOR_KT = 0.025; // TODO: set for your motor

    /** Motor winding resistance (Î©). NEO â‰ˆ 0.114, Kraken X60 â‰ˆ 0.025. */
    public static final double FF_MOTOR_R_OHM = 0.114; // TODO: set for your motor

    /** Efficiency fudge factor (0..1). Start at 0.85, tune on the real mechanism. */
    public static final double FF_EFFICIENCY = 0.85;
  }

  public static class ConveyorConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 45;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
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
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
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

    // Indexer Percent Output
    public static final double TO_SHOOTER_PERCENT = 0.7;
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
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
    public static final double GEAR_RATIO =
        1.0 / 26.812; // mechanism rotations per motor rotation (reduction)
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

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
    public static final double ACCELERATION = 10.0 / GEAR_RATIO; // motor rotations/secÂ²
    public static final double JERK = 100.0 / GEAR_RATIO; // motor rotations/secÂ³

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 150.0;
    public static final double SUPPLY_CURRENT_LIMIT = 80.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Position Tolerance (radians)
    public static final double AIMING_TOLERANCE_RAD = Units.degreesToRadians(2.0);

    // Boot Position (radians) â€” the mechanism angle when the robot powers on.
    // The turret physically starts at -90Â° (facing right when viewed from above).
    // TurretIOTalonFX seeds the encoder to this value via motor.setPosition() at boot
    // because FeedbackRotorOffset is limited to [0,1) motor rotations and cannot
    // represent the multi-rotation offset required by the 26.8:1 gear ratio.
    public static final double BOOT_POSITION_RAD = Units.degreesToRadians(-90.0);

    // Position Setpoints (radians)
    public static final double STOW_POSITION = Units.degreesToRadians(-90.0); // Facing right
  }

  public static class ShooterConstants {
    // Hardware Configuration
    public static final int MOTOR_CAN_ID = 49;
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
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
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;
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
    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;

    // ==================== Secondary Hook Servo Configuration ====================
    //
    // Two servo types:
    //   Angle servos   â€” 180Â° travel, 500Âµsâ€“2500Âµs pulse range
    //   Hardstop servos â€” 100Â° travel, 1000Âµsâ€“2000Âµs pulse range
    //
    // Inversion convention (all servos are CCW-positive when non-inverted):
    //   Non-inverted: set(0.0) = 0Â°,   requesting +NÂ° moves 0Â° â†’ NÂ°
    //   Inverted:     set(0.0) = maxÂ°, requesting +NÂ° moves maxÂ° â†’ (maxâˆ’N)Â°
    //   In code: pwmValue = inverted ? (1.0 âˆ’ input) : input
    //
    // Per-side inversion:
    //   Right angle   â†’ NOT inverted   (CCW-positive, 0Â°â†’180Â°)
    //   Right hardstop â†’ INVERTED      (CW-positive,  100Â°â†’0Â°)
    //   Left angle    â†’ INVERTED       (CW-positive,  180Â°â†’0Â°)
    //   Left hardstop â†’ NOT inverted   (CCW-positive, 0Â°â†’100Â°)

    /** Angle servo config: 180Â° range, 500Âµsâ€“2500Âµs pulse. */
    public static class AngleServo {
      public static final double FULL_RANGE_DEG = 180.0;
      public static final int PULSE_MIN_US = 500;
      public static final int PULSE_MAX_US = 2500;
      public static final double STOWED_POSITION = 40.0 / 180.0; // 0.0â€“1.0 (before inversion)
      public static final double RELEASED_POSITION = 180.0 / 180.0; // 150Â° â†’ 0.833 (before inv.)
      /** Time (seconds) for the angle servo to travel its full 180Â° range. */
      public static final double TRAVEL_TIME_SEC = 1.0;
    }

    /** Hardstop servo config: 100Â° range, 1000Âµsâ€“2000Âµs pulse. */
    public static class HardstopServo {
      public static final double FULL_RANGE_DEG = 100.0;
      public static final int PULSE_MIN_US = 1000;
      public static final int PULSE_MAX_US = 2000;
      public static final double STOWED_POSITION = 30.0 / 100.0; // 0.0â€“1.0 (before inversion)
      public static final double RELEASED_POSITION = 87.0 / 100.0; // 52Â° â†’ 0.52 (before inv.)
      /** Time (seconds) for the hardstop servo to travel its full 100Â° range. */
      public static final double TRAVEL_TIME_SEC = 0.7;
    }

    // PWM Channels
    public static final int RIGHT_ANGLE_SERVO_PWM = 6;
    public static final int RIGHT_HARDSTOP_SERVO_PWM = 7;
    public static final int LEFT_ANGLE_SERVO_PWM = 8;
    public static final int LEFT_HARDSTOP_SERVO_PWM = 9;

    // Per-servo inversion flags (true = CW-positive, maxÂ° at set(0.0))
    public static final boolean RIGHT_ANGLE_SERVO_INVERTED = false;
    public static final boolean RIGHT_HARDSTOP_SERVO_INVERTED = true;
    public static final boolean LEFT_ANGLE_SERVO_INVERTED = true;
    public static final boolean LEFT_HARDSTOP_SERVO_INVERTED = false;

    // Gear Ratios: Mechanism rotations per motor rotation (speed reduction)
    // Front motors: 100:1 reduction â†’ 1 motor rotation = 1/100 drum rotation
    // Back motors: 80:1 reduction â†’ 1 motor rotation = 1/80 drum rotation
    // Used in both Phoenix SensorToMechanismRatio and IK calculations
    public static final double FRONT_GEAR_RATIO = 1.0 / 100.0;
    public static final double BACK_GEAR_RATIO = 1.0 / 80.0;

    // Motor Inversion - Configure each motor independently
    public static final InvertedValue RIGHT_FRONT_MOTOR_INVERTED =
        InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue RIGHT_BACK_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue LEFT_FRONT_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue LEFT_BACK_MOTOR_INVERTED =
        InvertedValue.CounterClockwise_Positive;

    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

    // PID and Feedforward Constants (Slot 0 â€” MotionMagicVoltage position control)
    public static final double KP = 10.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;
    public static final double KG = 0.0;

    // Velocity PID and Feedforward Constants (Slot 1 â€” VelocityVoltage path following)
    // Matched to ShooterConstants velocity control gains
    public static final double VELOCITY_KP = 0.5;
    public static final double VELOCITY_KI = 0.0;
    public static final double VELOCITY_KD = 0.0;
    public static final double VELOCITY_KS = 0.25;
    public static final double VELOCITY_KV = 0.12;
    public static final double VELOCITY_KA = 0.01;

    // Motion Magic Constants (in mechanism/drum rotations per second)
    // IO layer converts to motor units: motorVel = drumVel / GEAR_RATIO
    // Derived from Kraken/Falcon free speed (~100 motor rot/s) and gear ratios.
    // Limited by the slower (front) gear ratio to keep both motor pairs within safe speed.
    // CRUISE_VELOCITY = MOTOR_FREE_SPEED * FRONT_GEAR_RATIO * SPEED_UTILIZATION
    public static final double MOTOR_FREE_SPEED_RPS = 100.0; // Kraken/Falcon free speed (rot/s)
    public static final double SPEED_UTILIZATION = 0.80; // 80% of free speed
    public static final double CRUISE_VELOCITY =
        MOTOR_FREE_SPEED_RPS * FRONT_GEAR_RATIO * SPEED_UTILIZATION; // mechanism (drum) rot/s
    public static final double ACCELERATION =
        CRUISE_VELOCITY * 3.75; // mechanism (drum) rotations per second^2
    public static final double JERK = ACCELERATION * 4.0; // mechanism (drum) rotations per second^3

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
    // Back winch (W_back) = (0, 0) â€” origin, drives the back cable
    // Front winch (W_front) â€” drives the front cable
    public static final double FRONT_WINCH_X_METERS = 0.341;
    public static final double FRONT_WINCH_Y_METERS = 0.13208;
    // Shoulder joint (S) â€” fixed pivot where link 1 attaches to the frame
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

    // Cable Layer Buildup â€” as cable stacks on the drum, effective circumference grows.
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
    public static final double MIN_SHOULDER_ANGLE_RAD = Math.toRadians(85);
    public static final double MAX_SHOULDER_ANGLE_RAD = Math.toRadians(160);
    public static final double MIN_ELBOW_ANGLE_RAD = Math.toRadians(-140);
    public static final double MAX_ELBOW_ANGLE_RAD = Math.toRadians(-30);
    // Set false for testing paths outside joint limits
    public static final boolean ENABLE_JOINT_LIMITS = true;

    // Workspace Limits (for end effector reachability checks)
    // NOTE: Opened wide for testing â€” tighten these after validating real mechanism range
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
    // Derived from cruise velocity and drum geometry so they stay consistent
    // when gear ratios change.
    //   Cable speed = CRUISE_VELOCITY * effective drum circumference
    //   Use mid-layer circumference (~layer 1) as a conservative average.
    private static final double MID_LAYER_CIRCUMFERENCE_M =
        CABLE_DRUM_CIRCUMFERENCE_METERS + CIRCUMFERENCE_PER_LAYER_METERS; // ~layer 1
    public static final double PATH_MAX_VELOCITY_MPS =
        CRUISE_VELOCITY * MID_LAYER_CIRCUMFERENCE_M; // Maximum end effector velocity (m/s)
    public static final double PATH_MAX_ACCELERATION_MPS2 =
        ACCELERATION * MID_LAYER_CIRCUMFERENCE_M; // Maximum end effector acceleration (m/s^2)

    // Velocity control feedforward (for pulling paths under load)
    public static final double VELOCITY_KG_PULLING =
        0.15; // Extra feedforward voltage when pulling robot (0-1.0)

    // Test Mode Tuning
    public static final double TEST_MODE_POSITION_INCREMENT =
        1.0; // rotations increment per button press
    public static final double TEST_MODE_VOLTAGE = 3.0; // volts for manual control
  }

  public static class OrchestraConstants {
    /**
     * Name of the .chrp file to play. Place the file in {@code src/main/deploy/} — it will be
     * automatically deployed to the roboRIO. Convert MIDI → CHRP using Phoenix Tuner X's CHRP
     * Converter (Tools → CHRP Converter).
     */
    public static final String CHRP_FILE = "4trackCruelAngel.chrp";

    /**
     * When true, applies the {@code AllowMusicDurDisable} audio config so Orchestra can play even
     * when the robot is disabled. Useful for pre-match entertainment.
     */
    public static final boolean ALLOW_MUSIC_DURING_DISABLE = true;

    /**
     * Number of tracks in the .chrp file. Motors are distributed across tracks using modulo — if
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
    // (TurretConstants + MechanismVisualization) — no need to duplicate values here.

    /**
     * Height above floor at which the fuel leaves the shooter (meters). Derived from turret height
     * + hood offset above turret.
     */
    public static final double PROJECTILE_INITIAL_HEIGHT_METERS =
        MechanismVisualization.TURRET_HEIGHT_M + MechanismVisualization.HOOD_Z_ABOVE_TURRET_M;

    /**
     * Launch-speed scaling factor. Actual launch speed (m/s) = flywheel RPS / 100 *
     * LAUNCH_SPEED_SCALE. Example: 50 RPS → 5 m/s at scale 10, 80 RPS → 8 m/s.
     */
    public static final double LAUNCH_SPEED_SCALE = 10.0;

    /**
     * Compression / efficiency multiplier (0–1). On a real robot the ball compresses against the
     * flywheel and hood, so exit speed is lower than ideal surface speed. 1.0 = no loss, 0.7 = 30%
     * speed loss. Tune this until sim trajectories match real-robot shot distances.
     */
    public static final double FUEL_SPEED_EFFICIENCY = 1.15;

    /** Minimum flywheel velocity (rot/s) before the sim will launch a projectile. */
    public static final double MIN_LAUNCH_VELOCITY_RPS = 10.0;

    /** Minimum allowed launch angle (degrees) — clamp for hood safety. */
    public static final double MIN_LAUNCH_ANGLE_DEG = 15.0;

    /** Maximum allowed launch angle (degrees) — clamp for hood safety. */
    public static final double MAX_LAUNCH_ANGLE_DEG = 75.0;

    // --- Conveyor / Fuel Transfer ---
    /**
     * Conveyor percent-output threshold to trigger feeding toward the shooter (negative = feed).
     */
    public static final double CONVEYOR_FEED_THRESHOLD = -0.05;

    /** Cooldown ticks (× 20 ms) between consecutive fuel transfers from intake → shooter. */
    public static final int FEED_COOLDOWN_TICKS = 7;
  }

  // ==================== 3D Mechanism Visualization (AdvantageScope) ====================
  // Component layout for articulated 3D rendering in AdvantageScope's 3D Field tab.
  // Components (indexed 0–2 in the robot model config):
  //   0 = Turret   — yaw rotation, mounted on frame
  //   1 = Hood     — pitch rotation, mounted on turret
  //   2 = IntakePivot — pitch rotation, mounted on frame rear
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
     * model with the actual mechanism — positive tilts the model further up.
     */
    public static final double HOOD_PITCH_OFFSET_RAD = Units.degreesToRadians(-48.0);

    // --- Component 2: Intake (rack-and-pinion linear extension) ---
    /** Height of the intake mount point above the floor (meters). */
    public static final double INTAKE_PIVOT_HEIGHT_M = 0.25;
    /** X offset of the intake mount from robot center (negative = rear of robot, meters). */
    public static final double INTAKE_PIVOT_X_M = -0.35;
    /** Y offset of the intake mount from robot center (meters). Centered. */
    public static final double INTAKE_PIVOT_Y_M = 0.0;

    /** Total linear travel of the intake rack when fully extended (meters). */
    public static final double INTAKE_FULL_TRAVEL_M = 0.28;
    /**
     * Tilt angle of the intake slide rail from horizontal (radians). 6.278° → mostly rearward,
     * slightly downward.
     */
    public static final double INTAKE_TILT_RAD = Units.degreesToRadians(6.278);
    /** Motor rotations at full extension (soft-limit forward). */
    public static final double INTAKE_FULL_TRAVEL_ROTATIONS =
        IntakePivotConstants.SOFT_LIMIT_FORWARD;

    /**
     * X component of full intake extension (meters). 6.278° from horizontal → cos is the large X
     * part.
     */
    public static final double INTAKE_EXTENSION_X_M =
        INTAKE_FULL_TRAVEL_M * Math.cos(INTAKE_TILT_RAD);
    /**
     * Z component of full intake extension (meters). 6.278° from horizontal → sin is the small Z
     * part.
     */
    public static final double INTAKE_EXTENSION_Z_M =
        INTAKE_FULL_TRAVEL_M * Math.sin(INTAKE_TILT_RAD);
  }
}
