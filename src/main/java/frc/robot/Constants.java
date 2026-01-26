// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
    // Motor CAN IDs - 2 motors synced, opposite direction
    public static final int LEADER_MOTOR_CAN_ID = 40;
    public static final int FOLLOWER_MOTOR_CAN_ID = 41;
    public static final String CAN_BUS = "rio";

    // Motor inversion
    public static final boolean LEADER_INVERTED = true;
    public static final boolean FOLLOWER_INVERTED = false;

    // Neutral mode
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 80.0;
    public static final double SUPPLY_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Shooter duty cycle presets (-1.0 to 1.0)
    public static final double IDLE_DUTY_CYCLE = 0.0;
    public static final double HUB_DUTY_CYCLE = 1.0;
    public static final double PASS_DUTY_CYCLE = 0.3;
  }

  public static class IntakeConstants {
    // Motor CAN ID - Single REV NEO Vortex
    public static final int MOTOR_CAN_ID = 42;
    public static final String CAN_BUS = "rio";

    // Neutral mode
    public static final IdleMode NEUTRAL_MODE = IdleMode.kCoast;

    // Current limit (amps)
    public static final int CURRENT_LIMIT = 40;

    // Intake percent output (0.0 to 1.0)
    public static final double INTAKE_PERCENT = -0.3;
    public static final double OUTTAKE_PERCENT = 0.3;
  }

  public static class IntakePivotConstants {
    // Motor CAN IDs - 2 motors synced, opposite direction
    public static final int LEADER_MOTOR_CAN_ID = 43;
    public static final int FOLLOWER_MOTOR_CAN_ID = 44;
    public static final String CAN_BUS = "rio";

    // Motor inversion
    public static final boolean LEADER_INVERTED = false;
    public static final boolean FOLLOWER_INVERTED = true; // Opposite of leader

    // Neutral mode
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

    // Motor encoder offset (in rotations)
    // Set this to the current position when the mechanism is at its zero/reference position
    public static final double MOTOR_ROTOR_OFFSET = -0.287;

    // Soft limits (in rotations)
    public static final double SOFT_LIMIT_REVERSE = 0.0; // Reverse soft limit (minimum position)
    public static final double SOFT_LIMIT_FORWARD = 28.0; // Forward soft limit (maximum position)

    // PID constants
    public static final double KP = 0.05;
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
    public static final double DEPLOYED_POSITION = 5.4; // Extended/down position for intaking

    // Position tolerance
    public static final double POSITION_TOLERANCE = 0.5; // rotations
  }

  public static class ConveyorConstants {
    // Motor CAN ID
    public static final int MOTOR_CAN_ID = 45;
    public static final String CAN_BUS = "rio";

    // Neutral mode
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    // Conveyor duty cycles (-1.0 to 1.0)
    public static final double TO_SHOOTER_DUTY_CYCLE = 0.5; // 50% speed toward shooter
    public static final double TO_BUCKET_DUTY_CYCLE = -0.5; // 50% speed toward bucket
  }
}
