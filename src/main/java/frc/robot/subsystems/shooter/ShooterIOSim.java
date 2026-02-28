package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

/**
 * Simulation implementation of ShooterIO interface. Uses WPILib's DCMotorSim for realistic Kraken
 * X60 flywheel physics and integrates with maple-sim for projectile launching. Motor constants (KV,
 * gear ratio) come from ShooterConstants — no hardcoded motor physics.
 */
public class ShooterIOSim implements ShooterIO {
  /** WPILib physics simulation for the flywheel motor. */
  private final DCMotorSim flywheelSim;

  private double appliedVolts = 0.0;

  /** Reference to the drivetrain simulation for getting pose and chassis speeds. */
  private final SwerveDriveSimulation driveSimulation;

  /**
   * Supplier for the current turret angle in radians (field-relative offset from robot heading).
   */
  private final DoubleSupplier turretAngleRadSupplier;

  /** Supplier for the current hood angle in radians (from horizontal). */
  private final DoubleSupplier hoodAngleRadSupplier;

  /**
   * Number of fuel queued for launch (set by ConveyorIOSim when fuel is obtained from intake).
   * Using a counter instead of a boolean so multiple fuel can be queued.
   */
  private static int fuelQueuedCount = 0;

  public ShooterIOSim(
      SwerveDriveSimulation driveSimulation,
      DoubleSupplier turretAngleRadSupplier,
      DoubleSupplier hoodAngleRadSupplier) {
    this.driveSimulation = driveSimulation;
    this.turretAngleRadSupplier = turretAngleRadSupplier;
    this.hoodAngleRadSupplier = hoodAngleRadSupplier;

    // Use WPILib DCMotorSim with real motor model and gear ratio from constants
    // Shooter is direct-drive (GEAR_RATIO = 1.0), MOI ≈ 0.004 kg*m² for a typical flywheel
    DCMotor motor = DCMotor.getKrakenX60(1);
    double gearing = 1.0 / ShooterConstants.GEAR_RATIO; // motor-to-mechanism reduction
    double jKgMetersSquared = 0.004; // flywheel moment of inertia
    flywheelSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, jKgMetersSquared, gearing), motor);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Apply voltage and step the physics sim
    flywheelSim.setInputVoltage(appliedVolts);
    flywheelSim.update(0.02); // 20ms loop time

    // Read real simulated outputs
    inputs.velocityRotPerSec = flywheelSim.getAngularVelocityRPM() / 60.0; // RPM -> RPS
    inputs.positionRot = flywheelSim.getAngularPositionRotations();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(flywheelSim.getCurrentDrawAmps());
    inputs.temperatureCelsius = 25.0; // constant in sim

    // If there is fuel queued and the shooter is spinning fast enough, launch it
    if (fuelQueuedCount > 0
        && Math.abs(inputs.velocityRotPerSec) > Constants.SimConstants.MIN_LAUNCH_VELOCITY_RPS) {
      launchFuelProjectile(inputs.velocityRotPerSec);
      fuelQueuedCount--;
    }
  }

  @Override
  public void setVelocity(double velocityRotPerSec) {
    // Use the real ShooterConstants feedforward gains (same as TalonFX config)
    // V = kS * sign(v) + kV * v
    appliedVolts =
        ShooterConstants.KS * Math.signum(velocityRotPerSec)
            + ShooterConstants.KV * velocityRotPerSec;
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    this.appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  /**
   * Notify the simulated shooter that a fuel is ready to be launched. Called from ConveyorIOSim
   * when fuel is obtained from the intake. Increments the queue counter.
   */
  public static void notifyFuelReady() {
    fuelQueuedCount++;
  }

  /**
   * Launch a fuel projectile using the current drivetrain pose, turret angle, and shooter state.
   *
   * @param currentVelocityRPS the current flywheel velocity in rotations per second (from sim)
   */
  private void launchFuelProjectile(double currentVelocityRPS) {
    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    ChassisSpeeds fieldRelativeSpeeds =
        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();

    // Get the turret angle (radians, relative to robot heading)
    double turretAngleRad = turretAngleRadSupplier.getAsDouble();

    // Shooter facing = robot heading + turret angle
    Rotation2d shooterFacing = robotPose.getRotation().plus(Rotation2d.fromRadians(turretAngleRad));

    // Shooter exit XY = turret center on the robot
    Translation2d shooterOffsetOnRobot = Constants.TurretConstants.TURRET_OFFSET_FROM_ROBOT_CENTER;

    // Get the hood angle (radians from horizontal) and compute launch angle = 90 - hood
    double hoodAngleDeg = Math.toDegrees(hoodAngleRadSupplier.getAsDouble());
    double launchAngleDeg = 90.0 - hoodAngleDeg;
    // Clamp to a sane range in case hood hasn't been set yet
    launchAngleDeg =
        Math.max(
            Constants.SimConstants.MIN_LAUNCH_ANGLE_DEG,
            Math.min(launchAngleDeg, Constants.SimConstants.MAX_LAUNCH_ANGLE_DEG));

    // Launch speed proportional to actual flywheel RPS, with compression efficiency applied
    double launchSpeedMps =
        Math.abs(currentVelocityRPS)
            / 100.0
            * Constants.SimConstants.LAUNCH_SPEED_SCALE
            * Constants.SimConstants.FUEL_SPEED_EFFICIENCY;

    RebuiltFuelOnFly fuelOnFly =
        new RebuiltFuelOnFly(
            robotPose.getTranslation(),
            shooterOffsetOnRobot,
            fieldRelativeSpeeds,
            shooterFacing,
            Meters.of(Constants.SimConstants.PROJECTILE_INITIAL_HEIGHT_METERS),
            MetersPerSecond.of(launchSpeedMps),
            Degrees.of(launchAngleDeg));

    // Configure trajectory visualization
    fuelOnFly.withProjectileTrajectoryDisplayCallBack(
        (poses) ->
            Logger.recordOutput("Flywheel/FuelProjectileSuccessful", poses.toArray(Pose3d[]::new)),
        (poses) ->
            Logger.recordOutput("Flywheel/FuelProjectileMissed", poses.toArray(Pose3d[]::new)));

    // Let it become a game piece on the field after touchdown
    fuelOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();

    // Add to the simulated arena
    SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);

    Logger.recordOutput("Flywheel/LaunchSpeedMps", launchSpeedMps);
    Logger.recordOutput("Flywheel/LaunchAngleDeg", launchAngleDeg);
  }
}
