package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.ShooterSetpoint;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Shooter subsystem: flywheel control with ShooterSetpoint for coordinated aiming. */
public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // ShooterSetpoint supplier for coordinated aiming
  private Supplier<ShooterSetpoint> setpointSupplier = ShooterSetpoint::invalid;

  // Track current velocity setpoint for logging
  private double velocitySetpointRPS = 0.0;

  /** Constructs a {@link ShooterSubsystem} subsystem instance */
  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  /** Set the ShooterSetpoint supplier (wired from RobotContainer). */
  public void setShooterSetpointSupplier(Supplier<ShooterSetpoint> supplier) {
    this.setpointSupplier = supplier;
  }

  /** Command to spin up using ShooterSetpoint (hub or neutral zone speed). */
  public Command spinUp() {
    return run(() -> {
          ShooterSetpoint setpoint = setpointSupplier.get();
          setVelocity(setpoint.getShooterRPS());
        })
        .withName("ShooterSpinUp");
  }

  /**
   * Command to stop the shooter
   *
   * @return A command that stops the shooter
   */
  public Command stop() {
    return runOnce(
            () -> {
              stopMotor();
            })
        .withName("ShooterStop");
  }

  /**
   * Sets the velocity of shooter motors
   *
   * @param velocityRotPerSec The target velocity in rotations per second
   */
  public void setVelocity(double velocityRotPerSec) {
    velocitySetpointRPS = velocityRotPerSec;
    io.setVelocity(velocityRotPerSec);
  }

  /**
   * Gets the shooter motor velocity
   *
   * @return The motor velocity in rotations per second
   */
  public double getVelocity() {
    return inputs.velocityRotPerSec;
  }

  /**
   * Checks if the shooter is at the target velocity
   *
   * @param targetVelocity The target velocity in rotations per second
   * @return True if motor is within tolerance of the target velocity
   */
  public boolean atVelocity(double targetVelocity) {
    return Math.abs(inputs.velocityRotPerSec - targetVelocity)
        < ShooterConstants.VELOCITY_TOLERANCE;
  }

  /**
   * Checks if the shooter is at the setpoint velocity from ShooterSetpoint.
   *
   * @return True if at the calculated setpoint velocity
   */
  public boolean isReady() {
    ShooterSetpoint setpoint = setpointSupplier.get();
    return atVelocity(setpoint.getShooterRPS()) && setpoint.isValid();
  }

  /** Check if shooter is stopped (for climb readiness) */
  public boolean atSetpoint() {
    return atVelocity(0.0);
  }

  /** Stops the shooter motors */
  public void stopMotor() {
    io.stop();
  }

  /** Apply spin-up via ShooterSetpoint (void, for Superstructure.periodic()). */
  public void applySpinUp() {
    ShooterSetpoint setpoint = setpointSupplier.get();
    setVelocity(setpoint.getShooterRPS());
  }

  /** Stator current (amps). Spikes when FUEL passes through — used for shot detection. */
  public double getCurrentAmps() {
    return inputs.currentAmps;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/VelocitySetpointRPS", velocitySetpointRPS);
    // Inline isReady() logic to avoid a second setpointSupplier.get() call this cycle
    ShooterSetpoint cachedSetpoint = setpointSupplier.get();
    Logger.recordOutput(
        "Shooter/IsReady", atVelocity(cachedSetpoint.getShooterRPS()) && cachedSetpoint.isValid());
  }
}
