package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.ShooterSetpoint;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem that controls the flywheel(s) for launching game pieces. Now supports
 * ShooterSetpoint for coordinated aiming.
 */
public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // ShooterSetpoint supplier for coordinated aiming
  private Supplier<ShooterSetpoint> setpointSupplier =
      () -> new ShooterSetpoint(0, 0, 0, 0, 0, false);

  // Track current velocity setpoint for logging
  private double velocitySetpointRPS = 0.0;

  /** Constructs a {@link ShooterSubsystem} subsystem instance */
  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  /**
   * Set the shooter setpoint supplier for coordinated aiming. Call this from RobotContainer to
   * connect all aiming subsystems through ShooterSetpoint.
   */
  public void setShooterSetpointSupplier(Supplier<ShooterSetpoint> supplier) {
    this.setpointSupplier = supplier;
  }

  /**
   * Command to spin up the shooter using ShooterSetpoint calculations. This uses the coordinated
   * setpoint for optimal shooting (hub or neutral zone).
   *
   * @return A command that spins the shooter to the calculated speed
   */
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
  public Command stopShooter() {
    return runOnce(
            () -> {
              stop();
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
    return atVelocity(0.0); // Check if at zero velocity (stopped)
  }

  /** Stops the shooter motors */
  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/VelocitySetpointRPS", velocitySetpointRPS);
    Logger.recordOutput("Shooter/IsReady", isReady());
  }
}
