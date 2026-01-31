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
   * Command to spin up the shooter to a specific velocity
   *
   * @param velocityRotPerSec The target velocity in rotations per second
   * @return A command that spins the shooter to the target velocity
   */
  public Command spinUp(double velocityRotPerSec) {
    return run(() -> {
          setVelocity(velocityRotPerSec);
        })
        .withName("ShooterSpinUp");
  }

  /**
   * Command to spin up the shooter to hub shooting speed
   *
   * @return A command that spins the shooter to hub speed
   */
  public Command spinUpForHub() {
    return spinUp(ShooterConstants.HUB_SPEED);
  }

  /**
   * Command to spin up the shooter using ShooterSetpoint calculations. This uses the coordinated
   * setpoint for optimal shooting.
   *
   * @return A command that spins the shooter to the calculated speed
   */
  public Command spinUpForAim() {
    return run(() -> {
          ShooterSetpoint setpoint = setpointSupplier.get();
          setVelocity(setpoint.getShooterRPS());
        })
        .withName("ShooterSpinUpForAim");
  }

  /**
   * Command to spin up the shooter to pass shooting speed
   *
   * @return A command that spins the shooter to pass speed
   */
  public Command spinUpForPass() {
    return spinUp(ShooterConstants.PASS_SPEED);
  }

  /**
   * Command to idle the shooter at a low speed
   *
   * @return A command that idles the shooter
   */
  public Command idle() {
    return run(() -> {
          setVelocity(ShooterConstants.IDLE_SPEED);
        })
        .withName("ShooterIdle");
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
   * Checks if the shooter is ready to shoot at hub speed
   *
   * @return True if at hub speed
   */
  public boolean readyForHub() {
    return atVelocity(ShooterConstants.HUB_SPEED);
  }

  /**
   * Checks if the shooter is ready to shoot at pass speed
   *
   * @return True if at pass speed
   */
  public boolean readyForPass() {
    return atVelocity(ShooterConstants.PASS_SPEED);
  }

  /**
   * Checks if the shooter is at the setpoint velocity from ShooterSetpoint.
   *
   * @return True if at the calculated setpoint velocity
   */
  public boolean readyForAim() {
    ShooterSetpoint setpoint = setpointSupplier.get();
    return atVelocity(setpoint.getShooterRPS()) && setpoint.getIsValid();
  }

  /** Stops the shooter motors */
  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/ReadyForHub", readyForHub());
    Logger.recordOutput("Shooter/ReadyForPass", readyForPass());
    Logger.recordOutput("Shooter/ReadyForAim", readyForAim());
  }
}
