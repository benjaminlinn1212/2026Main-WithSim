package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

  private static ShooterSubsystem system;

  /** Constructs a {@link ShooterSubsystem} subsystem instance */
  private ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  /**
   * Command to spin up the shooter to a specific duty cycle
   *
   * @param dutyCycle The target duty cycle (-1.0 to 1.0)
   * @return A command that spins the shooter to the target duty cycle
   */
  public Command spinUp(double dutyCycle) {
    return run(() -> {
          setDutyCycle(dutyCycle);
        })
        .withName("ShooterSpinUp");
  }

  /**
   * Command to spin up the shooter to hub shooting speed
   *
   * @return A command that spins the shooter to hub speed
   */
  public Command spinUpForHub() {
    return spinUp(ShooterConstants.HUB_DUTY_CYCLE);
  }

  /**
   * Command to spin up the shooter to pass shooting speed
   *
   * @return A command that spins the shooter to pass speed
   */
  public Command spinUpForPass() {
    return spinUp(ShooterConstants.PASS_DUTY_CYCLE);
  }

  /**
   * Command to idle the shooter at a low speed
   *
   * @return A command that idles the shooter
   */
  public Command idle() {
    return run(() -> {
          setDutyCycle(ShooterConstants.IDLE_DUTY_CYCLE);
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
   * Sets the duty cycle of shooter motors
   *
   * @param dutyCycle The target duty cycle (-1.0 to 1.0)
   */
  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
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
   * Checks if the shooter is ready to shoot (simplified - always true for duty cycle control)
   *
   * @return Always returns true since we're using duty cycle control
   */
  public boolean readyForHub() {
    return true;
  }

  /**
   * Checks if the shooter is ready to pass (simplified - always true for duty cycle control)
   *
   * @return Always returns true since we're using duty cycle control
   */
  public boolean readyForPass() {
    return true;
  }

  /** Stops the shooter motors */
  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Shooter/VelocityRotPerSec", inputs.velocityRotPerSec);
    Logger.recordOutput("Shooter/CurrentAmps", inputs.currentAmps);
  }

  /**
   * Gets the {@link ShooterSubsystem} subsystem instance
   *
   * @return The {@link ShooterSubsystem} subsystem instance
   */
  public static ShooterSubsystem system() {
    if (system == null) {
      if (RobotBase.isSimulation()) {
        system = new ShooterSubsystem(new ShooterIOSim());
      } else {
        system = new ShooterSubsystem(new ShooterIOTalonFX());
      }
    }

    return system;
  }

  /** Gets the {@link ShooterSubsystem} subsystem instance (alias for system()) */
  public static ShooterSubsystem getInstance() {
    return system();
  }
}
