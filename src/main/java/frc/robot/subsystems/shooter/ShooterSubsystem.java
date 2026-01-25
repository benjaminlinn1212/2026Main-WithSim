package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  /** Stops the shooter motors */
  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Shooter/VelocityRotPerSec", inputs.velocityRotPerSec);
    Logger.recordOutput("Shooter/PositionRot", inputs.positionRot);
    Logger.recordOutput("Shooter/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("Shooter/CurrentAmps", inputs.currentAmps);
    Logger.recordOutput("Shooter/TempCelsius", inputs.tempCelsius);

    SmartDashboard.putNumber("Shooter Velocity", inputs.velocityRotPerSec);
    SmartDashboard.putBoolean("Shooter Ready Hub", readyForHub());
    SmartDashboard.putBoolean("Shooter Ready Pass", readyForPass());
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
        System.out.println("ShooterSubsystem: Using simulation IO");
      } else {
        system = new ShooterSubsystem(new ShooterIOTalonFX());
        System.out.println("ShooterSubsystem: Using TalonFX hardware IO");
      }
    }

    return system;
  }

  /** Gets the {@link ShooterSubsystem} subsystem instance (alias for system()) */
  public static ShooterSubsystem getInstance() {
    return system();
  }
}
