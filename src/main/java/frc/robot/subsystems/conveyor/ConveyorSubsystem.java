package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import org.littletonrobotics.junction.Logger;

public class ConveyorSubsystem extends SubsystemBase {

  private final ConveyorIO io;
  private final ConveyorIO.ConveyorIOInputs inputs = new ConveyorIO.ConveyorIOInputs();

  private static ConveyorSubsystem instance;

  /** Constructs a {@link ConveyorSubsystem} subsystem instance */
  private ConveyorSubsystem(ConveyorIO io) {
    this.io = io;
  }

  /** Gets the singleton instance of the conveyor subsystem */
  public static ConveyorSubsystem getInstance() {
    if (instance == null) {
      if (RobotBase.isReal()) {
        instance = new ConveyorSubsystem(new ConveyorIOTalonFX());
      } else {
        instance = new ConveyorSubsystem(new ConveyorIOSim());
      }
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Log all inputs
    Logger.recordOutput("Conveyor/VelocityRotPerSec", inputs.velocityRotPerSec);
    Logger.recordOutput("Conveyor/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("Conveyor/CurrentAmps", inputs.currentAmps);
    Logger.recordOutput("Conveyor/TemperatureCelsius", inputs.temperatureCelsius);
  }

  /**
   * Command to feed game piece to shooter
   *
   * @return A command that runs conveyor toward shooter
   */
  public Command goToShooter() {
    return run(() -> {
          io.setDutyCycle(ConveyorConstants.TO_SHOOTER_DUTY_CYCLE);
        })
        .withName("ConveyorToShooter");
  }

  /**
   * Command to move game piece to bucket (for ejecting)
   *
   * @return A command that runs conveyor toward bucket
   */
  public Command goToBucket() {
    return run(() -> {
          io.setDutyCycle(ConveyorConstants.TO_BUCKET_DUTY_CYCLE);
        })
        .withName("ConveyorToBucket");
  }

  /**
   * Command to stop the conveyor
   *
   * @return A command that stops the conveyor
   */
  public Command stop() {
    return runOnce(
            () -> {
              io.stop();
            })
        .withName("ConveyorStop");
  }

  /** Immediately stop the conveyor motor */
  public void stopMotor() {
    io.stop();
  }
}
