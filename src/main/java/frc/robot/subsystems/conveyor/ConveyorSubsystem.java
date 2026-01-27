package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import org.littletonrobotics.junction.Logger;

public class ConveyorSubsystem extends SubsystemBase {

  private final ConveyorIO io;
  private final ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

  /** Constructs a {@link ConveyorSubsystem} subsystem instance */
  public ConveyorSubsystem(ConveyorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Conveyor", inputs);
  }

  /**
   * Command to feed game piece to shooter
   *
   * @return A command that runs conveyor toward shooter
   */
  public Command goToShooter() {
    return run(() -> {
          io.setVoltage(ConveyorConstants.TO_SHOOTER_VOLTAGE);
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
          io.setVoltage(ConveyorConstants.TO_BUCKET_VOLTAGE);
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
