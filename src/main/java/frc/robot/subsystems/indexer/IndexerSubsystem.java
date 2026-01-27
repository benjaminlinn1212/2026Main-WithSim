package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  /** Constructs a {@link IndexerSubsystem} subsystem instance */
  public IndexerSubsystem(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  /**
   * Command to feed game piece to shooter
   *
   * @return A command that runs indexer toward shooter
   */
  public Command toShooter() {
    return run(() -> {
          io.setVoltage(IndexerConstants.TO_SHOOTER_VOLTAGE);
        })
        .withName("IndexerToShooter");
  }

  /**
   * Command to stop the indexer
   *
   * @return A command that stops the indexer
   */
  public Command stop() {
    return runOnce(
            () -> {
              io.stop();
            })
        .withName("IndexerStop");
  }

  /** Stop indexer motor immediately (for emergency use) */
  public void stopMotor() {
    io.stop();
  }
}
