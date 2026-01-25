package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {

  private final IndexerIO io;
  private final IndexerIO.IndexerIOInputs inputs = new IndexerIO.IndexerIOInputs();

  private static IndexerSubsystem instance;

  /** Constructs a {@link IndexerSubsystem} subsystem instance */
  private IndexerSubsystem(IndexerIO io) {
    this.io = io;
  }

  /** Gets the singleton instance of the indexer subsystem */
  public static IndexerSubsystem getInstance() {
    if (instance == null) {
      if (RobotBase.isReal()) {
        instance = new IndexerSubsystem(new IndexerIOTalonFX());
      } else {
        instance = new IndexerSubsystem(new IndexerIOSim());
      }
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Log all inputs
    Logger.recordOutput("Indexer/VelocityRotPerSec", inputs.velocityRotPerSec);
    Logger.recordOutput("Indexer/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("Indexer/CurrentAmps", inputs.currentAmps);
    Logger.recordOutput("Indexer/TemperatureCelsius", inputs.temperatureCelsius);
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
