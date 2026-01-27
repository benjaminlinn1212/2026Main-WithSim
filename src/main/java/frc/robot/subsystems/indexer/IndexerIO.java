package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction interface for the Indexer subsystem */
public interface IndexerIO {

  /** Inputs class for indexer data */
  @AutoLog
  public static class IndexerIOInputs {
    public double velocityRotPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Run indexer at specified duty cycle */
  public default void setDutyCycle(double dutyCycle) {}

  /** Stop the indexer motors */
  public default void stop() {}
}
