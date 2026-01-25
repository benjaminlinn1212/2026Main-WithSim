package frc.robot.subsystems.indexer;

/** Hardware abstraction interface for the Indexer subsystem */
public interface IndexerIO {

  /** Inputs class for indexer data */
  public static class IndexerIOInputs {
    public double velocityRotPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Run indexer at specified voltage */
  public default void setVoltage(double volts) {}

  /** Stop the indexer motor */
  public default void stop() {}
}
