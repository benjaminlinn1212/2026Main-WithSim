package frc.robot.subsystems.conveyor;

public interface ConveyorIO {

  public static class ConveyorIOInputs {
    public double velocityRotPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ConveyorIOInputs inputs) {}

  /** Run the conveyor motor at a specific voltage. */
  public default void setVoltage(double volts) {}

  /** Stop the conveyor motor. */
  public default void stop() {}
}
