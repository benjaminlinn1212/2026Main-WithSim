package frc.robot.subsystems.climb;

public interface ClimbIO {

  public static class ClimbIOInputs {
    public double positionRotations = 0.0;
    public double velocityRotPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Set the climb motor to a target position. */
  public default void setPosition(double positionRotations) {}

  /** Run the climb motor at a specific voltage. */
  public default void setVoltage(double volts) {}

  /** Stop the climb motor. */
  public default void stop() {}
}
