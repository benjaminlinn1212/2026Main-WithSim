package frc.robot.subsystems.intake;

public interface IntakeIO {

  public static class IntakeIOInputs {
    public double velocityRotPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the intake motor at a specific voltage. */
  public default void setVoltage(double volts) {}

  /** Stop the intake motor. */
  public default void stop() {}
}
