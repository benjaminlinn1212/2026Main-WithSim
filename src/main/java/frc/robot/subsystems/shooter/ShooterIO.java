package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double velocityRotPerSec = 0.0;
    public double positionRot = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set the duty cycle (-1.0 to 1.0) */
  public default void setDutyCycle(double dutyCycle) {}

  /** Stop the shooter motor */
  public default void stop() {}

  /** Set voltage for characterization */
  public default void setVoltage(double volts) {}
}
