package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double upperVelocityRotPerSec = 0.0;
    public double upperAppliedVolts = 0.0;
    public double upperCurrentAmps = 0.0;
    public double upperTemperatureCelsius = 0.0;

    public double lowerVelocityRotPerSec = 0.0;
    public double lowerAppliedVolts = 0.0;
    public double lowerCurrentAmps = 0.0;
    public double lowerTemperatureCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run both intake motors at a specific percent output (-1.0 to 1.0). */
  public default void setPercent(double percent) {}

  /** Stop both intake motors. */
  public default void stop() {}
}
