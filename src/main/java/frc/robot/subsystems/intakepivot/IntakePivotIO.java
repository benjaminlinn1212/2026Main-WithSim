package frc.robot.subsystems.intakepivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {

  @AutoLog
  public static class IntakePivotIOInputs {
    public double positionRotations = 0.0;
    public double velocityRotPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakePivotIOInputs inputs) {}

  /** Set the pivot motor to a target position. */
  public default void setPosition(double positionRotations) {}

  /** Run the pivot motor at a specific voltage. */
  public default void setVoltage(double volts) {}

  /** Stop the pivot motor. */
  public default void stop() {}
}
