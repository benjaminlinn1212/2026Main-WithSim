package frc.robot.subsystems.intakepivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {

  @AutoLog
  public static class IntakePivotIOInputs {
    public double rightPositionRotations = 0.0;
    public double rightVelocityRotPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
    public double rightTemperatureCelsius = 0.0;

    public double leftPositionRotations = 0.0;
    public double leftVelocityRotPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double leftTemperatureCelsius = 0.0;
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
