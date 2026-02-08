package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

  @AutoLog
  public static class ClimbIOInputs {
    // Right Front Motor
    public double rightFrontPositionRotations = 0.0;
    public double rightFrontVelocityRotPerSec = 0.0;
    public double rightFrontAppliedVolts = 0.0;
    public double rightFrontCurrentAmps = 0.0;
    public double rightFrontTemperatureCelsius = 0.0;

    // Right Back Motor
    public double rightBackPositionRotations = 0.0;
    public double rightBackVelocityRotPerSec = 0.0;
    public double rightBackAppliedVolts = 0.0;
    public double rightBackCurrentAmps = 0.0;
    public double rightBackTemperatureCelsius = 0.0;

    // Left Front Motor
    public double leftFrontPositionRotations = 0.0;
    public double leftFrontVelocityRotPerSec = 0.0;
    public double leftFrontAppliedVolts = 0.0;
    public double leftFrontCurrentAmps = 0.0;
    public double leftFrontTemperatureCelsius = 0.0;

    // Left Back Motor
    public double leftBackPositionRotations = 0.0;
    public double leftBackVelocityRotPerSec = 0.0;
    public double leftBackAppliedVolts = 0.0;
    public double leftBackCurrentAmps = 0.0;
    public double leftBackTemperatureCelsius = 0.0;

    // Passive Hook Release Servos
    public double leftHookServoPosition = 0.0;
    public double rightHookServoPosition = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Set the right front motor to a target position. */
  public default void setRightFrontPosition(double positionRotations) {}

  /** Set the right back motor to a target position. */
  public default void setRightBackPosition(double positionRotations) {}

  /** Set the left front motor to a target position. */
  public default void setLeftFrontPosition(double positionRotations) {}

  /** Set the left back motor to a target position. */
  public default void setLeftBackPosition(double positionRotations) {}

  /** Run the right front motor at a specific voltage. */
  public default void setRightFrontVoltage(double volts) {}

  /** Run the right back motor at a specific voltage. */
  public default void setRightBackVoltage(double volts) {}

  /** Run the left front motor at a specific voltage. */
  public default void setLeftFrontVoltage(double volts) {}

  /** Run the left back motor at a specific voltage. */
  public default void setLeftBackVoltage(double volts) {}

  /** Set passive hook servo positions (0.0 = stowed, 1.0 = released). */
  public default void setLeftHookPosition(double position) {}

  /** Set passive hook servo positions (0.0 = stowed, 1.0 = released). */
  public default void setRightHookPosition(double position) {}

  /** Stop all climb motors. */
  public default void stop() {}
}
