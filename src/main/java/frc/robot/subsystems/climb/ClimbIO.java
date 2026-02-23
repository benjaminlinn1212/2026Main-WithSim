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

    // Secondary Hook Angle Servos
    public double leftSecondaryHookAngleServoPosition = 0.0;
    public double rightSecondaryHookAngleServoPosition = 0.0;

    // Secondary Hook Hardstop Servos
    public double leftSecondaryHookHardstopServoPosition = 0.0;
    public double rightSecondaryHookHardstopServoPosition = 0.0;
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

  /** Set the right front motor to a target velocity. */
  public default void setRightFrontVelocity(double velocityRotPerSec, double feedforwardVolts) {}

  /** Set the right back motor to a target velocity. */
  public default void setRightBackVelocity(double velocityRotPerSec, double feedforwardVolts) {}

  /** Set the left front motor to a target velocity. */
  public default void setLeftFrontVelocity(double velocityRotPerSec, double feedforwardVolts) {}

  /** Set the left back motor to a target velocity. */
  public default void setLeftBackVelocity(double velocityRotPerSec, double feedforwardVolts) {}

  /** Run the right front motor at a specific voltage. */
  public default void setRightFrontVoltage(double volts) {}

  /** Run the right back motor at a specific voltage. */
  public default void setRightBackVoltage(double volts) {}

  /** Run the left front motor at a specific voltage. */
  public default void setLeftFrontVoltage(double volts) {}

  /** Run the left back motor at a specific voltage. */
  public default void setLeftBackVoltage(double volts) {}

  /** Set left secondary hook angle servo position. */
  public default void setLeftSecondaryHookAnglePosition(double position) {}

  /** Set right secondary hook angle servo position. */
  public default void setRightSecondaryHookAnglePosition(double position) {}

  /** Set left secondary hook hardstop servo position. */
  public default void setLeftSecondaryHookHardstopPosition(double position) {}

  /** Set right secondary hook hardstop servo position. */
  public default void setRightSecondaryHookHardstopPosition(double position) {}

  /** Stop all climb motors. */
  public default void stop() {}

  /**
   * Reset all motor positions (and velocities) to the STOWED cable-length rotations. Used in SIM to
   * re-initialize the climb when restarting auto without restarting the whole program.
   */
  public default void resetToStowed() {}

  /**
   * Re-seed encoder positions to match the initial STOWED end-effector pose IK solution. Used after
   * calibration mode to realign the encoder frame of reference so FK/IK produces correct positions.
   * On real hardware this calls setPosition() on each motor; in SIM it resets the simulated
   * positions.
   */
  public default void recalibrateEncoders() {}
}
