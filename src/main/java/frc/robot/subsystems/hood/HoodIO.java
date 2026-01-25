package frc.robot.subsystems.hood;

public interface HoodIO {

  public static class HoodIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  /** Set the hood angle setpoint in radians with velocity */
  public default void setPositionSetpoint(double positionRad, double velocityRadPerSec) {}

  /** Set open loop duty cycle (-1.0 to 1.0) for manual control */
  public default void setOpenLoopDutyCycle(double dutyCycle) {}

  /** Stop the hood */
  public default void stop() {}
}
