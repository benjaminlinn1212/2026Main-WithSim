package frc.robot.subsystems.turret;

public interface TurretIO {

  public static class TurretIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double absolutePosition = 0.0;
    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Set the turret position setpoint in radians with feedforward velocity */
  public default void setPositionSetpoint(double positionRad, double ffVelocity) {}

  /** Set open loop duty cycle (-1.0 to 1.0) for manual control */
  public default void setOpenLoopDutyCycle(double dutyCycle) {}

  /** Stop the turret */
  public default void stop() {}
}
