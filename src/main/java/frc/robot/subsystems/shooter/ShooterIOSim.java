package frc.robot.subsystems.shooter;

/**
 * Simulation implementation of ShooterIO interface. This class provides simulated shooter behavior
 * for testing without hardware.
 */
public class ShooterIOSim implements ShooterIO {
  private double velocityRotPerSec = 0.0;
  private double positionRot = 0.0;
  private double appliedVolts = 0.0;

  private double targetDutyCycle = 0.0;

  private static final double ACCELERATION = 100.0; // rot/s^2
  private static final double MAX_VELOCITY = 80.0; // Max rotations per second at full duty cycle

  public ShooterIOSim() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Simulate acceleration towards target velocity based on duty cycle
    double dt = 0.02; // 20ms loop time

    double targetVelocity = targetDutyCycle * MAX_VELOCITY;

    // Motor simulation
    double error = targetVelocity - velocityRotPerSec;
    double accel = Math.signum(error) * Math.min(Math.abs(error) / dt, ACCELERATION);
    velocityRotPerSec += accel * dt;
    positionRot += velocityRotPerSec * dt;
    appliedVolts = targetDutyCycle * 12.0; // Simulate battery voltage

    inputs.velocityRotPerSec = velocityRotPerSec;
    inputs.positionRot = positionRot;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(appliedVolts) * 5.0; // Simulated current
    inputs.tempCelsius = 25.0 + Math.abs(velocityRotPerSec) * 0.1; // Temp rises with speed
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    this.targetDutyCycle = dutyCycle;
  }

  @Override
  public void stop() {
    targetDutyCycle = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    this.appliedVolts = volts;
    // Convert voltage to duty cycle
    this.targetDutyCycle = volts / 12.0;
  }
}
