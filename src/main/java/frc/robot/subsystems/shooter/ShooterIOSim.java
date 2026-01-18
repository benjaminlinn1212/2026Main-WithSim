package frc.robot.subsystems.shooter;

/**
 * Simulation implementation of ShooterIO interface. This class provides simulated shooter behavior
 * for testing without hardware.
 */
public class ShooterIOSim implements ShooterIO {
  private double velocityRotPerSec = 0.0;
  private double positionRot = 0.0;
  private double appliedVolts = 0.0;

  private double targetVelocity = 0.0;

  private static final double ACCELERATION = 100.0; // rot/s^2
  private static final double VOLTAGE_PER_RPS = 0.12; // Volts per rotation per second

  public ShooterIOSim() {
    System.out.println("ShooterIOSim: Initialized");
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Simulate acceleration towards target velocity
    double dt = 0.02; // 20ms loop time

    // Motor simulation
    double error = targetVelocity - velocityRotPerSec;
    double accel = Math.signum(error) * Math.min(Math.abs(error) / dt, ACCELERATION);
    velocityRotPerSec += accel * dt;
    positionRot += velocityRotPerSec * dt;
    appliedVolts = velocityRotPerSec * VOLTAGE_PER_RPS;

    inputs.velocityRotPerSec = velocityRotPerSec;
    inputs.positionRot = positionRot;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(appliedVolts) * 5.0; // Simulated current
    inputs.tempCelsius = 25.0 + Math.abs(velocityRotPerSec) * 0.1; // Temp rises with speed
  }

  @Override
  public void setVelocity(double velocityRotPerSec) {
    this.targetVelocity = velocityRotPerSec;
  }

  @Override
  public void stop() {
    targetVelocity = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    this.appliedVolts = volts;
    // Simplified voltage to velocity conversion
    this.targetVelocity = volts / VOLTAGE_PER_RPS;
  }
}
