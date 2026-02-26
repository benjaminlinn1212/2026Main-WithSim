package frc.robot.subsystems.shooter;

/**
 * Simulation implementation of ShooterIO interface. Uses simplified physics for Kraken X60 motor.
 */
public class ShooterIOSim implements ShooterIO {
  private double velocityRotPerSec = 0.0;
  private double positionRot = 0.0;
  private double appliedVolts = 0.0;

  private static final double ACCELERATION = 120.0; // rot/s^2 (Kraken X60 is faster)
  private static final double KV = 0.1; // Volts per rotation per second (Kraken X60)

  public ShooterIOSim() {
    System.out.println("ShooterIOSim: Initialized with Kraken X60 model");
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double dt = 0.02; // 20ms loop time

    // Simplified motor physics for Kraken X60
    double targetVelocity = appliedVolts / KV;
    double error = targetVelocity - velocityRotPerSec;
    double accel = Math.signum(error) * Math.min(Math.abs(error) / dt, ACCELERATION);
    velocityRotPerSec += accel * dt;
    positionRot += velocityRotPerSec * dt;

    inputs.velocityRotPerSec = velocityRotPerSec;
    inputs.positionRot = positionRot;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(appliedVolts) * 8.0; // Kraken draws more current
    inputs.temperatureCelsius = 25.0 + Math.abs(velocityRotPerSec) * 0.1;
  }

  @Override
  public void setVelocity(double velocityRotPerSec) {
    // Control via voltage for velocity control (simplified)
    appliedVolts = velocityRotPerSec * 0.12; // kV approximation
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    this.appliedVolts = volts;
  }
}
