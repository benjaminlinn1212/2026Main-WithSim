package frc.robot.subsystems.intake;

/** Simulation implementation of IntakeIO. This provides basic intake simulation without physics. */
public class IntakeIOSim implements IntakeIO {

  private double appliedVolts = 0.0;

  public IntakeIOSim() {
    // Simple simulation without physics
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
    // Simulate motor velocity based on voltage
    inputs.velocityRotPerSec = appliedVolts * 10; // Rough simulation
  }

  @Override
  public void setVoltage(double volts) {
    this.appliedVolts = volts;
  }

  @Override
  public void stop() {
    this.appliedVolts = 0.0;
  }
}
