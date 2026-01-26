package frc.robot.subsystems.intake;

/** Simulation implementation of IntakeIO. This provides basic intake simulation without physics. */
public class IntakeIOSim implements IntakeIO {

  private double appliedPercent = 0.0;

  public IntakeIOSim() {
    // Simple simulation without physics
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedVolts = appliedPercent * 12.0; // Assume 12V max
    inputs.velocityRotPerSec = appliedPercent * 80.0; // Rough simulation
  }

  @Override
  public void setPercent(double percent) {
    this.appliedPercent = percent;
  }

  @Override
  public void stop() {
    this.appliedPercent = 0.0;
  }
}
