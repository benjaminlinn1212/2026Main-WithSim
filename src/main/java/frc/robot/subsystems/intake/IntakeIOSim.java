package frc.robot.subsystems.intake;

/** Simulation implementation of IntakeIO. This provides basic intake simulation without physics. */
public class IntakeIOSim implements IntakeIO {

  private double appliedPercent = 0.0;

  public IntakeIOSim() {
    // Simple simulation without physics
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Upper motor
    inputs.upperAppliedVolts = appliedPercent * 12.0; // Assume 12V max
    inputs.upperVelocityRotPerSec = appliedPercent * 80.0; // Rough simulation

    // Lower motor
    inputs.lowerAppliedVolts = appliedPercent * 12.0;
    inputs.lowerVelocityRotPerSec = appliedPercent * 80.0;
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
