package frc.robot.subsystems.intake;

/** Simulation implementation of IntakeIO. This provides basic intake simulation without physics. */
public class IntakeIOSim implements IntakeIO {

  private double upperAppliedPercent = 0.0;
  private double lowerAppliedPercent = 0.0;

  public IntakeIOSim() {
    // Simple simulation without physics
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Upper motor
    inputs.upperAppliedVolts = upperAppliedPercent * 12.0; // Assume 12V max
    inputs.upperVelocityRotPerSec = upperAppliedPercent * 80.0; // Rough simulation
    inputs.upperCurrentAmps = Math.abs(upperAppliedPercent) * 10.0;
    inputs.upperTemperatureCelsius = 25.0;

    // Lower motor
    inputs.lowerAppliedVolts = lowerAppliedPercent * 12.0;
    inputs.lowerVelocityRotPerSec = lowerAppliedPercent * 80.0;
    inputs.lowerCurrentAmps = Math.abs(lowerAppliedPercent) * 10.0;
    inputs.lowerTemperatureCelsius = 25.0;
  }

  @Override
  public void setUpperPercent(double percent) {
    this.upperAppliedPercent = percent;
  }

  @Override
  public void setLowerPercent(double percent) {
    this.lowerAppliedPercent = percent;
  }

  @Override
  public void setPercent(double percent) {
    setUpperPercent(percent);
    setLowerPercent(percent);
  }

  @Override
  public void stop() {
    this.upperAppliedPercent = 0.0;
    this.lowerAppliedPercent = 0.0;
  }
}
