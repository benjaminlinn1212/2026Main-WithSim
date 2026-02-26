package frc.robot.subsystems.indexer;

public class IndexerIOSim implements IndexerIO {

  private double appliedDutyCycle = 0.0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Simplified simulation - duty cycle to voltage approximation
    inputs.appliedVolts = appliedDutyCycle * 12.0; // Assume 12V supply
    inputs.velocityRotPerSec = appliedDutyCycle * 120.0; // Rough simulation
    inputs.currentAmps = Math.abs(appliedDutyCycle) * 10.0;
    inputs.temperatureCelsius = 25.0;
  }

  @Override
  public void setPercent(double percent) {
    this.appliedDutyCycle = percent;
  }

  @Override
  public void stop() {
    this.appliedDutyCycle = 0.0;
  }
}
