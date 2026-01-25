package frc.robot.subsystems.indexer;

public class IndexerIOSim implements IndexerIO {

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
    // Simplified simulation - just track applied voltage
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
