package frc.robot.subsystems.conveyor;

public class ConveyorIOSim implements ConveyorIO {

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
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
