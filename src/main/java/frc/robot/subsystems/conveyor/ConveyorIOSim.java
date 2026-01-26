package frc.robot.subsystems.conveyor;

public class ConveyorIOSim implements ConveyorIO {

  private double dutyCycle = 0.0;

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    inputs.appliedVolts = dutyCycle * 12.0; // Assume 12V max
    // Simplified simulation - just track duty cycle
    inputs.velocityRotPerSec = dutyCycle * 80; // Rough simulation
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    this.dutyCycle = dutyCycle;
  }

  @Override
  public void stop() {
    this.dutyCycle = 0.0;
  }
}
