package frc.robot.subsystems.conveyor;

public class ConveyorIOSim implements ConveyorIO {

  private double appliedPercent = 0.0;
  private boolean directionInverted = false;

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    inputs.appliedVolts = appliedPercent * 12.0; // Simulate voltage from percent
    inputs.directionInverted = directionInverted;
    // Simplified simulation - just track applied voltage
    inputs.velocityRotPerSec = appliedPercent * 10; // Rough simulation
  }

  @Override
  public void setPercent(double percent) {
    this.appliedPercent = directionInverted ? -percent : percent;
  }

  @Override
  public void stop() {
    this.appliedPercent = 0.0;
  }

  @Override
  public void toggleDirection() {
    directionInverted = !directionInverted;
  }

  @Override
  public void setDirectionInverted(boolean inverted) {
    directionInverted = inverted;
  }
}
