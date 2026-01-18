package frc.robot.subsystems.intakepivot;

public class IntakePivotIOSim implements IntakePivotIO {

  private double positionRotations = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.positionRotations = positionRotations;
    inputs.appliedVolts = appliedVolts;
    inputs.velocityRotPerSec = appliedVolts * 5; // Simplified simulation
  }

  @Override
  public void setPosition(double positionRotations) {
    this.positionRotations = positionRotations;
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
