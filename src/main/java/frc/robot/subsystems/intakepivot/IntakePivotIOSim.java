package frc.robot.subsystems.intakepivot;

public class IntakePivotIOSim implements IntakePivotIO {

  private double positionRotations = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    // Both motors report same values in sim
    inputs.rightPositionRotations = positionRotations;
    inputs.rightAppliedVolts = appliedVolts;
    inputs.rightVelocityRotPerSec = appliedVolts * 5; // Simplified simulation
    inputs.rightCurrentAmps = Math.abs(appliedVolts) * 2.0;
    inputs.rightTemperatureCelsius = 25.0;

    inputs.leftPositionRotations = positionRotations;
    inputs.leftAppliedVolts = appliedVolts;
    inputs.leftVelocityRotPerSec = appliedVolts * 5; // Simplified simulation
    inputs.leftCurrentAmps = Math.abs(appliedVolts) * 2.0;
    inputs.leftTemperatureCelsius = 25.0;
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
