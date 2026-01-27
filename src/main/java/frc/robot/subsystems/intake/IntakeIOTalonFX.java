package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX upperMotor;
  private final TalonFX lowerMotor;
  private final DutyCycleOut upperDutyCycleControl = new DutyCycleOut(0);
  private final DutyCycleOut lowerDutyCycleControl = new DutyCycleOut(0);

  public IntakeIOTalonFX() {
    upperMotor = new TalonFX(IntakeConstants.UPPER_MOTOR_CAN_ID, IntakeConstants.CAN_BUS);
    lowerMotor = new TalonFX(IntakeConstants.LOWER_MOTOR_CAN_ID, IntakeConstants.CAN_BUS);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = IntakeConstants.NEUTRAL_MODE;

    upperMotor.getConfigurator().apply(config);
    lowerMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.upperVelocityRotPerSec = upperMotor.getVelocity().getValueAsDouble();
    inputs.upperAppliedVolts = upperMotor.getMotorVoltage().getValueAsDouble();
    inputs.upperCurrentAmps = upperMotor.getStatorCurrent().getValueAsDouble();
    inputs.upperTemperatureCelsius = upperMotor.getDeviceTemp().getValueAsDouble();

    inputs.lowerVelocityRotPerSec = lowerMotor.getVelocity().getValueAsDouble();
    inputs.lowerAppliedVolts = lowerMotor.getMotorVoltage().getValueAsDouble();
    inputs.lowerCurrentAmps = lowerMotor.getStatorCurrent().getValueAsDouble();
    inputs.lowerTemperatureCelsius = lowerMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setUpperPercent(double percent) {
    upperMotor.setControl(upperDutyCycleControl.withOutput(percent));
  }

  @Override
  public void setLowerPercent(double percent) {
    lowerMotor.setControl(lowerDutyCycleControl.withOutput(percent));
  }

  @Override
  public void setPercent(double percent) {
    setUpperPercent(percent);
    setLowerPercent(percent);
  }

  @Override
  public void stop() {
    upperMotor.stopMotor();
    lowerMotor.stopMotor();
  }
}
