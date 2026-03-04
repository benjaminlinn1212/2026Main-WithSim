package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX upperMotor;
  private final TalonFX lowerMotor;
  private final VelocityVoltage upperVelocityControl = new VelocityVoltage(0);
  private final VelocityVoltage lowerVelocityControl = new VelocityVoltage(0);

  public IntakeIOTalonFX() {
    upperMotor = new TalonFX(IntakeConstants.UPPER_MOTOR_CAN_ID, IntakeConstants.CAN_BUS);
    lowerMotor = new TalonFX(IntakeConstants.LOWER_MOTOR_CAN_ID, IntakeConstants.CAN_BUS);

    // Upper motor configuration with its own PID
    TalonFXConfiguration upperConfig = new TalonFXConfiguration();
    upperConfig.MotorOutput.Inverted = IntakeConstants.MOTOR_INVERTED;
    upperConfig.MotorOutput.NeutralMode = IntakeConstants.NEUTRAL_MODE;
    upperConfig.Slot0 =
        new Slot0Configs()
            .withKP(IntakeConstants.UPPER_KP)
            .withKI(IntakeConstants.UPPER_KI)
            .withKD(IntakeConstants.UPPER_KD)
            .withKS(IntakeConstants.UPPER_KS)
            .withKV(IntakeConstants.UPPER_KV)
            .withKA(IntakeConstants.UPPER_KA);
    upperConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;
    upperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    upperConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
    upperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Lower motor configuration with its own PID
    TalonFXConfiguration lowerConfig = new TalonFXConfiguration();
    lowerConfig.MotorOutput.Inverted = IntakeConstants.MOTOR_INVERTED;
    lowerConfig.MotorOutput.NeutralMode = IntakeConstants.NEUTRAL_MODE;
    lowerConfig.Slot0 =
        new Slot0Configs()
            .withKP(IntakeConstants.LOWER_KP)
            .withKI(IntakeConstants.LOWER_KI)
            .withKD(IntakeConstants.LOWER_KD)
            .withKS(IntakeConstants.LOWER_KS)
            .withKV(IntakeConstants.LOWER_KV)
            .withKA(IntakeConstants.LOWER_KA);
    lowerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;
    lowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    lowerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
    lowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    upperMotor.getConfigurator().apply(upperConfig);
    lowerMotor.getConfigurator().apply(lowerConfig);

    upperMotor.optimizeBusUtilization();
    lowerMotor.optimizeBusUtilization();
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
  public void setUpperVelocity(double velocityRPS) {
    upperMotor.setControl(upperVelocityControl.withVelocity(velocityRPS));
  }

  @Override
  public void setLowerVelocity(double velocityRPS) {
    lowerMotor.setControl(lowerVelocityControl.withVelocity(velocityRPS));
  }

  @Override
  public void stop() {
    upperMotor.stopMotor();
    lowerMotor.stopMotor();
  }
}
