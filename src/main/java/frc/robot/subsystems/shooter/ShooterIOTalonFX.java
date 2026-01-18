package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX motor;

  private final VelocityVoltage velocityControl = new VelocityVoltage(0);
  private final VoltageOut voltageControl = new VoltageOut(0);

  public ShooterIOTalonFX() {
    motor = new TalonFX(ShooterConstants.MOTOR_CAN_ID, ShooterConstants.CAN_BUS);

    motor.getConfigurator().apply(shooterConfiguration());

    motor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityRotPerSec = motor.getVelocity().getValueAsDouble();
    inputs.positionRot = motor.getPosition().getValueAsDouble();
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRotPerSec) {
    motor.setControl(velocityControl.withVelocity(velocityRotPerSec));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageControl.withOutput(volts));
  }

  /**
   * Gets the {@link TalonFXConfiguration} for the shooter motor
   *
   * @return The {@link TalonFXConfiguration} for the shooter motor
   */
  private TalonFXConfiguration shooterConfiguration() {
    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;

    configuration.Slot0 =
        new Slot0Configs()
            .withKS(ShooterConstants.KS)
            .withKV(ShooterConstants.KV)
            .withKA(ShooterConstants.KA)
            .withKP(ShooterConstants.KP)
            .withKI(ShooterConstants.KI)
            .withKD(ShooterConstants.KD);

    configuration.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ShooterConstants.CRUISE_VELOCITY)
            .withMotionMagicAcceleration(ShooterConstants.ACCELERATION)
            .withMotionMagicJerk(ShooterConstants.JERK);

    configuration.CurrentLimits.StatorCurrentLimit = ShooterConstants.STATOR_CURRENT_LIMIT;
    configuration.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT;
    configuration.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.SUPPLY_CURRENT_LOWER_TIME;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

    configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    return configuration;
  }
}
