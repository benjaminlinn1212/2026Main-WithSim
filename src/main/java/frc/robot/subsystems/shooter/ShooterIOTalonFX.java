package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;

  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
  private final VoltageOut voltageControl = new VoltageOut(0);

  public ShooterIOTalonFX() {
    leaderMotor = new TalonFX(ShooterConstants.LEADER_MOTOR_CAN_ID, ShooterConstants.CAN_BUS);
    followerMotor = new TalonFX(ShooterConstants.FOLLOWER_MOTOR_CAN_ID, ShooterConstants.CAN_BUS);

    leaderMotor.getConfigurator().apply(shooterConfiguration());
    leaderMotor.setNeutralMode(ShooterConstants.NEUTRAL_MODE);

    // Configure follower motor
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.STATOR_CURRENT_LIMIT;
    followerConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT;
    followerConfig.CurrentLimits.SupplyCurrentLowerTime =
        ShooterConstants.SUPPLY_CURRENT_LOWER_TIME;
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    followerConfig.MotorOutput.NeutralMode = ShooterConstants.NEUTRAL_MODE;
    // Set follower motor inversion from constants
    followerConfig.MotorOutput.Inverted =
        ShooterConstants.FOLLOWER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    followerMotor.getConfigurator().apply(followerConfig);

    // Set follower to follow leader
    followerMotor.setControl(new StrictFollower(ShooterConstants.LEADER_MOTOR_CAN_ID));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityRotPerSec = leaderMotor.getVelocity().getValueAsDouble();
    inputs.positionRot = leaderMotor.getPosition().getValueAsDouble();
    inputs.appliedVolts = leaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = leaderMotor.getStatorCurrent().getValueAsDouble();
    inputs.tempCelsius = leaderMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    leaderMotor.setControl(dutyCycleControl.withOutput(dutyCycle));
  }

  @Override
  public void stop() {
    leaderMotor.stopMotor();
    followerMotor.stopMotor();
  }

  @Override
  public void setVoltage(double volts) {
    leaderMotor.setControl(voltageControl.withOutput(volts));
  }

  /**
   * Gets the {@link TalonFXConfiguration} for the shooter motor
   *
   * @return The {@link TalonFXConfiguration} for the shooter motor
   */
  private TalonFXConfiguration shooterConfiguration() {
    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.CurrentLimits.StatorCurrentLimit = ShooterConstants.STATOR_CURRENT_LIMIT;
    configuration.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT;
    configuration.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.SUPPLY_CURRENT_LOWER_TIME;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

    configuration.MotorOutput.Inverted =
        ShooterConstants.LEADER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    return configuration;
  }
}
