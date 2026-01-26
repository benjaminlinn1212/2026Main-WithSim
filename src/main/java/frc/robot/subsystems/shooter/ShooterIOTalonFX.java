package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;

  private final VelocityDutyCycle velocityControl = new VelocityDutyCycle(0);
  private final VoltageOut voltageControl = new VoltageOut(0);

  public ShooterIOTalonFX() {
    leaderMotor = new TalonFX(ShooterConstants.LEADER_MOTOR_CAN_ID, ShooterConstants.CAN_BUS);
    followerMotor = new TalonFX(ShooterConstants.FOLLOWER_MOTOR_CAN_ID, ShooterConstants.CAN_BUS);

    leaderMotor.getConfigurator().apply(shooterConfiguration());
    leaderMotor.setNeutralMode(NeutralModeValue.Coast);

    // Configure follower motor
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.STATOR_CURRENT_LIMIT;
    followerConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT;
    followerConfig.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.SUPPLY_CURRENT_LOWER_TIME;
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Set follower motor to opposite direction
    followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    followerMotor.getConfigurator().apply(followerConfig);

    // Set follower to follow leader with opposite direction
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
  public void setVelocity(double velocityRotPerSec) {
    leaderMotor.setControl(velocityControl.withVelocity(velocityRotPerSec));
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
