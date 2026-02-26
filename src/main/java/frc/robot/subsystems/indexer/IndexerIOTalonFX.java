package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOTalonFX implements IndexerIO {

  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

  public IndexerIOTalonFX() {
    leaderMotor = new TalonFX(IndexerConstants.LEADER_MOTOR_CAN_ID, IndexerConstants.CAN_BUS);
    followerMotor = new TalonFX(IndexerConstants.FOLLOWER_MOTOR_CAN_ID, IndexerConstants.CAN_BUS);

    // Configure leader motor
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();

    // Motor Inversion and Neutral Mode
    leaderConfig.MotorOutput.Inverted = IndexerConstants.LEADER_INVERTED;
    leaderConfig.MotorOutput.NeutralMode = IndexerConstants.NEUTRAL_MODE;

    // Current Limits
    leaderConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.STATOR_CURRENT_LIMIT;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.SUPPLY_CURRENT_LIMIT;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    leaderMotor.getConfigurator().apply(leaderConfig);

    // Configure follower motor
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();

    // Motor Inversion and Neutral Mode
    followerConfig.MotorOutput.Inverted = IndexerConstants.FOLLOWER_INVERTED;
    followerConfig.MotorOutput.NeutralMode = IndexerConstants.NEUTRAL_MODE;

    // Current Limits
    followerConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.STATOR_CURRENT_LIMIT;
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    followerConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.SUPPLY_CURRENT_LIMIT;
    followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    followerMotor.getConfigurator().apply(followerConfig);

    // Set follower to follow leader with StrictFollower
    followerMotor.setControl(new StrictFollower(IndexerConstants.LEADER_MOTOR_CAN_ID));

    leaderMotor.optimizeBusUtilization();
    followerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.velocityRotPerSec = leaderMotor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = leaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = leaderMotor.getStatorCurrent().getValueAsDouble();
    inputs.temperatureCelsius = leaderMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setPercent(double percent) {
    leaderMotor.setControl(dutyCycleControl.withOutput(percent));
  }

  @Override
  public void stop() {
    leaderMotor.stopMotor();
  }
}
