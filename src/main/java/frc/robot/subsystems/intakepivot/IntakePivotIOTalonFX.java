package frc.robot.subsystems.intakepivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants.IntakePivotConstants;

public class IntakePivotIOTalonFX implements IntakePivotIO {

  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;
  private final MotionMagicDutyCycle positionControl = new MotionMagicDutyCycle(0);

  public IntakePivotIOTalonFX() {
    leaderMotor =
        new TalonFX(IntakePivotConstants.LEADER_MOTOR_CAN_ID, IntakePivotConstants.CAN_BUS);
    followerMotor =
        new TalonFX(IntakePivotConstants.FOLLOWER_MOTOR_CAN_ID, IntakePivotConstants.CAN_BUS);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // PID configuration
    config.Slot0.kP = IntakePivotConstants.KP;
    config.Slot0.kI = IntakePivotConstants.KI;
    config.Slot0.kD = IntakePivotConstants.KD;
    config.Slot0.kS = IntakePivotConstants.KS;
    config.Slot0.kV = IntakePivotConstants.KV;
    config.Slot0.kA = IntakePivotConstants.KA;
    config.Slot0.kG = IntakePivotConstants.KG; // Gravity feedforward

    // Motion Magic configuration
    config.MotionMagic.MotionMagicCruiseVelocity = IntakePivotConstants.CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = IntakePivotConstants.ACCELERATION;
    config.MotionMagic.MotionMagicJerk = IntakePivotConstants.JERK;

    // Soft limits configuration
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakePivotConstants.SOFT_LIMIT_FORWARD;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakePivotConstants.SOFT_LIMIT_REVERSE;

    // Motor inversion for leader
    config.MotorOutput.Inverted =
        IntakePivotConstants.LEADER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Current limits
    config.CurrentLimits.StatorCurrentLimit = IntakePivotConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = IntakePivotConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = IntakePivotConstants.NEUTRAL_MODE;

    // Set motor encoder offset (adjust the motor's internal sensor reading)
    config.Feedback.FeedbackRotorOffset = IntakePivotConstants.MOTOR_ROTOR_OFFSET;

    leaderMotor.getConfigurator().apply(config);

    // Configure follower motor (opposite direction)
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.CurrentLimits.StatorCurrentLimit = IntakePivotConstants.STATOR_CURRENT_LIMIT;
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    followerConfig.CurrentLimits.SupplyCurrentLimit = IntakePivotConstants.SUPPLY_CURRENT_LIMIT;
    followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    followerConfig.MotorOutput.NeutralMode = IntakePivotConstants.NEUTRAL_MODE;
    // Set follower motor inversion from constants
    followerConfig.MotorOutput.Inverted =
        IntakePivotConstants.FOLLOWER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    followerMotor.getConfigurator().apply(followerConfig);

    // Set follower to follow leader
    followerMotor.setControl(new StrictFollower(IntakePivotConstants.LEADER_MOTOR_CAN_ID));
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.positionRotations = leaderMotor.getPosition().getValueAsDouble();
    inputs.velocityRotPerSec = leaderMotor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = leaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = leaderMotor.getStatorCurrent().getValueAsDouble();
    inputs.temperatureCelsius = leaderMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setPosition(double positionRotations) {
    leaderMotor.setControl(positionControl.withPosition(positionRotations));
  }

  @Override
  public void setVoltage(double volts) {
    leaderMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    leaderMotor.stopMotor();
    followerMotor.stopMotor();
  }
}
