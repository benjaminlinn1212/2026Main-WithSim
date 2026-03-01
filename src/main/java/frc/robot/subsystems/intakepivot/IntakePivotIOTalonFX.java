package frc.robot.subsystems.intakepivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.IntakePivotConstants;

public class IntakePivotIOTalonFX implements IntakePivotIO {

  private final TalonFX motor;
  private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);

  public IntakePivotIOTalonFX() {
    motor = new TalonFX(IntakePivotConstants.MOTOR_CAN_ID, IntakePivotConstants.CAN_BUS);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = IntakePivotConstants.MOTOR_INVERTED;
    config.MotorOutput.NeutralMode = IntakePivotConstants.NEUTRAL_MODE;

    config.Feedback.SensorToMechanismRatio = 1.0;
    config.Feedback.FeedbackRotorOffset = IntakePivotConstants.MOTOR_ROTOR_OFFSET;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakePivotConstants.SOFT_LIMIT_FORWARD;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakePivotConstants.SOFT_LIMIT_REVERSE;

    config.Slot0.kP = IntakePivotConstants.KP;
    config.Slot0.kI = IntakePivotConstants.KI;
    config.Slot0.kD = IntakePivotConstants.KD;
    config.Slot0.kS = IntakePivotConstants.KS;
    config.Slot0.kV = IntakePivotConstants.KV;
    config.Slot0.kA = IntakePivotConstants.KA;

    config.MotionMagic.MotionMagicCruiseVelocity = IntakePivotConstants.CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = IntakePivotConstants.ACCELERATION;
    config.MotionMagic.MotionMagicJerk = IntakePivotConstants.JERK;

    config.CurrentLimits.StatorCurrentLimit = IntakePivotConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = IntakePivotConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.positionRotations = motor.getPosition().getValueAsDouble();
    inputs.velocityRotPerSec = motor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setPosition(double positionRotations) {
    motor.setControl(positionControl.withPosition(positionRotations));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
