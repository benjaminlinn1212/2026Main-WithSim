package frc.robot.subsystems.intakepivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants.IntakePivotConstants;

public class IntakePivotIOTalonFX implements IntakePivotIO {

  private final TalonFX rightPivotMotor;
  private final TalonFX leftPivotMotor;
  private final MotionMagicDutyCycle rightPositionControl = new MotionMagicDutyCycle(0);
  private final MotionMagicDutyCycle leftPositionControl = new MotionMagicDutyCycle(0);

  public IntakePivotIOTalonFX() {
    rightPivotMotor =
        new TalonFX(IntakePivotConstants.RIGHT_MOTOR_CAN_ID, IntakePivotConstants.CAN_BUS);
    leftPivotMotor =
        new TalonFX(IntakePivotConstants.LEFT_MOTOR_CAN_ID, IntakePivotConstants.CAN_BUS);

    // Create base configuration shared by both motors
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

    // Current limits
    config.CurrentLimits.StatorCurrentLimit = IntakePivotConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = IntakePivotConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = IntakePivotConstants.NEUTRAL_MODE;

    // Configure right motor with right-specific settings
    config.Feedback.FeedbackRotorOffset = IntakePivotConstants.RIGHT_MOTOR_ROTOR_OFFSET;
    config.MotorOutput.Inverted =
        IntakePivotConstants.RIGHT_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    rightPivotMotor.getConfigurator().apply(config);

    // Configure left motor with left-specific settings
    config.Feedback.FeedbackRotorOffset = IntakePivotConstants.LEFT_MOTOR_ROTOR_OFFSET;
    config.MotorOutput.Inverted =
        IntakePivotConstants.LEFT_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    leftPivotMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.rightPositionRotations = rightPivotMotor.getPosition().getValueAsDouble();
    inputs.rightVelocityRotPerSec = rightPivotMotor.getVelocity().getValueAsDouble();
    inputs.rightAppliedVolts = rightPivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightCurrentAmps = rightPivotMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightTemperatureCelsius = rightPivotMotor.getDeviceTemp().getValueAsDouble();

    inputs.leftPositionRotations = leftPivotMotor.getPosition().getValueAsDouble();
    inputs.leftVelocityRotPerSec = leftPivotMotor.getVelocity().getValueAsDouble();
    inputs.leftAppliedVolts = leftPivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftCurrentAmps = leftPivotMotor.getStatorCurrent().getValueAsDouble();
    inputs.leftTemperatureCelsius = leftPivotMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setPosition(double positionRotations) {
    rightPivotMotor.setControl(rightPositionControl.withPosition(positionRotations));
    leftPivotMotor.setControl(leftPositionControl.withPosition(positionRotations));
  }

  @Override
  public void setVoltage(double volts) {
    rightPivotMotor.setVoltage(volts);
    leftPivotMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    rightPivotMotor.stopMotor();
    leftPivotMotor.stopMotor();
  }
}
