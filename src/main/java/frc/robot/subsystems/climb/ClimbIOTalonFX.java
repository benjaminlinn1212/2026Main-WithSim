package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOTalonFX implements ClimbIO {

  private final TalonFX rightFrontMotor;
  private final TalonFX rightBackMotor;
  private final TalonFX leftFrontMotor;
  private final TalonFX leftBackMotor;

  private final Servo leftHookServo;
  private final Servo rightHookServo;

  private final MotionMagicVoltage rightFrontPositionControl = new MotionMagicVoltage(0);
  private final MotionMagicVoltage rightBackPositionControl = new MotionMagicVoltage(0);
  private final MotionMagicVoltage leftFrontPositionControl = new MotionMagicVoltage(0);
  private final MotionMagicVoltage leftBackPositionControl = new MotionMagicVoltage(0);

  private final VelocityVoltage rightFrontVelocityControl = new VelocityVoltage(0);
  private final VelocityVoltage rightBackVelocityControl = new VelocityVoltage(0);
  private final VelocityVoltage leftFrontVelocityControl = new VelocityVoltage(0);
  private final VelocityVoltage leftBackVelocityControl = new VelocityVoltage(0);

  public ClimbIOTalonFX() {
    rightFrontMotor = new TalonFX(ClimbConstants.RIGHT_FRONT_MOTOR_CAN_ID, ClimbConstants.CAN_BUS);
    rightBackMotor = new TalonFX(ClimbConstants.RIGHT_BACK_MOTOR_CAN_ID, ClimbConstants.CAN_BUS);
    leftFrontMotor = new TalonFX(ClimbConstants.LEFT_FRONT_MOTOR_CAN_ID, ClimbConstants.CAN_BUS);
    leftBackMotor = new TalonFX(ClimbConstants.LEFT_BACK_MOTOR_CAN_ID, ClimbConstants.CAN_BUS);

    // Initialize passive hook release servos
    leftHookServo = new Servo(ClimbConstants.LEFT_HOOK_SERVO_PWM);
    rightHookServo = new Servo(ClimbConstants.RIGHT_HOOK_SERVO_PWM);

    // Start with hooks in stowed (locked) position
    leftHookServo.set(ClimbConstants.HOOK_STOWED_POSITION);
    rightHookServo.set(ClimbConstants.HOOK_STOWED_POSITION);

    // Base configuration (shared by all motors)
    TalonFXConfiguration baseConfig = new TalonFXConfiguration();

    // Neutral Mode (same for all)
    baseConfig.MotorOutput.NeutralMode = ClimbConstants.NEUTRAL_MODE;

    // PID and Feedforward (order: KP, KI, KD, KS, KV, KA, KG)
    baseConfig.Slot0.kP = ClimbConstants.KP;
    baseConfig.Slot0.kI = ClimbConstants.KI;
    baseConfig.Slot0.kD = ClimbConstants.KD;
    baseConfig.Slot0.kS = ClimbConstants.KS;
    baseConfig.Slot0.kV = ClimbConstants.KV;
    baseConfig.Slot0.kA = ClimbConstants.KA;
    baseConfig.Slot0.kG = ClimbConstants.KG;

    // Motion Magic
    baseConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.CRUISE_VELOCITY;
    baseConfig.MotionMagic.MotionMagicAcceleration = ClimbConstants.ACCELERATION;
    baseConfig.MotionMagic.MotionMagicJerk = ClimbConstants.JERK;

    // Current Limits
    baseConfig.CurrentLimits.StatorCurrentLimit = ClimbConstants.STATOR_CURRENT_LIMIT;
    baseConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    baseConfig.CurrentLimits.SupplyCurrentLimit = ClimbConstants.SUPPLY_CURRENT_LIMIT;
    baseConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Right Front motor config (100:1 gear ratio)
    TalonFXConfiguration rightFrontConfig = new TalonFXConfiguration();
    rightFrontConfig.MotorOutput.Inverted = ClimbConstants.RIGHT_FRONT_MOTOR_INVERTED;
    rightFrontConfig.MotorOutput.NeutralMode = baseConfig.MotorOutput.NeutralMode;
    rightFrontConfig.Slot0 = baseConfig.Slot0;
    rightFrontConfig.MotionMagic = baseConfig.MotionMagic;
    rightFrontConfig.CurrentLimits = baseConfig.CurrentLimits;
    rightFrontConfig.Feedback.SensorToMechanismRatio = ClimbConstants.FRONT_GEAR_RATIO;

    // Right Back motor config (80:1 gear ratio)
    TalonFXConfiguration rightBackConfig = new TalonFXConfiguration();
    rightBackConfig.MotorOutput.Inverted = ClimbConstants.RIGHT_BACK_MOTOR_INVERTED;
    rightBackConfig.MotorOutput.NeutralMode = baseConfig.MotorOutput.NeutralMode;
    rightBackConfig.Slot0 = baseConfig.Slot0;
    rightBackConfig.MotionMagic = baseConfig.MotionMagic;
    rightBackConfig.CurrentLimits = baseConfig.CurrentLimits;
    rightBackConfig.Feedback.SensorToMechanismRatio = ClimbConstants.BACK_GEAR_RATIO;

    // Left Front motor config (100:1 gear ratio)
    TalonFXConfiguration leftFrontConfig = new TalonFXConfiguration();
    leftFrontConfig.MotorOutput.Inverted = ClimbConstants.LEFT_FRONT_MOTOR_INVERTED;
    leftFrontConfig.MotorOutput.NeutralMode = baseConfig.MotorOutput.NeutralMode;
    leftFrontConfig.Slot0 = baseConfig.Slot0;
    leftFrontConfig.MotionMagic = baseConfig.MotionMagic;
    leftFrontConfig.CurrentLimits = baseConfig.CurrentLimits;
    leftFrontConfig.Feedback.SensorToMechanismRatio = ClimbConstants.FRONT_GEAR_RATIO;

    // Left Back motor config (80:1 gear ratio)
    TalonFXConfiguration leftBackConfig = new TalonFXConfiguration();
    leftBackConfig.MotorOutput.Inverted = ClimbConstants.LEFT_BACK_MOTOR_INVERTED;
    leftBackConfig.MotorOutput.NeutralMode = baseConfig.MotorOutput.NeutralMode;
    leftBackConfig.Slot0 = baseConfig.Slot0;
    leftBackConfig.MotionMagic = baseConfig.MotionMagic;
    leftBackConfig.CurrentLimits = baseConfig.CurrentLimits;
    leftBackConfig.Feedback.SensorToMechanismRatio = ClimbConstants.BACK_GEAR_RATIO;

    // Apply configs
    rightFrontMotor.getConfigurator().apply(rightFrontConfig);
    rightBackMotor.getConfigurator().apply(rightBackConfig);
    leftFrontMotor.getConfigurator().apply(leftFrontConfig);
    leftBackMotor.getConfigurator().apply(leftBackConfig);

    // Reset positions on startup
    rightFrontMotor.setPosition(0);
    rightBackMotor.setPosition(0);
    leftFrontMotor.setPosition(0);
    leftBackMotor.setPosition(0);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // Right Front Motor
    inputs.rightFrontPositionRotations = rightFrontMotor.getPosition().getValueAsDouble();
    inputs.rightFrontVelocityRotPerSec = rightFrontMotor.getVelocity().getValueAsDouble();
    inputs.rightFrontAppliedVolts = rightFrontMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightFrontCurrentAmps = rightFrontMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightFrontTemperatureCelsius = rightFrontMotor.getDeviceTemp().getValueAsDouble();

    // Right Back Motor
    inputs.rightBackPositionRotations = rightBackMotor.getPosition().getValueAsDouble();
    inputs.rightBackVelocityRotPerSec = rightBackMotor.getVelocity().getValueAsDouble();
    inputs.rightBackAppliedVolts = rightBackMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightBackCurrentAmps = rightBackMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightBackTemperatureCelsius = rightBackMotor.getDeviceTemp().getValueAsDouble();

    // Left Front Motor
    inputs.leftFrontPositionRotations = leftFrontMotor.getPosition().getValueAsDouble();
    inputs.leftFrontVelocityRotPerSec = leftFrontMotor.getVelocity().getValueAsDouble();
    inputs.leftFrontAppliedVolts = leftFrontMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftFrontCurrentAmps = leftFrontMotor.getStatorCurrent().getValueAsDouble();
    inputs.leftFrontTemperatureCelsius = leftFrontMotor.getDeviceTemp().getValueAsDouble();

    // Left Back Motor
    inputs.leftBackPositionRotations = leftBackMotor.getPosition().getValueAsDouble();
    inputs.leftBackVelocityRotPerSec = leftBackMotor.getVelocity().getValueAsDouble();
    inputs.leftBackAppliedVolts = leftBackMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftBackCurrentAmps = leftBackMotor.getStatorCurrent().getValueAsDouble();
    inputs.leftBackTemperatureCelsius = leftBackMotor.getDeviceTemp().getValueAsDouble();

    // Passive Hook Servos
    inputs.leftHookServoPosition = leftHookServo.get();
    inputs.rightHookServoPosition = rightHookServo.get();
  }

  @Override
  public void setRightFrontPosition(double positionRotations) {
    rightFrontMotor.setControl(rightFrontPositionControl.withPosition(positionRotations));
  }

  @Override
  public void setRightBackPosition(double positionRotations) {
    rightBackMotor.setControl(rightBackPositionControl.withPosition(positionRotations));
  }

  @Override
  public void setLeftFrontPosition(double positionRotations) {
    leftFrontMotor.setControl(leftFrontPositionControl.withPosition(positionRotations));
  }

  @Override
  public void setLeftBackPosition(double positionRotations) {
    leftBackMotor.setControl(leftBackPositionControl.withPosition(positionRotations));
  }

  @Override
  public void setRightFrontVelocity(double velocityRotPerSec, double feedforwardVolts) {
    rightFrontMotor.setControl(
        rightFrontVelocityControl
            .withVelocity(velocityRotPerSec)
            .withFeedForward(feedforwardVolts));
  }

  @Override
  public void setRightBackVelocity(double velocityRotPerSec, double feedforwardVolts) {
    rightBackMotor.setControl(
        rightBackVelocityControl.withVelocity(velocityRotPerSec).withFeedForward(feedforwardVolts));
  }

  @Override
  public void setLeftFrontVelocity(double velocityRotPerSec, double feedforwardVolts) {
    leftFrontMotor.setControl(
        leftFrontVelocityControl.withVelocity(velocityRotPerSec).withFeedForward(feedforwardVolts));
  }

  @Override
  public void setLeftBackVelocity(double velocityRotPerSec, double feedforwardVolts) {
    leftBackMotor.setControl(
        leftBackVelocityControl.withVelocity(velocityRotPerSec).withFeedForward(feedforwardVolts));
  }

  @Override
  public void setRightFrontVoltage(double volts) {
    rightFrontMotor.setVoltage(volts);
  }

  @Override
  public void setRightBackVoltage(double volts) {
    rightBackMotor.setVoltage(volts);
  }

  @Override
  public void setLeftFrontVoltage(double volts) {
    leftFrontMotor.setVoltage(volts);
  }

  @Override
  public void setLeftBackVoltage(double volts) {
    leftBackMotor.setVoltage(volts);
  }

  @Override
  public void setLeftHookPosition(double position) {
    leftHookServo.set(position);
  }

  @Override
  public void setRightHookPosition(double position) {
    rightHookServo.set(position);
  }

  @Override
  public void stop() {
    rightFrontMotor.stopMotor();
    rightBackMotor.stopMotor();
    leftFrontMotor.stopMotor();
    leftBackMotor.stopMotor();
  }
}
