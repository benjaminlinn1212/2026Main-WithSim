package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.util.ClimbIK;

public class ClimbIOTalonFX implements ClimbIO {

  private final TalonFX rightFrontMotor;
  private final TalonFX rightBackMotor;
  private final TalonFX leftFrontMotor;
  private final TalonFX leftBackMotor;

  private final Servo leftSecondaryHookAngleServo;
  private final Servo rightSecondaryHookAngleServo;
  private final Servo leftSecondaryHookHardstopServo;
  private final Servo rightSecondaryHookHardstopServo;

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

    // Initialize secondary hook angle servos — 180° servo with 500µs–2500µs pulse range.
    leftSecondaryHookAngleServo = new Servo(ClimbConstants.LEFT_ANGLE_SERVO_PWM);
    leftSecondaryHookAngleServo.setBoundsMicroseconds(
        ClimbConstants.AngleServo.PULSE_MAX_US, 0, 0, 0, ClimbConstants.AngleServo.PULSE_MIN_US);
    rightSecondaryHookAngleServo = new Servo(ClimbConstants.RIGHT_ANGLE_SERVO_PWM);
    rightSecondaryHookAngleServo.setBoundsMicroseconds(
        ClimbConstants.AngleServo.PULSE_MAX_US, 0, 0, 0, ClimbConstants.AngleServo.PULSE_MIN_US);

    leftSecondaryHookAngleServo.set(
        applyInversion(
            ClimbConstants.AngleServo.STOWED_POSITION, ClimbConstants.LEFT_ANGLE_SERVO_INVERTED));
    rightSecondaryHookAngleServo.set(
        applyInversion(
            ClimbConstants.AngleServo.STOWED_POSITION, ClimbConstants.RIGHT_ANGLE_SERVO_INVERTED));

    // Initialize secondary hook hardstop servos — 100° servo with 1000µs–2000µs pulse range.
    leftSecondaryHookHardstopServo = new Servo(ClimbConstants.LEFT_HARDSTOP_SERVO_PWM);
    leftSecondaryHookHardstopServo.setBoundsMicroseconds(
        ClimbConstants.HardstopServo.PULSE_MAX_US,
        0,
        0,
        0,
        ClimbConstants.HardstopServo.PULSE_MIN_US);
    rightSecondaryHookHardstopServo = new Servo(ClimbConstants.RIGHT_HARDSTOP_SERVO_PWM);
    rightSecondaryHookHardstopServo.setBoundsMicroseconds(
        ClimbConstants.HardstopServo.PULSE_MAX_US,
        0,
        0,
        0,
        ClimbConstants.HardstopServo.PULSE_MIN_US);

    leftSecondaryHookHardstopServo.set(
        applyInversion(
            ClimbConstants.HardstopServo.STOWED_POSITION,
            ClimbConstants.LEFT_HARDSTOP_SERVO_INVERTED));
    rightSecondaryHookHardstopServo.set(
        applyInversion(
            ClimbConstants.HardstopServo.STOWED_POSITION,
            ClimbConstants.RIGHT_HARDSTOP_SERVO_INVERTED));

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
    // Motion Magic - convert mechanism speeds to motor speeds
    rightFrontConfig.MotionMagic.MotionMagicCruiseVelocity =
        ClimbConstants.CRUISE_VELOCITY / ClimbConstants.FRONT_GEAR_RATIO;
    rightFrontConfig.MotionMagic.MotionMagicAcceleration =
        ClimbConstants.ACCELERATION / ClimbConstants.FRONT_GEAR_RATIO;
    rightFrontConfig.MotionMagic.MotionMagicJerk =
        ClimbConstants.JERK / ClimbConstants.FRONT_GEAR_RATIO;
    rightFrontConfig.CurrentLimits = baseConfig.CurrentLimits;
    // Per CTRE recommendation, set to 1.0 and handle conversions in code
    rightFrontConfig.Feedback.SensorToMechanismRatio = 1.0;

    // Right Back motor config (80:1 gear ratio)
    TalonFXConfiguration rightBackConfig = new TalonFXConfiguration();
    rightBackConfig.MotorOutput.Inverted = ClimbConstants.RIGHT_BACK_MOTOR_INVERTED;
    rightBackConfig.MotorOutput.NeutralMode = baseConfig.MotorOutput.NeutralMode;
    rightBackConfig.Slot0 = baseConfig.Slot0;
    // Motion Magic - convert mechanism speeds to motor speeds
    rightBackConfig.MotionMagic.MotionMagicCruiseVelocity =
        ClimbConstants.CRUISE_VELOCITY / ClimbConstants.BACK_GEAR_RATIO;
    rightBackConfig.MotionMagic.MotionMagicAcceleration =
        ClimbConstants.ACCELERATION / ClimbConstants.BACK_GEAR_RATIO;
    rightBackConfig.MotionMagic.MotionMagicJerk =
        ClimbConstants.JERK / ClimbConstants.BACK_GEAR_RATIO;
    rightBackConfig.CurrentLimits = baseConfig.CurrentLimits;
    // Per CTRE recommendation, set to 1.0 and handle conversions in code
    rightBackConfig.Feedback.SensorToMechanismRatio = 1.0;

    // Left Front motor config (100:1 gear ratio)
    TalonFXConfiguration leftFrontConfig = new TalonFXConfiguration();
    leftFrontConfig.MotorOutput.Inverted = ClimbConstants.LEFT_FRONT_MOTOR_INVERTED;
    leftFrontConfig.MotorOutput.NeutralMode = baseConfig.MotorOutput.NeutralMode;
    leftFrontConfig.Slot0 = baseConfig.Slot0;
    // Motion Magic - convert mechanism speeds to motor speeds
    leftFrontConfig.MotionMagic.MotionMagicCruiseVelocity =
        ClimbConstants.CRUISE_VELOCITY / ClimbConstants.FRONT_GEAR_RATIO;
    leftFrontConfig.MotionMagic.MotionMagicAcceleration =
        ClimbConstants.ACCELERATION / ClimbConstants.FRONT_GEAR_RATIO;
    leftFrontConfig.MotionMagic.MotionMagicJerk =
        ClimbConstants.JERK / ClimbConstants.FRONT_GEAR_RATIO;
    leftFrontConfig.CurrentLimits = baseConfig.CurrentLimits;
    // Per CTRE recommendation, set to 1.0 and handle conversions in code
    leftFrontConfig.Feedback.SensorToMechanismRatio = 1.0;

    // Left Back motor config (80:1 gear ratio)
    TalonFXConfiguration leftBackConfig = new TalonFXConfiguration();
    leftBackConfig.MotorOutput.Inverted = ClimbConstants.LEFT_BACK_MOTOR_INVERTED;
    leftBackConfig.MotorOutput.NeutralMode = baseConfig.MotorOutput.NeutralMode;
    leftBackConfig.Slot0 = baseConfig.Slot0;
    // Motion Magic - convert mechanism speeds to motor speeds
    leftBackConfig.MotionMagic.MotionMagicCruiseVelocity =
        ClimbConstants.CRUISE_VELOCITY / ClimbConstants.BACK_GEAR_RATIO;
    leftBackConfig.MotionMagic.MotionMagicAcceleration =
        ClimbConstants.ACCELERATION / ClimbConstants.BACK_GEAR_RATIO;
    leftBackConfig.MotionMagic.MotionMagicJerk =
        ClimbConstants.JERK / ClimbConstants.BACK_GEAR_RATIO;
    leftBackConfig.CurrentLimits = baseConfig.CurrentLimits;
    // Per CTRE recommendation, set to 1.0 and handle conversions in code
    leftBackConfig.Feedback.SensorToMechanismRatio = 1.0;

    // Apply configs
    rightFrontMotor.getConfigurator().apply(rightFrontConfig);
    rightBackMotor.getConfigurator().apply(rightBackConfig);
    leftFrontMotor.getConfigurator().apply(leftFrontConfig);
    leftBackMotor.getConfigurator().apply(leftBackConfig);

    // Seed encoder positions to match the STOWED cable-length rotations from IK.
    // The robot physically starts in STOWED, so the encoders must reflect the correct
    // mechanism rotations (converted to motor rotations via gear ratio) for the IK/FK
    // system to produce valid arm positions from the first periodic() call.
    Translation2d stowedPos = ClimbState.STOWED.getTargetPosition();
    ClimbIK.ClimbSideIKResult stowedIK = ClimbIK.calculateIK(stowedPos);
    if (stowedIK.isValid) {
      rightFrontMotor.setPosition(stowedIK.frontMotorRotations / ClimbConstants.FRONT_GEAR_RATIO);
      rightBackMotor.setPosition(stowedIK.backMotorRotations / ClimbConstants.BACK_GEAR_RATIO);
      leftFrontMotor.setPosition(stowedIK.frontMotorRotations / ClimbConstants.FRONT_GEAR_RATIO);
      leftBackMotor.setPosition(stowedIK.backMotorRotations / ClimbConstants.BACK_GEAR_RATIO);
    } else {
      // Fallback — shouldn't happen for STOWED
      rightFrontMotor.setPosition(0);
      rightBackMotor.setPosition(0);
      leftFrontMotor.setPosition(0);
      leftBackMotor.setPosition(0);
    }

    rightFrontMotor.optimizeBusUtilization();
    rightBackMotor.optimizeBusUtilization();
    leftFrontMotor.optimizeBusUtilization();
    leftBackMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // Right Front Motor - Convert motor rotations to mechanism rotations
    double rightFrontMotorRot = rightFrontMotor.getPosition().getValueAsDouble();
    inputs.rightFrontPositionRotations = rightFrontMotorRot * ClimbConstants.FRONT_GEAR_RATIO;
    double rightFrontMotorVel = rightFrontMotor.getVelocity().getValueAsDouble();
    inputs.rightFrontVelocityRotPerSec = rightFrontMotorVel * ClimbConstants.FRONT_GEAR_RATIO;
    inputs.rightFrontAppliedVolts = rightFrontMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightFrontCurrentAmps = rightFrontMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightFrontTemperatureCelsius = rightFrontMotor.getDeviceTemp().getValueAsDouble();

    // Right Back Motor - Convert motor rotations to mechanism rotations
    double rightBackMotorRot = rightBackMotor.getPosition().getValueAsDouble();
    inputs.rightBackPositionRotations = rightBackMotorRot * ClimbConstants.BACK_GEAR_RATIO;
    double rightBackMotorVel = rightBackMotor.getVelocity().getValueAsDouble();
    inputs.rightBackVelocityRotPerSec = rightBackMotorVel * ClimbConstants.BACK_GEAR_RATIO;
    inputs.rightBackAppliedVolts = rightBackMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightBackCurrentAmps = rightBackMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightBackTemperatureCelsius = rightBackMotor.getDeviceTemp().getValueAsDouble();

    // Left Front Motor - Convert motor rotations to mechanism rotations
    double leftFrontMotorRot = leftFrontMotor.getPosition().getValueAsDouble();
    inputs.leftFrontPositionRotations = leftFrontMotorRot * ClimbConstants.FRONT_GEAR_RATIO;
    double leftFrontMotorVel = leftFrontMotor.getVelocity().getValueAsDouble();
    inputs.leftFrontVelocityRotPerSec = leftFrontMotorVel * ClimbConstants.FRONT_GEAR_RATIO;
    inputs.leftFrontAppliedVolts = leftFrontMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftFrontCurrentAmps = leftFrontMotor.getStatorCurrent().getValueAsDouble();
    inputs.leftFrontTemperatureCelsius = leftFrontMotor.getDeviceTemp().getValueAsDouble();

    // Left Back Motor - Convert motor rotations to mechanism rotations
    double leftBackMotorRot = leftBackMotor.getPosition().getValueAsDouble();
    inputs.leftBackPositionRotations = leftBackMotorRot * ClimbConstants.BACK_GEAR_RATIO;
    double leftBackMotorVel = leftBackMotor.getVelocity().getValueAsDouble();
    inputs.leftBackVelocityRotPerSec = leftBackMotorVel * ClimbConstants.BACK_GEAR_RATIO;
    inputs.leftBackAppliedVolts = leftBackMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftBackCurrentAmps = leftBackMotor.getStatorCurrent().getValueAsDouble();
    inputs.leftBackTemperatureCelsius = leftBackMotor.getDeviceTemp().getValueAsDouble();

    // Secondary Hook Angle Servos
    inputs.leftSecondaryHookAngleServoPosition = leftSecondaryHookAngleServo.get();
    inputs.rightSecondaryHookAngleServoPosition = rightSecondaryHookAngleServo.get();

    // Secondary Hook Hardstop Servos
    inputs.leftSecondaryHookHardstopServoPosition = leftSecondaryHookHardstopServo.get();
    inputs.rightSecondaryHookHardstopServoPosition = rightSecondaryHookHardstopServo.get();
  }

  @Override
  public void setRightFrontPosition(double positionRotations) {
    // Convert mechanism rotations to motor rotations: motor_rot = mechanism_rot / gear_ratio
    double motorRotations = positionRotations / ClimbConstants.FRONT_GEAR_RATIO;
    rightFrontMotor.setControl(rightFrontPositionControl.withPosition(motorRotations));
  }

  @Override
  public void setRightBackPosition(double positionRotations) {
    // Convert mechanism rotations to motor rotations: motor_rot = mechanism_rot / gear_ratio
    double motorRotations = positionRotations / ClimbConstants.BACK_GEAR_RATIO;
    rightBackMotor.setControl(rightBackPositionControl.withPosition(motorRotations));
  }

  @Override
  public void setLeftFrontPosition(double positionRotations) {
    // Convert mechanism rotations to motor rotations: motor_rot = mechanism_rot / gear_ratio
    double motorRotations = positionRotations / ClimbConstants.FRONT_GEAR_RATIO;
    leftFrontMotor.setControl(leftFrontPositionControl.withPosition(motorRotations));
  }

  @Override
  public void setLeftBackPosition(double positionRotations) {
    // Convert mechanism rotations to motor rotations: motor_rot = mechanism_rot / gear_ratio
    double motorRotations = positionRotations / ClimbConstants.BACK_GEAR_RATIO;
    leftBackMotor.setControl(leftBackPositionControl.withPosition(motorRotations));
  }

  @Override
  public void setRightFrontVelocity(double velocityRotPerSec, double feedforwardVolts) {
    // Convert mechanism velocity to motor velocity: motor_vel = mechanism_vel / gear_ratio
    double motorVelocity = velocityRotPerSec / ClimbConstants.FRONT_GEAR_RATIO;
    rightFrontMotor.setControl(
        rightFrontVelocityControl.withVelocity(motorVelocity).withFeedForward(feedforwardVolts));
  }

  @Override
  public void setRightBackVelocity(double velocityRotPerSec, double feedforwardVolts) {
    // Convert mechanism velocity to motor velocity: motor_vel = mechanism_vel / gear_ratio
    double motorVelocity = velocityRotPerSec / ClimbConstants.BACK_GEAR_RATIO;
    rightBackMotor.setControl(
        rightBackVelocityControl.withVelocity(motorVelocity).withFeedForward(feedforwardVolts));
  }

  @Override
  public void setLeftFrontVelocity(double velocityRotPerSec, double feedforwardVolts) {
    // Convert mechanism velocity to motor velocity: motor_vel = mechanism_vel / gear_ratio
    double motorVelocity = velocityRotPerSec / ClimbConstants.FRONT_GEAR_RATIO;
    leftFrontMotor.setControl(
        leftFrontVelocityControl.withVelocity(motorVelocity).withFeedForward(feedforwardVolts));
  }

  @Override
  public void setLeftBackVelocity(double velocityRotPerSec, double feedforwardVolts) {
    // Convert mechanism velocity to motor velocity: motor_vel = mechanism_vel / gear_ratio
    double motorVelocity = velocityRotPerSec / ClimbConstants.BACK_GEAR_RATIO;
    leftBackMotor.setControl(
        leftBackVelocityControl.withVelocity(motorVelocity).withFeedForward(feedforwardVolts));
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
  public void setLeftSecondaryHookAnglePosition(double position) {
    leftSecondaryHookAngleServo.set(
        applyInversion(position, ClimbConstants.LEFT_ANGLE_SERVO_INVERTED));
  }

  @Override
  public void setRightSecondaryHookAnglePosition(double position) {
    rightSecondaryHookAngleServo.set(
        applyInversion(position, ClimbConstants.RIGHT_ANGLE_SERVO_INVERTED));
  }

  @Override
  public void setLeftSecondaryHookHardstopPosition(double position) {
    leftSecondaryHookHardstopServo.set(
        applyInversion(position, ClimbConstants.LEFT_HARDSTOP_SERVO_INVERTED));
  }

  @Override
  public void setRightSecondaryHookHardstopPosition(double position) {
    rightSecondaryHookHardstopServo.set(
        applyInversion(position, ClimbConstants.RIGHT_HARDSTOP_SERVO_INVERTED));
  }

  /**
   * Apply inversion to a servo position (0.0–1.0). When inverted, 0.0 maps to the servo's max angle
   * and 1.0 maps to 0°, effectively reversing the direction of positive rotation.
   */
  private static double applyInversion(double position, boolean inverted) {
    return inverted ? 1.0 - position : position;
  }

  @Override
  public void stop() {
    rightFrontMotor.stopMotor();
    rightBackMotor.stopMotor();
    leftFrontMotor.stopMotor();
    leftBackMotor.stopMotor();
  }

  @Override
  public void recalibrateEncoders() {
    // Re-seed encoder positions to match STOWED cable-length rotations (same as constructor).
    // This realigns the encoder frame of reference after manual calibration adjustments.
    Translation2d stowedPos = ClimbState.STOWED.getTargetPosition();
    ClimbIK.ClimbSideIKResult stowedIK = ClimbIK.calculateIK(stowedPos);
    if (stowedIK.isValid) {
      rightFrontMotor.setPosition(stowedIK.frontMotorRotations / ClimbConstants.FRONT_GEAR_RATIO);
      rightBackMotor.setPosition(stowedIK.backMotorRotations / ClimbConstants.BACK_GEAR_RATIO);
      leftFrontMotor.setPosition(stowedIK.frontMotorRotations / ClimbConstants.FRONT_GEAR_RATIO);
      leftBackMotor.setPosition(stowedIK.backMotorRotations / ClimbConstants.BACK_GEAR_RATIO);
    }
  }
}
