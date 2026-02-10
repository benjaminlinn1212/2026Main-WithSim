package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Hardware implementation of HoodIO using TalonFX motor controller. */
public class HoodIOTalonFX implements HoodIO {
  private final TalonFX motor;

  private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withSlot(0);
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

  public HoodIOTalonFX() {
    motor = new TalonFX(Constants.HoodConstants.MOTOR_CAN_ID, Constants.HoodConstants.CAN_BUS);

    // Configure TalonFX
    var motorConfig = new TalonFXConfiguration();

    // Motor Inversion and Neutral Mode
    motorConfig.MotorOutput.Inverted = Constants.HoodConstants.MOTOR_INVERTED;
    motorConfig.MotorOutput.NeutralMode = Constants.HoodConstants.NEUTRAL_MODE;

    // Feedback Configuration
    motorConfig.Feedback.RotorToSensorRatio = 1.0;
    // Per CTRE recommendation, set SensorToMechanismRatio to 1.0 and handle conversions in code
    motorConfig.Feedback.SensorToMechanismRatio = 1.0;
    motorConfig.Feedback.FeedbackRotorOffset = Constants.HoodConstants.ROTOR_OFFSET;

    // Software Limits (motor rotations - need to convert from mechanism limits)
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // Convert mechanism angle limits to motor rotations: motor_rot = mechanism_rot / gear_ratio
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(
                Constants.HoodConstants.MAX_POSITION_RAD
                    - Units.degreesToRadians(Constants.HoodConstants.MECHANISM_ZERO_ANGLE_DEG))
            / Constants.HoodConstants.GEAR_RATIO;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.radiansToRotations(
                Constants.HoodConstants.MIN_POSITION_RAD
                    - Units.degreesToRadians(Constants.HoodConstants.MECHANISM_ZERO_ANGLE_DEG))
            / Constants.HoodConstants.GEAR_RATIO;

    // PID and Feedforward (order: KP, KI, KD, KS, KV, KA, KG)
    motorConfig.Slot0.kP = Constants.HoodConstants.KP;
    motorConfig.Slot0.kI = Constants.HoodConstants.KI;
    motorConfig.Slot0.kD = Constants.HoodConstants.KD;
    motorConfig.Slot0.kS = Constants.HoodConstants.KS;
    motorConfig.Slot0.kV = Constants.HoodConstants.KV;
    motorConfig.Slot0.kA = Constants.HoodConstants.KA;
    motorConfig.Slot0.kG = Constants.HoodConstants.KG;
    motorConfig.Slot0.GravityType = Constants.HoodConstants.GRAVITY_TYPE;

    // Motion Magic
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.HoodConstants.CRUISE_VELOCITY;
    motorConfig.MotionMagic.MotionMagicAcceleration = Constants.HoodConstants.ACCELERATION;
    motorConfig.MotionMagic.MotionMagicJerk = Constants.HoodConstants.JERK;

    // Current Limits
    motorConfig.CurrentLimits.StatorCurrentLimit = Constants.HoodConstants.STATOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.HoodConstants.SUPPLY_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Factory default first to clear any cached config, then apply new config
    motor.getConfigurator().apply(new TalonFXConfiguration());
    motor.getConfigurator().apply(motorConfig);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Read motor position in motor rotations
    double motorPositionRot = motor.getPosition().getValueAsDouble();

    // Convert motor rotations to mechanism rotations: mechanism_rot = motor_rot * gear_ratio
    double mechanismPositionRot = motorPositionRot * Constants.HoodConstants.GEAR_RATIO;

    // Add the zero offset to get the actual mechanism angle from horizontal
    inputs.positionRad =
        Units.rotationsToRadians(mechanismPositionRot)
            + Units.degreesToRadians(Constants.HoodConstants.MECHANISM_ZERO_ANGLE_DEG);

    // Velocity: motor rotations/sec to mechanism rotations/sec
    double motorVelocityRotPerSec = motor.getVelocity().getValueAsDouble();
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(motorVelocityRotPerSec * Constants.HoodConstants.GEAR_RATIO);

    // Read electrical data
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentStatorAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.currentSupplyAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();

    // Log mechanism position for debugging
    inputs.mechanismPositionBeforeOffsetRot = mechanismPositionRot;
  }

  @Override
  public void setPositionSetpoint(double radiansFromHorizontal, double radPerSecond) {
    // Clamp setpoint to limits
    double clampedRadians =
        MathUtil.clamp(
            radiansFromHorizontal,
            Constants.HoodConstants.MIN_POSITION_RAD,
            Constants.HoodConstants.MAX_POSITION_RAD);

    // Subtract the zero offset to get the mechanism angle from its zero
    // (commanded angle from horizontal - zero offset angle = mechanism angle from zero)
    double mechanismAngleRad =
        clampedRadians - Units.degreesToRadians(Constants.HoodConstants.MECHANISM_ZERO_ANGLE_DEG);

    // Convert mechanism rotations to motor rotations: motor_rot = mechanism_rot / gear_ratio
    double mechanismRotations = Units.radiansToRotations(mechanismAngleRad);
    double motorRotations = mechanismRotations / Constants.HoodConstants.GEAR_RATIO;

    // MotionMagic uses configured cruise velocity/acceleration, feedforward is ignored
    motor.setControl(positionControl.withPosition(motorRotations));
  }

  @Override
  public void setOpenLoopDutyCycle(double dutyCycle) {
    motor.setControl(dutyCycleControl.withOutput(dutyCycle));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
