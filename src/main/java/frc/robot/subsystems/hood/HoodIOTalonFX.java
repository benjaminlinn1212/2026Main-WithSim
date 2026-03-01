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

    motorConfig.MotorOutput.Inverted = Constants.HoodConstants.MOTOR_INVERTED;
    motorConfig.MotorOutput.NeutralMode = Constants.HoodConstants.NEUTRAL_MODE;

    motorConfig.Feedback.RotorToSensorRatio = 1.0;
    motorConfig.Feedback.SensorToMechanismRatio = 1.0;
    motorConfig.Feedback.FeedbackRotorOffset = Constants.HoodConstants.ROTOR_OFFSET;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
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

    motorConfig.Slot0.kP = Constants.HoodConstants.KP;
    motorConfig.Slot0.kI = Constants.HoodConstants.KI;
    motorConfig.Slot0.kD = Constants.HoodConstants.KD;
    motorConfig.Slot0.kS = Constants.HoodConstants.KS;
    motorConfig.Slot0.kV = Constants.HoodConstants.KV;
    motorConfig.Slot0.kA = Constants.HoodConstants.KA;
    motorConfig.Slot0.kG = Constants.HoodConstants.KG;
    motorConfig.Slot0.GravityType = Constants.HoodConstants.GRAVITY_TYPE;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.HoodConstants.CRUISE_VELOCITY;
    motorConfig.MotionMagic.MotionMagicAcceleration = Constants.HoodConstants.ACCELERATION;
    motorConfig.MotionMagic.MotionMagicJerk = Constants.HoodConstants.JERK;

    motorConfig.CurrentLimits.StatorCurrentLimit = Constants.HoodConstants.STATOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.HoodConstants.SUPPLY_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(new TalonFXConfiguration());
    motor.getConfigurator().apply(motorConfig);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double motorPositionRot = motor.getPosition().getValueAsDouble();
    double mechanismPositionRot = motorPositionRot * Constants.HoodConstants.GEAR_RATIO;

    inputs.positionRad =
        Units.rotationsToRadians(mechanismPositionRot)
            + Units.degreesToRadians(Constants.HoodConstants.MECHANISM_ZERO_ANGLE_DEG);

    double motorVelocityRotPerSec = motor.getVelocity().getValueAsDouble();
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(motorVelocityRotPerSec * Constants.HoodConstants.GEAR_RATIO);

    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentStatorAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.currentSupplyAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();

    inputs.mechanismPositionBeforeOffsetRot = mechanismPositionRot;
  }

  @Override
  public void setPositionSetpoint(double radiansFromHorizontal, double radPerSecond) {
    double clampedRadians =
        MathUtil.clamp(
            radiansFromHorizontal,
            Constants.HoodConstants.MIN_POSITION_RAD,
            Constants.HoodConstants.MAX_POSITION_RAD);

    // Subtract zero offset to get mechanism angle from its zero
    double mechanismAngleRad =
        clampedRadians - Units.degreesToRadians(Constants.HoodConstants.MECHANISM_ZERO_ANGLE_DEG);

    double mechanismRotations = Units.radiansToRotations(mechanismAngleRad);
    double motorRotations = mechanismRotations / Constants.HoodConstants.GEAR_RATIO;

    // Convert velocity feedforward: rad/s → motor rot/s → volts (via KV)
    double mechanismRotPerSec = Units.radiansToRotations(radPerSecond);
    double motorRotPerSec = mechanismRotPerSec / Constants.HoodConstants.GEAR_RATIO;
    double feedforwardVolts = motorRotPerSec * Constants.HoodConstants.KV;

    motor.setControl(
        positionControl.withPosition(motorRotations).withFeedForward(feedforwardVolts));
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
