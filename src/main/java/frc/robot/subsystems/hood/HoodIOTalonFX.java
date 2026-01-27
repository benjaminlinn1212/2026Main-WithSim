package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Hardware implementation of HoodIO using TalonFX motor controller. */
public class HoodIOTalonFX implements HoodIO {
  private final TalonFX motor;

  private final PositionVoltage positionControl = new PositionVoltage(0.0).withSlot(0);
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

  public HoodIOTalonFX() {
    motor = new TalonFX(Constants.HoodConstants.MOTOR_CAN_ID);

    // Configure TalonFX
    var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Software limits
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(Constants.HoodConstants.MAX_POSITION_RAD)
            / Constants.HoodConstants.GEAR_RATIO;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.radiansToRotations(Constants.HoodConstants.MIN_POSITION_RAD)
            / Constants.HoodConstants.GEAR_RATIO;

    // PID configuration with gravity compensation
    motorConfig.Slot0.kP = Constants.HoodConstants.KP;
    motorConfig.Slot0.kI = Constants.HoodConstants.KI;
    motorConfig.Slot0.kD = Constants.HoodConstants.KD;
    motorConfig.Slot0.kS = Constants.HoodConstants.KS;
    motorConfig.Slot0.kV = Constants.HoodConstants.KV;
    motorConfig.Slot0.kA = Constants.HoodConstants.KA;
    motorConfig.Slot0.kG = Constants.HoodConstants.KG;
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // Motion Magic configuration
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.HoodConstants.CRUISE_VELOCITY;
    motorConfig.MotionMagic.MotionMagicAcceleration = Constants.HoodConstants.ACCELERATION;
    motorConfig.MotionMagic.MotionMagicJerk = Constants.HoodConstants.JERK;

    // Current limits
    motorConfig.CurrentLimits.StatorCurrentLimit = Constants.HoodConstants.STATOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.HoodConstants.SUPPLY_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(motorConfig);

    motor.optimizeBusUtilization();

    // Zero the motor position at startup (assumes hood starts at minimum angle)
    motor.setPosition(
        Units.radiansToRotations(Constants.HoodConstants.MIN_POSITION_RAD)
            / Constants.HoodConstants.GEAR_RATIO);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Read motor position and convert to radians
    double motorPositionRot = motor.getPosition().getValueAsDouble();
    inputs.positionRad =
        Units.rotationsToRadians(motorPositionRot * Constants.HoodConstants.GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(
            motor.getVelocity().getValueAsDouble() * Constants.HoodConstants.GEAR_RATIO);

    // Read electrical data
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentStatorAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.currentSupplyAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setPositionSetpoint(double radiansFromHorizontal, double radPerSecond) {
    // Clamp setpoint to limits
    double clampedRadians =
        MathUtil.clamp(
            radiansFromHorizontal,
            Constants.HoodConstants.MIN_POSITION_RAD,
            Constants.HoodConstants.MAX_POSITION_RAD);

    // Convert to motor rotations
    double motorRotations =
        Units.radiansToRotations(clampedRadians) / Constants.HoodConstants.GEAR_RATIO;
    double motorVelocity =
        Units.radiansToRotations(radPerSecond) / Constants.HoodConstants.GEAR_RATIO;

    motor.setControl(positionControl.withPosition(motorRotations).withVelocity(motorVelocity));
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
