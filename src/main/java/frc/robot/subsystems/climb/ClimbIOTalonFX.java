package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOTalonFX implements ClimbIO {

  private final TalonFX motor;
  private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);

  public ClimbIOTalonFX() {
    motor = new TalonFX(ClimbConstants.MOTOR_CAN_ID, ClimbConstants.CAN_BUS);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor Inversion and Neutral Mode
    config.MotorOutput.Inverted = ClimbConstants.MOTOR_INVERTED;
    config.MotorOutput.NeutralMode = ClimbConstants.NEUTRAL_MODE;

    // PID and Feedforward (order: KP, KI, KD, KS, KV, KA, KG)
    config.Slot0.kP = ClimbConstants.KP;
    config.Slot0.kI = ClimbConstants.KI;
    config.Slot0.kD = ClimbConstants.KD;
    config.Slot0.kS = ClimbConstants.KS;
    config.Slot0.kV = ClimbConstants.KV;
    config.Slot0.kA = ClimbConstants.KA;
    config.Slot0.kG = ClimbConstants.KG;

    // Motion Magic
    config.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = ClimbConstants.ACCELERATION;
    config.MotionMagic.MotionMagicJerk = ClimbConstants.JERK;

    // Current Limits
    config.CurrentLimits.StatorCurrentLimit = ClimbConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ClimbConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);
    motor.setPosition(0); // Reset position on startup
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
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
