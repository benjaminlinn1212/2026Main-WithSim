package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Hardware implementation of TurretIO using TalonFX motor controller and CANCoder. */
public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor;
  private final CANcoder cancoder;

  private final PositionVoltage positionControl = new PositionVoltage(0.0).withSlot(0);
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

  private boolean hasZeroed = false;

  public TurretIOTalonFX() {
    motor = new TalonFX(Constants.TurretConstants.MOTOR_CAN_ID, Constants.TurretConstants.CAN_BUS);
    cancoder =
        new CANcoder(Constants.TurretConstants.CANCODER_CAN_ID, Constants.TurretConstants.CAN_BUS);

    // Configure CANCoder
    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = Constants.TurretConstants.CANCODER_OFFSET;
    cancoder.getConfigurator().apply(cancoderConfig);

    // Configure TalonFX
    var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Software limits
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(Constants.TurretConstants.MAX_POSITION_RAD)
            / Constants.TurretConstants.GEAR_RATIO;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.radiansToRotations(Constants.TurretConstants.MIN_POSITION_RAD)
            / Constants.TurretConstants.GEAR_RATIO;

    // PID configuration
    motorConfig.Slot0.kP = Constants.TurretConstants.KP;
    motorConfig.Slot0.kI = Constants.TurretConstants.KI;
    motorConfig.Slot0.kD = Constants.TurretConstants.KD;
    motorConfig.Slot0.kS = Constants.TurretConstants.KS;
    motorConfig.Slot0.kV = Constants.TurretConstants.KV;
    motorConfig.Slot0.kA = Constants.TurretConstants.KA;

    // Motion Magic configuration
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretConstants.CRUISE_VELOCITY;
    motorConfig.MotionMagic.MotionMagicAcceleration = Constants.TurretConstants.ACCELERATION;
    motorConfig.MotionMagic.MotionMagicJerk = Constants.TurretConstants.JERK;

    // Current limits
    motorConfig.CurrentLimits.StatorCurrentLimit = Constants.TurretConstants.STATOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.SUPPLY_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(motorConfig);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Zero the motor position to CANCoder on first update
    if (!hasZeroed) {
      double cancoderPositionRot = cancoder.getAbsolutePosition().getValueAsDouble();
      motor.setPosition(cancoderPositionRot / Constants.TurretConstants.GEAR_RATIO);
      hasZeroed = true;
    }

    // Read motor position and convert to radians
    double motorPositionRot = motor.getPosition().getValueAsDouble();
    inputs.positionRad =
        Units.rotationsToRadians(motorPositionRot * Constants.TurretConstants.GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(
            motor.getVelocity().getValueAsDouble() * Constants.TurretConstants.GEAR_RATIO);

    // Read CANCoder absolute position (in radians)
    inputs.absolutePosition =
        Units.rotationsToRadians(cancoder.getAbsolutePosition().getValueAsDouble());

    // Read electrical data
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentStatorAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.currentSupplyAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setPositionSetpoint(double radiansFromCenter, double radPerSecond) {
    // Clamp setpoint to limits
    double clampedRadians =
        MathUtil.clamp(
            radiansFromCenter,
            Constants.TurretConstants.MIN_POSITION_RAD,
            Constants.TurretConstants.MAX_POSITION_RAD);

    // Convert to motor rotations
    double motorRotations =
        Units.radiansToRotations(clampedRadians) / Constants.TurretConstants.GEAR_RATIO;
    double motorVelocity =
        Units.radiansToRotations(radPerSecond) / Constants.TurretConstants.GEAR_RATIO;

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
