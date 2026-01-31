package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Hardware implementation of TurretIO using TalonFX motor controller. */
public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor;

  private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withSlot(0);
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

  public TurretIOTalonFX() {
    motor = new TalonFX(Constants.TurretConstants.MOTOR_CAN_ID, Constants.TurretConstants.CAN_BUS);

    // Configure TalonFX
    var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = Constants.TurretConstants.NEUTRAL_MODE;
    motorConfig.MotorOutput.Inverted = Constants.TurretConstants.MOTOR_INVERTED;

    // Software limits in motor rotor rotations
    // GEAR_RATIO is mechanism rotations per motor rotation
    // So: motor_rotation = mechanism_rotation / GEAR_RATIO
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

    // Motor inversion
    motorConfig.MotorOutput.Inverted = Constants.TurretConstants.MOTOR_INVERTED;

    // Current limits
    motorConfig.CurrentLimits.StatorCurrentLimit = Constants.TurretConstants.STATOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.SUPPLY_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Set motor encoder offset (adjust the motor's internal sensor reading)
    motorConfig.Feedback.FeedbackRotorOffset = Constants.TurretConstants.ROTOR_OFFSET;

    // Apply configuration
    motor.getConfigurator().apply(motorConfig);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Read motor position in rotations
    // GEAR_RATIO is mechanism rotations per motor rotation (e.g., 1/26.812)
    // So: mech_position = motor_position * GEAR_RATIO
    double motorPositionRot = motor.getPosition().getValueAsDouble();
    inputs.positionRot = motorPositionRot * Constants.TurretConstants.GEAR_RATIO;
    inputs.velocityRotPerSec =
        motor.getVelocity().getValueAsDouble() * Constants.TurretConstants.GEAR_RATIO;

    // No CANcoder - absolute position not available
    inputs.absolutePosition = 0.0;

    // Read electrical data
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentStatorAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.currentSupplyAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setPositionSetpoint(double rotationsFromCenter, double feedforwardVolts) {
    // Clamp setpoint to limits (in mechanism rotations)
    double clampedRotations =
        MathUtil.clamp(
            rotationsFromCenter,
            Units.radiansToRotations(Constants.TurretConstants.MIN_POSITION_RAD),
            Units.radiansToRotations(Constants.TurretConstants.MAX_POSITION_RAD));

    // Convert to motor rotations
    // GEAR_RATIO is mechanism rotations per motor rotation (e.g., 1/26.812)
    // So: motor_rotation = mechanism_rotation / GEAR_RATIO
    double motorRotations = clampedRotations / Constants.TurretConstants.GEAR_RATIO;

    // feedforwardVolts is already in Volts - use directly in MotionMagicVoltage
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
