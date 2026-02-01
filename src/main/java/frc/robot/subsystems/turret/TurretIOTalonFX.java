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

    // Motor Inversion and Neutral Mode
    motorConfig.MotorOutput.Inverted = Constants.TurretConstants.MOTOR_INVERTED;
    motorConfig.MotorOutput.NeutralMode = Constants.TurretConstants.NEUTRAL_MODE;

    // Feedback Configuration
    // Configure SensorToMechanismRatio so Phoenix handles the conversion
    // SensorToMechanismRatio in Phoenix 6 = sensor rotations per mechanism rotation
    // Our GEAR_RATIO is mechanism per motor, so we need the reciprocal
    motorConfig.Feedback.SensorToMechanismRatio = 1.0 / Constants.TurretConstants.GEAR_RATIO;
    motorConfig.Feedback.FeedbackRotorOffset = Constants.TurretConstants.ROTOR_OFFSET;

    // Software Limits (now in mechanism rotations since we use SensorToMechanismRatio)
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(Constants.TurretConstants.MAX_POSITION_RAD);
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.radiansToRotations(Constants.TurretConstants.MIN_POSITION_RAD);

    // PID and Feedforward (order: KP, KI, KD, KS, KV, KA, KG)
    motorConfig.Slot0.kP = Constants.TurretConstants.KP;
    motorConfig.Slot0.kI = Constants.TurretConstants.KI;
    motorConfig.Slot0.kD = Constants.TurretConstants.KD;
    motorConfig.Slot0.kS = Constants.TurretConstants.KS;
    motorConfig.Slot0.kV = Constants.TurretConstants.KV;
    motorConfig.Slot0.kA = Constants.TurretConstants.KA;
    motorConfig.Slot0.kG = Constants.TurretConstants.KG;

    // Motion Magic
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretConstants.CRUISE_VELOCITY;
    motorConfig.MotionMagic.MotionMagicAcceleration = Constants.TurretConstants.ACCELERATION;
    motorConfig.MotionMagic.MotionMagicJerk = Constants.TurretConstants.JERK;

    // Current Limits
    motorConfig.CurrentLimits.StatorCurrentLimit = Constants.TurretConstants.STATOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.SUPPLY_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Apply configuration
    motor.getConfigurator().apply(motorConfig);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Read motor position - Phoenix 6 already converts to mechanism rotations via
    // SensorToMechanismRatio
    inputs.positionRot = motor.getPosition().getValueAsDouble();
    inputs.velocityRotPerSec = motor.getVelocity().getValueAsDouble();

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

    // Phoenix 6 handles the gear ratio conversion via SensorToMechanismRatio
    // Just send mechanism rotations directly
    motor.setControl(
        positionControl.withPosition(clampedRotations).withFeedForward(feedforwardVolts));
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
