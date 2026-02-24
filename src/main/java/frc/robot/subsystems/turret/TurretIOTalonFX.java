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
    // Per CTRE recommendation, set SensorToMechanismRatio to 1.0 and handle conversions in code
    motorConfig.Feedback.SensorToMechanismRatio = 1.0;
    // FeedbackRotorOffset is limited to [0,1) motor rotations — insufficient for
    // our 26.8:1 ratio.  We seed the encoder with setPosition() after applying config.
    motorConfig.Feedback.FeedbackRotorOffset = 0.0;

    // Software Limits (motor rotations - need to convert from mechanism limits)
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // Convert mechanism angle limits to motor rotations: motor_rot = mechanism_rot / gear_ratio
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(Constants.TurretConstants.MAX_POSITION_RAD)
            / Constants.TurretConstants.GEAR_RATIO;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.radiansToRotations(Constants.TurretConstants.MIN_POSITION_RAD)
            / Constants.TurretConstants.GEAR_RATIO;

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

    // Seed the encoder to the known boot position.
    // The turret physically starts at BOOT_POSITION_RAD (-90°).
    // setPosition() takes mechanism rotations, so convert from radians.
    // The motor internally divides by SensorToMechanismRatio (1.0) to get rotor position.
    double bootMechRot =
        Units.radiansToRotations(Constants.TurretConstants.BOOT_POSITION_RAD)
            / Constants.TurretConstants.GEAR_RATIO;
    motor.setPosition(bootMechRot);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Read motor position in motor rotations
    double motorPositionRot = motor.getPosition().getValueAsDouble();

    // Convert motor rotations to mechanism rotations: mechanism_rot = motor_rot * gear_ratio
    inputs.positionRot = motorPositionRot * Constants.TurretConstants.GEAR_RATIO;

    // Velocity: motor rotations/sec to mechanism rotations/sec
    double motorVelocityRotPerSec = motor.getVelocity().getValueAsDouble();
    inputs.velocityRotPerSec = motorVelocityRotPerSec * Constants.TurretConstants.GEAR_RATIO;

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

    // Convert mechanism rotations to motor rotations: motor_rot = mechanism_rot / gear_ratio
    double motorRotations = clampedRotations / Constants.TurretConstants.GEAR_RATIO;

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
