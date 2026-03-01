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

    motorConfig.MotorOutput.Inverted = Constants.TurretConstants.MOTOR_INVERTED;
    motorConfig.MotorOutput.NeutralMode = Constants.TurretConstants.NEUTRAL_MODE;

    // SensorToMechanismRatio = 1.0; we handle gear ratio conversions in code.
    // FeedbackRotorOffset is limited to [0,1) motor rotations — insufficient for
    // our 26.8:1 ratio. We seed the encoder with setPosition() after applying config.
    motorConfig.Feedback.SensorToMechanismRatio = 1.0;
    motorConfig.Feedback.FeedbackRotorOffset = 0.0;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(Constants.TurretConstants.MAX_POSITION_RAD)
            / Constants.TurretConstants.GEAR_RATIO;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.radiansToRotations(Constants.TurretConstants.MIN_POSITION_RAD)
            / Constants.TurretConstants.GEAR_RATIO;

    motorConfig.Slot0.kP = Constants.TurretConstants.KP;
    motorConfig.Slot0.kI = Constants.TurretConstants.KI;
    motorConfig.Slot0.kD = Constants.TurretConstants.KD;
    motorConfig.Slot0.kS = Constants.TurretConstants.KS;
    motorConfig.Slot0.kV = Constants.TurretConstants.KV;
    motorConfig.Slot0.kA = Constants.TurretConstants.KA;
    motorConfig.Slot0.kG = Constants.TurretConstants.KG;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretConstants.CRUISE_VELOCITY;
    motorConfig.MotionMagic.MotionMagicAcceleration = Constants.TurretConstants.ACCELERATION;
    motorConfig.MotionMagic.MotionMagicJerk = Constants.TurretConstants.JERK;

    motorConfig.CurrentLimits.StatorCurrentLimit = Constants.TurretConstants.STATOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.SUPPLY_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(motorConfig);

    // Seed encoder to known boot position (-90°). setPosition() takes motor rotations.
    double bootMechRot =
        Units.radiansToRotations(Constants.TurretConstants.BOOT_POSITION_RAD)
            / Constants.TurretConstants.GEAR_RATIO;
    motor.setPosition(bootMechRot);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    double motorPositionRot = motor.getPosition().getValueAsDouble();
    inputs.positionRot = motorPositionRot * Constants.TurretConstants.GEAR_RATIO;

    double motorVelocityRotPerSec = motor.getVelocity().getValueAsDouble();
    inputs.velocityRotPerSec = motorVelocityRotPerSec * Constants.TurretConstants.GEAR_RATIO;

    inputs.absolutePosition = 0.0;

    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentStatorAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.currentSupplyAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setPositionSetpoint(double rotationsFromCenter, double feedforwardVolts) {
    double clampedRotations =
        MathUtil.clamp(
            rotationsFromCenter,
            Units.radiansToRotations(Constants.TurretConstants.MIN_POSITION_RAD),
            Units.radiansToRotations(Constants.TurretConstants.MAX_POSITION_RAD));

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
