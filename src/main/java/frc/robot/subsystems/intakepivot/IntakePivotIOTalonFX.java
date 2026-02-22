package frc.robot.subsystems.intakepivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.util.IntakePivotFF;

public class IntakePivotIOTalonFX implements IntakePivotIO {

  private final TalonFX motor;
  private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);
  private final IntakePivotFF gravityFF;

  public IntakePivotIOTalonFX() {
    motor = new TalonFX(IntakePivotConstants.MOTOR_CAN_ID, IntakePivotConstants.CAN_BUS);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor Inversion and Neutral Mode
    config.MotorOutput.Inverted = IntakePivotConstants.MOTOR_INVERTED;
    config.MotorOutput.NeutralMode = IntakePivotConstants.NEUTRAL_MODE;

    // Feedback Configuration
    // Per CTRE recommendation, set SensorToMechanismRatio to 1.0 and handle conversions in code
    // For IntakePivot: GEAR_RATIO = 1.0 (direct drive), so motor rotations = mechanism rotations
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.Feedback.FeedbackRotorOffset = IntakePivotConstants.MOTOR_ROTOR_OFFSET;

    // Soft Limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakePivotConstants.SOFT_LIMIT_FORWARD;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakePivotConstants.SOFT_LIMIT_REVERSE;

    // PID and Feedforward (order: KP, KI, KD, KS, KV, KA)
    // Note: kG is NOT set here — gravity FF is computed by IntakePivotFF via linkage kinematics
    // and injected as FeedForward voltage on the MotionMagicVoltage control request.
    config.Slot0.kP = IntakePivotConstants.KP;
    config.Slot0.kI = IntakePivotConstants.KI;
    config.Slot0.kD = IntakePivotConstants.KD;
    config.Slot0.kS = IntakePivotConstants.KS;
    config.Slot0.kV = IntakePivotConstants.KV;
    config.Slot0.kA = IntakePivotConstants.KA;

    // Motion Magic
    config.MotionMagic.MotionMagicCruiseVelocity = IntakePivotConstants.CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = IntakePivotConstants.ACCELERATION;
    config.MotionMagic.MotionMagicJerk = IntakePivotConstants.JERK;

    // Current Limits
    config.CurrentLimits.StatorCurrentLimit = IntakePivotConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = IntakePivotConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);

    motor.optimizeBusUtilization();

    // Build IntakePivotFF parameters from Constants
    IntakePivotFF.Params ffParams = new IntakePivotFF.Params();
    ffParams.pinionRadius_m = IntakePivotConstants.FF_PINION_RADIUS_M;
    ffParams.rackGearRatio = IntakePivotConstants.FF_RACK_GEAR_RATIO;
    ffParams.rackTheta_rad = IntakePivotConstants.FF_RACK_THETA_RAD;
    ffParams.A0x_m = IntakePivotConstants.FF_A0X_M;
    ffParams.A0y_m = IntakePivotConstants.FF_A0Y_M;
    ffParams.Ox_m = IntakePivotConstants.FF_OX_M;
    ffParams.Oy_m = IntakePivotConstants.FF_OY_M;
    ffParams.elbowRadius_m = IntakePivotConstants.FF_ELBOW_RADIUS_M;
    ffParams.couplerLength_m = IntakePivotConstants.FF_COUPLER_LENGTH_M;
    ffParams.mass_kg = IntakePivotConstants.FF_MASS_KG;
    ffParams.comRadius_m = IntakePivotConstants.FF_COM_RADIUS_M;
    ffParams.comAngleOffset_rad = IntakePivotConstants.FF_COM_ANGLE_OFFSET_RAD;
    ffParams.motorKt_NmPerA = IntakePivotConstants.FF_MOTOR_KT;
    ffParams.motorR_ohm = IntakePivotConstants.FF_MOTOR_R_OHM;
    ffParams.efficiency = IntakePivotConstants.FF_EFFICIENCY;
    gravityFF = new IntakePivotFF(ffParams);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    // Note: GEAR_RATIO = 1.0 (direct drive), so motor rotations = mechanism rotations
    inputs.positionRotations = motor.getPosition().getValueAsDouble();
    inputs.velocityRotPerSec = motor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setPosition(double positionRotations) {
    double ffVolts = 0.0;
    if (IntakePivotConstants.USE_CALCULATED_FF) {
      // Compute gravity FF voltage from linkage kinematics at the CURRENT position
      double currentPos = motor.getPosition().getValueAsDouble();
      ffVolts = gravityFF.calculateVoltageFF(currentPos);
    }

    // MotionMagicVoltage: PID + Motion Profile output is in Volts; FeedForward is additive Volts
    motor.setControl(positionControl.withPosition(positionRotations).withFeedForward(ffVolts));
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
