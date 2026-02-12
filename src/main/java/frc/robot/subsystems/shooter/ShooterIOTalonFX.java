package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX motor;

  private final VelocityVoltage velocityControl = new VelocityVoltage(0);
  private final VoltageOut voltageControl = new VoltageOut(0);

  public ShooterIOTalonFX() {
    motor = new TalonFX(ShooterConstants.MOTOR_CAN_ID, ShooterConstants.CAN_BUS);

    motor.getConfigurator().apply(shooterConfiguration());

    motor.setNeutralMode(ShooterConstants.NEUTRAL_MODE);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityRotPerSec = motor.getVelocity().getValueAsDouble();
    inputs.positionRot = motor.getPosition().getValueAsDouble();
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRotPerSec) {
    motor.setControl(velocityControl.withVelocity(velocityRotPerSec));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageControl.withOutput(volts));
  }

  /**
   * Gets the {@link TalonFXConfiguration} for the shooter motor
   *
   * @return The {@link TalonFXConfiguration} for the shooter motor
   */
  private TalonFXConfiguration shooterConfiguration() {
    TalonFXConfiguration configuration = new TalonFXConfiguration();

    // Per CTRE recommendation, set to 1.0 and handle conversions in code
    // For Shooter: GEAR_RATIO = 1.0 (direct drive), so motor rotations = mechanism rotations
    configuration.Feedback.SensorToMechanismRatio = 1.0;

    // Motor Inversion
    configuration.MotorOutput.Inverted = ShooterConstants.MOTOR_INVERTED;

    // PID and Feedforward (VelocityVoltage control)
    configuration.Slot0 =
        new Slot0Configs()
            .withKP(ShooterConstants.KP)
            .withKI(ShooterConstants.KI)
            .withKD(ShooterConstants.KD)
            .withKS(ShooterConstants.KS)
            .withKV(ShooterConstants.KV)
            .withKA(ShooterConstants.KA);

    // Current Limits
    configuration.CurrentLimits.StatorCurrentLimit = ShooterConstants.STATOR_CURRENT_LIMIT;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.SUPPLY_CURRENT_LOWER_TIME;

    return configuration;
  }
}
