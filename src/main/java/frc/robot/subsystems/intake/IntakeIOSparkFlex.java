package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSparkFlex implements IntakeIO {

  private final SparkFlex motor;

  @SuppressWarnings("removal")
  public IntakeIOSparkFlex() {
    motor = new SparkFlex(IntakeConstants.MOTOR_CAN_ID, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    config.inverted(false);

    // Apply configuration
    motor.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRotPerSec = motor.getEncoder().getVelocity() / 60.0; // Convert RPM to RPS
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.temperatureCelsius = motor.getMotorTemperature();
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
