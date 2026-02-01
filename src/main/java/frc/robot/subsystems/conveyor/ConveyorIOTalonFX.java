package frc.robot.subsystems.conveyor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorIOTalonFX implements ConveyorIO {

  private final TalonFX motor;
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
  private boolean directionInverted = false;

  public ConveyorIOTalonFX() {
    motor = new TalonFX(ConveyorConstants.MOTOR_CAN_ID, ConveyorConstants.CAN_BUS);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor Inversion and Neutral Mode
    config.MotorOutput.Inverted = ConveyorConstants.MOTOR_INVERTED;
    config.MotorOutput.NeutralMode = ConveyorConstants.NEUTRAL_MODE;

    // Current Limits
    config.CurrentLimits.StatorCurrentLimit = ConveyorConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ConveyorConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    inputs.velocityRotPerSec = motor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
    inputs.directionInverted = directionInverted;
  }

  @Override
  public void setPercent(double percent) {
    motor.setControl(dutyCycleControl.withOutput(directionInverted ? -percent : percent));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void toggleDirection() {
    directionInverted = !directionInverted;
  }

  @Override
  public void setDirectionInverted(boolean inverted) {
    directionInverted = inverted;
  }
}
