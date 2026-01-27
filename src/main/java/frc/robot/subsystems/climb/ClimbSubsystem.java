package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {

  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  /** Constructs a {@link ClimbSubsystem} subsystem instance */
  public ClimbSubsystem(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  /**
   * Command to extend the climber to full extension
   *
   * @return A command that extends the climber
   */
  public Command extend() {
    return runOnce(
            () -> {
              io.setPosition(ClimbConstants.EXTENDED_POSITION);
            })
        .withName("ClimbExtend");
  }

  /**
   * Command to retract the climber (pull robot up)
   *
   * @return A command that retracts the climber
   */
  public Command retract() {
    return runOnce(
            () -> {
              io.setPosition(ClimbConstants.RETRACTED_POSITION);
            })
        .withName("ClimbRetract");
  }

  /**
   * Command to go to stowed position
   *
   * @return A command that stows the climber
   */
  public Command stow() {
    return runOnce(
            () -> {
              io.setPosition(ClimbConstants.STOWED_POSITION);
            })
        .withName("ClimbStow");
  }

  /** Get current climb position in rotations */
  public double getPosition() {
    return inputs.positionRotations;
  }

  /** Check if climber is at target position */
  public boolean atPosition(double targetPosition) {
    return Math.abs(inputs.positionRotations - targetPosition) < ClimbConstants.POSITION_TOLERANCE;
  }

  /** Immediately stop the climb motor */
  public void stopMotor() {
    io.stop();
  }
}
