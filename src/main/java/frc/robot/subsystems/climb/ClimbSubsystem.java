package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {

  private final ClimbIO io;
  private final ClimbIO.ClimbIOInputs inputs = new ClimbIO.ClimbIOInputs();

  private static ClimbSubsystem instance;

  /** Constructs a {@link ClimbSubsystem} subsystem instance */
  private ClimbSubsystem(ClimbIO io) {
    this.io = io;
  }

  /** Gets the singleton instance of the climb subsystem */
  public static ClimbSubsystem getInstance() {
    if (instance == null) {
      if (RobotBase.isReal()) {
        instance = new ClimbSubsystem(new ClimbIOTalonFX());
      } else {
        instance = new ClimbSubsystem(new ClimbIOSim());
      }
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Log all inputs
    Logger.recordOutput("Climb/PositionRotations", inputs.positionRotations);
    Logger.recordOutput("Climb/VelocityRotPerSec", inputs.velocityRotPerSec);
    Logger.recordOutput("Climb/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("Climb/CurrentAmps", inputs.currentAmps);
    Logger.recordOutput("Climb/TemperatureCelsius", inputs.temperatureCelsius);
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
