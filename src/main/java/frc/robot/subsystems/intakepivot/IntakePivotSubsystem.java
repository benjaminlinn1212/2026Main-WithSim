package frc.robot.subsystems.intakepivot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;
import org.littletonrobotics.junction.Logger;

public class IntakePivotSubsystem extends SubsystemBase {

  private final IntakePivotIO io;
  private final IntakePivotIO.IntakePivotIOInputs inputs = new IntakePivotIO.IntakePivotIOInputs();

  private static IntakePivotSubsystem instance;

  /** Constructs an {@link IntakePivotSubsystem} subsystem instance */
  private IntakePivotSubsystem(IntakePivotIO io) {
    this.io = io;
  }

  /** Gets the singleton instance of the intake pivot subsystem */
  public static IntakePivotSubsystem getInstance() {
    if (instance == null) {
      if (RobotBase.isReal()) {
        instance = new IntakePivotSubsystem(new IntakePivotIOTalonFX());
      } else {
        instance = new IntakePivotSubsystem(new IntakePivotIOSim());
      }
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Log all inputs
    Logger.recordOutput("IntakePivot/PositionRotations", inputs.positionRotations);
    Logger.recordOutput("IntakePivot/VelocityRotPerSec", inputs.velocityRotPerSec);
    Logger.recordOutput("IntakePivot/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("IntakePivot/CurrentAmps", inputs.currentAmps);
    Logger.recordOutput("IntakePivot/TemperatureCelsius", inputs.temperatureCelsius);
  }

  /**
   * Command to deploy/extend the intake (lower it down)
   *
   * @return A command that extends the intake pivot
   */
  public Command deploy() {
    return runOnce(
            () -> {
              io.setPosition(IntakePivotConstants.DEPLOYED_POSITION);
            })
        .withName("IntakePivotDeploy");
  }

  /**
   * Command to retract the intake (raise it up)
   *
   * @return A command that retracts the intake pivot
   */
  public Command retract() {
    return runOnce(
            () -> {
              io.setPosition(IntakePivotConstants.RETRACTED_POSITION);
            })
        .withName("IntakePivotRetract");
  }

  /** Get current pivot position in rotations */
  public double getPosition() {
    return inputs.positionRotations;
  }

  /** Check if pivot is at target position */
  public boolean atPosition(double targetPosition) {
    return Math.abs(inputs.positionRotations - targetPosition)
        < IntakePivotConstants.POSITION_TOLERANCE;
  }

  /** Check if intake is deployed */
  public boolean isDeployed() {
    return atPosition(IntakePivotConstants.DEPLOYED_POSITION);
  }

  /** Check if intake is retracted */
  public boolean isRetracted() {
    return atPosition(IntakePivotConstants.RETRACTED_POSITION);
  }

  /** Immediately stop the pivot motor */
  public void stopMotor() {
    io.stop();
  }
}
