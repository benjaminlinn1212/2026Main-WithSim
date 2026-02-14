package frc.robot.subsystems.intakepivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;
import org.littletonrobotics.junction.Logger;

public class IntakePivotSubsystem extends SubsystemBase {

  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  /** Constructs an {@link IntakePivotSubsystem} subsystem instance */
  public IntakePivotSubsystem(IntakePivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);
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
   * Command to stow the intake (raise it up)
   *
   * @return A command that stows the intake pivot
   */
  public Command stow() {
    return runOnce(
            () -> {
              io.setPosition(IntakePivotConstants.STOWED_POSITION);
            })
        .withName("IntakePivotStow");
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

  /** Check if intake is stowed */
  public boolean isStowed() {
    return atPosition(IntakePivotConstants.STOWED_POSITION);
  }

  /** Immediately stop the pivot motor */
  public void stopMotor() {
    io.stop();
  }

  /**
   * Directly apply the deployed position. Called by Superstructure.periodic() for states that need
   * intake deployed. Unlike the deploy() command, this is a plain void method.
   */
  public void applyDeploy() {
    io.setPosition(IntakePivotConstants.DEPLOYED_POSITION);
  }

  /**
   * Directly apply the stowed position. Called by Superstructure.periodic() for states that need
   * intake stowed. Unlike the stow() command, this is a plain void method.
   */
  public void applyStow() {
    io.setPosition(IntakePivotConstants.STOWED_POSITION);
  }
}
