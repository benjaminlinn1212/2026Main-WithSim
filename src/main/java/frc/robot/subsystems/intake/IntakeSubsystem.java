package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Constructs an {@link IntakeSubsystem} subsystem instance */
  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /**
   * Command to run the intake (pick up game pieces)
   *
   * @return A command that runs the intake
   */
  public Command intake() {
    return run(() -> {
          io.setUpperVelocity(IntakeConstants.UPPER_INTAKE_VELOCITY_RPS);
          io.setLowerVelocity(IntakeConstants.LOWER_INTAKE_VELOCITY_RPS);
        })
        .finallyDo(() -> io.stop())
        .withName("Intake");
  }

  /**
   * Command to stop the intake
   *
   * @return A command that stops the intake
   */
  public Command stop() {
    return runOnce(
            () -> {
              io.stop();
            })
        .withName("IntakeStop");
  }

  /** Check if intake is stopped (for climb readiness) */
  public boolean atSetpoint() {
    return Math.abs(inputs.upperVelocityRotPerSec) < 0.1
        && Math.abs(inputs.lowerVelocityRotPerSec) < 0.1; // Stopped if velocity near zero
  }

  /** Immediately stop the intake motor */
  public void stopMotor() {
    io.stop();
  }

  /** Apply intake motors (void, for Superstructure.periodic()). */
  public void applyIntake() {
    io.setUpperVelocity(IntakeConstants.UPPER_INTAKE_VELOCITY_RPS);
    io.setLowerVelocity(IntakeConstants.LOWER_INTAKE_VELOCITY_RPS);
  }

  /** Reverse lower roller during deploy (prevents FUEL falling out). Void for Superstructure. */
  public void applyDeployReverse() {
    io.setUpperVelocity(IntakeConstants.UPPER_INTAKE_VELOCITY_RPS);
    io.setLowerVelocity(IntakeConstants.LOWER_INTAKE_EJECT_RPS);
  }

  /** Reverse both rollers to eject FUEL out of the intake. Void for Superstructure. */
  public void applyEject() {
    io.setUpperVelocity(IntakeConstants.UPPER_INTAKE_EJECT_RPS);
    io.setLowerVelocity(IntakeConstants.LOWER_INTAKE_EJECT_RPS);
  }

  /** Lower roller only (upper stopped). Used during half-deploy jiggle. */
  public void applyIntakeLowerOnly() {
    io.setUpperVelocity(0.0);
    io.setLowerVelocity(IntakeConstants.LOWER_INTAKE_VELOCITY_RPS);
  }

  /** Upper roller stator current (amps). Spike = FUEL pickup detection in auto. */
  public double getUpperCurrentAmps() {
    return inputs.upperCurrentAmps;
  }

  /** Lower roller stator current (amps). Sustained low = no FUEL in path. */
  public double getLowerCurrentAmps() {
    return inputs.lowerCurrentAmps;
  }
}
