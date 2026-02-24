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
          io.setPercent(IntakeConstants.INTAKE_PERCENT);
        })
        .finallyDo(() -> io.stop())
        .withName("Intake");
  }

  /**
   * Command to run the intake in reverse (outtake)
   *
   * @return A command that runs the intake in reverse
   */
  public Command outtake() {
    return run(() -> {
          io.setPercent(IntakeConstants.OUTTAKE_PERCENT);
        })
        .finallyDo(() -> io.stop())
        .withName("Outtake");
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

  /**
   * Directly apply intake motor output. Called by Superstructure.periodic() every cycle when the
   * wanted state requires the intake to run. Unlike the intake() command, this is a plain void
   * method — no command scheduling overhead.
   */
  public void applyIntake() {
    io.setPercent(IntakeConstants.INTAKE_PERCENT);
  }

  /**
   * Directly apply outtake motor output. Called by Superstructure.periodic() every cycle when the
   * wanted state requires the intake to eject.
   */
  public void applyOuttake() {
    io.setPercent(IntakeConstants.OUTTAKE_PERCENT);
  }

  /**
   * Get the stator current of the upper intake roller (amps). Used for current-based FUEL pickup
   * detection in auto — a current spike above a threshold indicates FUEL has contacted the rollers.
   *
   * @return Upper roller stator current in amps
   */
  public double getUpperCurrentAmps() {
    return inputs.upperCurrentAmps;
  }

  /**
   * Get the stator current of the lower intake roller (amps). Used for current-based FUEL presence
   * detection — when the lower roller current drops below a threshold for a sustained period, no
   * FUEL is in the intake path.
   *
   * @return Lower roller stator current in amps
   */
  public double getLowerCurrentAmps() {
    return inputs.lowerCurrentAmps;
  }
}
