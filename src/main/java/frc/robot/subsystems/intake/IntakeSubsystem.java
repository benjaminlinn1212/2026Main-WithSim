package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();

  private static IntakeSubsystem instance;

  /** Constructs an {@link IntakeSubsystem} subsystem instance */
  private IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  /** Gets the singleton instance of the intake subsystem. */
  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      if (RobotBase.isReal()) {
        instance = new IntakeSubsystem(new IntakeIOTalonFX());
      } else {
        instance = new IntakeSubsystem(new IntakeIOSim());
      }
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Log all inputs for upper motor
    Logger.recordOutput("Intake/Upper/VelocityRotPerSec", inputs.upperVelocityRotPerSec);
    Logger.recordOutput("Intake/Upper/AppliedVolts", inputs.upperAppliedVolts);
    Logger.recordOutput("Intake/Upper/CurrentAmps", inputs.upperCurrentAmps);
    Logger.recordOutput("Intake/Upper/TemperatureCelsius", inputs.upperTemperatureCelsius);

    // Log all inputs for lower motor
    Logger.recordOutput("Intake/Lower/VelocityRotPerSec", inputs.lowerVelocityRotPerSec);
    Logger.recordOutput("Intake/Lower/AppliedVolts", inputs.lowerAppliedVolts);
    Logger.recordOutput("Intake/Lower/CurrentAmps", inputs.lowerCurrentAmps);
    Logger.recordOutput("Intake/Lower/TemperatureCelsius", inputs.lowerTemperatureCelsius);
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

  /** Immediately stop the intake motor */
  public void stopMotor() {
    io.stop();
  }
}
