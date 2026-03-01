package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import frc.robot.Constants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/**
 * Simulation implementation of IntakeIO. Uses maple-sim IntakeSimulation for physics-based game
 * piece intake on the BACK side of the drivetrain (180°).
 */
public class IntakeIOSim implements IntakeIO {

  private double upperAppliedPercent = 0.0;
  private double lowerAppliedPercent = 0.0;

  private final IntakeSimulation intakeSimulation;

  public IntakeIOSim(SwerveDriveSimulation driveTrainSimulation) {
    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel", // Game piece type for 2026 Rebuilt
            driveTrainSimulation,
            Meters.of(Constants.SimConstants.INTAKE_WIDTH_METERS), // Width of the intake
            Meters.of(Constants.SimConstants.INTAKE_EXTENSION_METERS), // Extension beyond frame
            IntakeSimulation.IntakeSide.BACK, // Intake direction at the back (180°)
            Constants.SimConstants.INTAKE_CAPACITY); // Max fuel capacity
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.upperAppliedVolts = upperAppliedPercent * 12.0;
    inputs.upperVelocityRotPerSec = upperAppliedPercent * 80.0;
    inputs.upperCurrentAmps = Math.abs(upperAppliedPercent) * 10.0;
    inputs.upperTemperatureCelsius = 25.0;

    inputs.lowerAppliedVolts = lowerAppliedPercent * 12.0;
    inputs.lowerVelocityRotPerSec = lowerAppliedPercent * 80.0;
    inputs.lowerCurrentAmps = Math.abs(lowerAppliedPercent) * 10.0;
    inputs.lowerTemperatureCelsius = 25.0;
  }

  @Override
  public void setUpperPercent(double percent) {
    this.upperAppliedPercent = percent;
    updateIntakeState();
  }

  @Override
  public void setLowerPercent(double percent) {
    this.lowerAppliedPercent = percent;
    updateIntakeState();
  }

  @Override
  public void setPercent(double percent) {
    setUpperPercent(percent);
    setLowerPercent(percent);
  }

  @Override
  public void stop() {
    this.upperAppliedPercent = 0.0;
    this.lowerAppliedPercent = 0.0;
    intakeSimulation.stopIntake();
  }

  /**
   * Update the intake simulation state based on motor percent outputs. If either motor is running
   * forward (positive percent), activate the intake. Otherwise, stop it.
   */
  private void updateIntakeState() {
    if (upperAppliedPercent > Constants.SimConstants.INTAKE_MOTOR_THRESHOLD
        || lowerAppliedPercent > Constants.SimConstants.INTAKE_MOTOR_THRESHOLD) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
  }

  /**
   * Check if there is a fuel game piece inside the intake.
   *
   * @return true if the intake has captured a fuel
   */
  public boolean hasFuel() {
    return intakeSimulation.getGamePiecesAmount() > 0;
  }

  /**
   * Remove a fuel from the intake (e.g., when feeding to shooter). Returns true if a fuel was
   * successfully obtained, false if the intake was empty.
   */
  public boolean obtainFuel() {
    return intakeSimulation.obtainGamePieceFromIntake();
  }
}
