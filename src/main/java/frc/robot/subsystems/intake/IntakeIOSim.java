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

  private double upperTargetVelocityRPS = 0.0;
  private double lowerTargetVelocityRPS = 0.0;

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
    // Simulate voltage/current proportional to target velocity (rough approximation)
    double upperFraction = upperTargetVelocityRPS / 80.0;
    double lowerFraction = lowerTargetVelocityRPS / 80.0;

    inputs.upperAppliedVolts = upperFraction * 12.0;
    inputs.upperVelocityRotPerSec = upperTargetVelocityRPS;
    inputs.upperCurrentAmps = Math.abs(upperFraction) * 10.0;
    inputs.upperTemperatureCelsius = 25.0;

    inputs.lowerAppliedVolts = lowerFraction * 12.0;
    inputs.lowerVelocityRotPerSec = lowerTargetVelocityRPS;
    inputs.lowerCurrentAmps = Math.abs(lowerFraction) * 10.0;
    inputs.lowerTemperatureCelsius = 25.0;
  }

  @Override
  public void setUpperVelocity(double velocityRPS) {
    this.upperTargetVelocityRPS = velocityRPS;
    updateIntakeState();
  }

  @Override
  public void setLowerVelocity(double velocityRPS) {
    this.lowerTargetVelocityRPS = velocityRPS;
    updateIntakeState();
  }

  @Override
  public void stop() {
    this.upperTargetVelocityRPS = 0.0;
    this.lowerTargetVelocityRPS = 0.0;
    intakeSimulation.stopIntake();
  }

  /**
   * Update the intake simulation state based on motor velocity targets. If either motor has a
   * positive velocity target, activate the intake. Otherwise, stop it.
   */
  private void updateIntakeState() {
    if (upperTargetVelocityRPS > Constants.SimConstants.INTAKE_MOTOR_THRESHOLD
        || lowerTargetVelocityRPS > Constants.SimConstants.INTAKE_MOTOR_THRESHOLD) {
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
