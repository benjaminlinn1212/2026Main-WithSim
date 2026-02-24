package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import org.littletonrobotics.junction.Logger;

public class ConveyorSubsystem extends SubsystemBase {

  private final ConveyorIO io;
  private final ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

  /** Constructs a {@link ConveyorSubsystem} subsystem instance */
  public ConveyorSubsystem(ConveyorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Conveyor", inputs);
  }

  /**
   * Command to feed game piece to shooter
   *
   * @return A command that runs conveyor toward shooter
   */
  public Command goToShooter() {
    return run(() -> {
          io.setPercent(ConveyorConstants.TO_SHOOTER_PERCENT);
        })
        .withName("ConveyorToShooter");
  }

  /**
   * Command to move game piece to bucket (for ejecting)
   *
   * @return A command that runs conveyor toward bucket
   */
  public Command goToBucket() {
    return run(() -> {
          io.setPercent(ConveyorConstants.TO_BUCKET_PERCENT);
        })
        .withName("ConveyorToBucket");
  }

  /**
   * Command to stop the conveyor
   *
   * @return A command that stops the conveyor
   */
  public Command stop() {
    return runOnce(
            () -> {
              io.stop();
            })
        .withName("ConveyorStop");
  }

  /** Check if conveyor is stopped (for climb readiness) */
  public boolean atSetpoint() {
    return Math.abs(inputs.velocityRotPerSec) < 0.1; // Stopped if velocity near zero
  }

  /** Immediately stop the conveyor motor */
  public void stopMotor() {
    io.stop();
  }

  /**
   * Directly apply feed-to-shooter output. Called by Superstructure.periodic() every cycle when the
   * wanted state requires the conveyor to feed. Unlike goToShooter() command, this is a plain void
   * method.
   */
  public void applyFeedToShooter() {
    io.setPercent(ConveyorConstants.TO_SHOOTER_PERCENT);
  }

  /** Directly apply feed-to-bucket output. Called by Superstructure.periodic() for eject state. */
  public void applyFeedToBucket() {
    io.setPercent(ConveyorConstants.TO_BUCKET_PERCENT);
  }

  /** Toggle the conveyor direction */
  public void toggleDirection() {
    io.toggleDirection();
  }

  /**
   * Get the stator current of the conveyor motor (amps). Used for current-based shot completion
   * detection — when the conveyor current drops below a threshold for a sustained period, no FUEL
   * is being fed to the shooter.
   *
   * @return Conveyor stator current in amps
   */
  public double getCurrentAmps() {
    return inputs.currentAmps;
  }
}
