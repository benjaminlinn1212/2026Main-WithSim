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
          io.setVelocity(ConveyorConstants.FEED_VELOCITY_RPS);
        })
        .withName("ConveyorToShooter");
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

  /** Apply feed-to-shooter (void, for Superstructure.periodic()). */
  public void applyFeedToShooter() {
    io.setVelocity(ConveyorConstants.FEED_VELOCITY_RPS);
  }

  /** Toggle the conveyor direction */
  public void toggleDirection() {
    io.toggleDirection();
  }

  /** Stator current (amps). Sustained low = no FUEL being fed. */
  public double getCurrentAmps() {
    return inputs.currentAmps;
  }
}
