package frc.robot.subsystems.conveyor;

import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.ShooterIOSim;

/**
 * Simulation implementation of ConveyorIO. Integrates with maple-sim: when the conveyor feeds
 * toward the shooter, it obtains fuel from the IntakeIOSim and notifies ShooterIOSim to launch a
 * projectile.
 */
public class ConveyorIOSim implements ConveyorIO {

  private double dutyCycle = 0.0;
  private boolean directionInverted = false;

  /** Reference to the intake sim for obtaining game pieces. May be null if not wired. */
  private IntakeIOSim intakeIOSim;

  /** Cooldown to avoid launching multiple projectiles per conveyor run. */
  private int feedCooldownTicks = 0;

  public ConveyorIOSim() {}

  /** Set the IntakeIOSim reference for fuel transfer. Call from RobotContainer after creation. */
  public void setIntakeIOSim(IntakeIOSim intakeIOSim) {
    this.intakeIOSim = intakeIOSim;
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    inputs.appliedVolts = dutyCycle * 12.0;
    inputs.currentAmps = Math.abs(dutyCycle) * 8.0;
    inputs.directionInverted = directionInverted;
    inputs.velocityRotPerSec = dutyCycle * 10.0; // rough approximation for logging

    // Decrement cooldown
    if (feedCooldownTicks > 0) {
      feedCooldownTicks--;
    }

    // Transfer fuel from intake to shooter when feeding
    if (dutyCycle > Constants.SimConstants.CONVEYOR_FEED_THRESHOLD) {
      // Signal the shooter sim that the conveyor is actively feeding this cycle.
      // This prevents preloaded fuel from launching while the flywheel is spinning
      // but the conveyor is stopped (e.g. AIMING_WHILE_INTAKING state).
      ShooterIOSim.setConveyorFeeding();

      if (feedCooldownTicks == 0 && intakeIOSim != null) {
        if (intakeIOSim.obtainFuel()) {
          ShooterIOSim.notifyFuelReady();
          feedCooldownTicks = Constants.SimConstants.FEED_COOLDOWN_TICKS;
        }
      }
    }
  }

  @Override
  public void setVelocity(double dutyCycle) {
    this.dutyCycle = directionInverted ? -dutyCycle : dutyCycle;
  }

  @Override
  public void stop() {
    this.dutyCycle = 0.0;
  }

  @Override
  public void toggleDirection() {
    directionInverted = !directionInverted;
  }

  @Override
  public void setDirectionInverted(boolean inverted) {
    directionInverted = inverted;
  }
}
