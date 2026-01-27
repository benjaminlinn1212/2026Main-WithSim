package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {

  @AutoLog
  public static class ConveyorIOInputs {
    public double velocityRotPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public boolean directionInverted = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ConveyorIOInputs inputs) {}

  /** Run the conveyor motor at a specific duty cycle percent. */
  public default void setPercent(double percent) {}

  /** Stop the conveyor motor. */
  public default void stop() {}

  /** Toggle the direction of the conveyor. */
  public default void toggleDirection() {}

  /** Set the direction inversion state. */
  public default void setDirectionInverted(boolean inverted) {}
}
