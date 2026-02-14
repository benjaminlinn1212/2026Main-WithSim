package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ShooterSetpoint;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Hood subsystem that controls the hood angle for shot trajectory adjustment. Supports two states:
 * stow and aimHub.
 *
 * <p>Uses ShooterSetpoint utility for aim calculations, which handles smart target selection
 * including neutral zone detection.
 */
public class HoodSubsystem extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private double positionSetpointRad = Constants.HoodConstants.STOW_POSITION;

  // ShooterSetpoint supplier for coordinated aiming
  private Supplier<ShooterSetpoint> setpointSupplier =
      () -> new ShooterSetpoint(0, 0, 0, 0, 0, false);

  public enum HoodState {
    STOW,
    AIM_HUB
  }

  private HoodState currentState = HoodState.STOW;

  public HoodSubsystem(HoodIO io) {
    this.io = io;
  }

  /**
   * Set the shooter setpoint supplier for coordinated aiming. Call this from RobotContainer to
   * connect all aiming subsystems through ShooterSetpoint.
   */
  public void setShooterSetpointSupplier(Supplier<ShooterSetpoint> supplier) {
    this.setpointSupplier = supplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    Logger.recordOutput("Hood/State", currentState.toString());
    Logger.recordOutput("Hood/SetpointRad", positionSetpointRad);
  }

  /** Command to set hood to stow position (safe angle). */
  public Command stow() {
    return runOnce(
            () -> {
              currentState = HoodState.STOW;
            })
        .andThen(positionSetpointCommand(() -> Constants.HoodConstants.STOW_POSITION, () -> 0.0))
        .withName("Hood Stow");
  }

  /**
   * Command to aim hood using ShooterSetpoint calculations. ShooterSetpoint handles smart target
   * selection including neutral zone detection.
   *
   * <p>Benefits: - Coordinated aiming with turret and shooter - Physics-based trajectory
   * calculation - Distance-based angle interpolation - No duplicate calculation logic
   */
  public Command aimHub() {
    return runOnce(
            () -> {
              currentState = HoodState.AIM_HUB;
            })
        .andThen(
            positionSetpointCommand(
                () -> {
                  // Get setpoint from ShooterSetpoint utility
                  ShooterSetpoint setpoint = setpointSupplier.get();

                  // Extract hood angle from setpoint
                  return setpoint.getHoodAngleRad();
                },
                () -> {
                  // Extract hood feedforward from setpoint
                  ShooterSetpoint setpoint = setpointSupplier.get();
                  return setpoint.getHoodFeedforwardRadPerSec();
                }))
        .withName("Hood Aim Hub");
  }

  /** Command to set hood position with feedforward velocity. */
  public Command positionSetpointCommand(
      DoubleSupplier radiansFromHorizontal, DoubleSupplier ffVelocity) {
    return run(() -> {
          double setpoint = radiansFromHorizontal.getAsDouble();
          setPositionSetpointImpl(setpoint, ffVelocity.getAsDouble());
          positionSetpointRad = setpoint;
        })
        .withName("Hood Position Setpoint");
  }

  private void setPositionSetpointImpl(double radiansFromHorizontal, double radPerSecond) {
    Logger.recordOutput("Hood/SetPositionSetpoint/radiansFromHorizontal", radiansFromHorizontal);
    Logger.recordOutput("Hood/SetPositionSetpoint/radPerSecond", radPerSecond);
    io.setPositionSetpoint(radiansFromHorizontal, radPerSecond);
  }

  /**
   * Direct method to set hood position (no command scheduling). Use this for simple test bindings
   * where you want to call it from a runOnce() lambda, similar to shooter.setVelocity().
   */
  public void setPosition(double radiansFromHorizontal) {
    positionSetpointRad = radiansFromHorizontal;
    setPositionSetpointImpl(radiansFromHorizontal, 0.0);
  }

  public double getCurrentPosition() {
    return inputs.positionRad;
  }

  public double getSetpoint() {
    return positionSetpointRad;
  }

  public HoodState getCurrentState() {
    return currentState;
  }

  /** Checks if hood is at the desired setpoint within tolerance. */
  public boolean atSetpoint() {
    return Math.abs(getCurrentPosition() - positionSetpointRad)
        < Constants.HoodConstants.AIMING_TOLERANCE_RAD;
  }

  // ==================== Direct Apply Methods (for Superstructure periodic) ====================

  /**
   * Directly apply the stow position. Called by Superstructure.periodic() every cycle when the
   * wanted state requires the hood to be stowed. Unlike the stow() command, this is a plain void
   * method — no command scheduling overhead.
   */
  public void applyStow() {
    currentState = HoodState.STOW;
    setPositionSetpointImpl(Constants.HoodConstants.STOW_POSITION, 0.0);
    positionSetpointRad = Constants.HoodConstants.STOW_POSITION;
  }

  /**
   * Directly apply the aiming position using ShooterSetpoint. Called by Superstructure.periodic()
   * every cycle when the wanted state requires the hood to aim. Unlike the aimHub() command, this
   * is a plain void method — no command scheduling overhead.
   */
  public void applyAiming() {
    currentState = HoodState.AIM_HUB;
    ShooterSetpoint setpoint = setpointSupplier.get();
    double hoodAngle = setpoint.getHoodAngleRad();
    double feedforward = setpoint.getHoodFeedforwardRadPerSec();
    setPositionSetpointImpl(hoodAngle, feedforward);
    positionSetpointRad = hoodAngle;
  }

  /** Command for open-loop duty cycle control (for testing). */
  public Command setDutyCycle(DoubleSupplier dutyCycle) {
    return run(() -> io.setOpenLoopDutyCycle(dutyCycle.getAsDouble()))
        .finallyDo(() -> io.stop())
        .withName("Hood Duty Cycle");
  }
}
