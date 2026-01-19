package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Superstructure coordinates multiple subsystems to achieve game objectives. Controls high-level
 * robot states and sequences.
 */
public class Superstructure extends SubsystemBase {

  // Subsystem instances
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;
  private final IntakePivotSubsystem intakePivot;
  private final ConveyorSubsystem conveyor;
  private final ClimbSubsystem climb;

  // Current state
  private SuperstructureState currentState = SuperstructureState.STOWED;

  private static Superstructure instance;

  /** Robot states for coordinated subsystem control */
  public enum SuperstructureState {
    STOWED, // Safe travel position
    INTAKING, // Picking up balls from ground
    HOLDING, // Has ball, ready to score
    AIMING, // Preparing to shoot (shooter spun up)
    SHOOTING, // Feeding ball to shooter
    CLIMBING_EXTEND, // Extending climber
    CLIMBING_RETRACT // Retracting climber (pulling up)
  }

  /** Private constructor - use getInstance() */
  private Superstructure() {
    this.shooter = ShooterSubsystem.getInstance();
    this.intake = IntakeSubsystem.getInstance();
    this.intakePivot = IntakePivotSubsystem.getInstance();
    this.conveyor = ConveyorSubsystem.getInstance();
    this.climb = ClimbSubsystem.getInstance();
  }

  /** Gets the singleton instance */
  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // Log current state
    Logger.recordOutput("Superstructure/State", currentState.toString());
  }

  /** Get current superstructure state */
  public SuperstructureState getState() {
    return currentState;
  }

  /** Set the current state (for internal use) */
  private void setState(SuperstructureState state) {
    this.currentState = state;
  }

  // ==================== High-Level Commands ====================

  /** Command to stow all mechanisms (safe travel position) */
  public Command stow() {
    return Commands.parallel(
            Commands.runOnce(() -> setState(SuperstructureState.STOWED)),
            intake.stop(),
            intakePivot.stow(), // Stow intake pivot
            conveyor.stop(),
            Commands.runOnce(() -> shooter.stop()))
        .withName("SuperstructureStow");
  }

  /** Command to intake balls from the ground */
  public Command intakeFromGround() {
    return Commands.sequence(
            Commands.runOnce(() -> setState(SuperstructureState.INTAKING)),
            Commands.parallel(
                intakePivot.deploy(), // Deploy intake pivot down
                intake.intake(), // Run intake
                conveyor.goToShooter() // Move ball inward
                ))
        .withName("SuperstructureIntake");
  }

  /** Command to prepare for scoring (spin up shooter) This should be run while driving/aiming */
  public Command prepareToScore() {
    return Commands.sequence(
            Commands.runOnce(() -> setState(SuperstructureState.AIMING)),
            Commands.parallel(
                shooter.spinUpForSpeaker(), // Spin up shooter
                intakePivot.stow(), // Stow intake pivot
                intake.stop(),
                conveyor.stop()))
        .withName("SuperstructurePrepare");
  }

  /**
   * Command to score in basket (feed ball when ready) Should only be called after prepareToScore()
   * and when aimed
   */
  public Command scoreInBasket() {
    return Commands.sequence(
            Commands.runOnce(() -> setState(SuperstructureState.SHOOTING)),
            // Wait for shooter to be at speed
            Commands.waitUntil(() -> shooter.atVelocity(60.0)), // Adjust target velocity
            // Feed the ball
            conveyor.goToShooter().withTimeout(1.0),
            // Return to holding state
            Commands.runOnce(() -> setState(SuperstructureState.HOLDING)))
        .withName("SuperstructureScore");
  }

  /** Complete shooting sequence: prepare + aim + shoot For use when already aimed at target */
  public Command fullShootingSequence() {
    return Commands.sequence(
            prepareToScore(),
            Commands.waitSeconds(0.5), // Allow shooter to spin up
            scoreInBasket(),
            stow())
        .withName("SuperstructureFullShoot");
  }

  /** Command to extend climber */
  public Command extendClimber() {
    return Commands.sequence(
            Commands.runOnce(() -> setState(SuperstructureState.CLIMBING_EXTEND)),
            stow(), // Stow other mechanisms first
            climb.extend())
        .withName("SuperstructureClimbExtend");
  }

  /** Command to retract climber (pull up) */
  public Command retractClimber() {
    return Commands.sequence(
            Commands.runOnce(() -> setState(SuperstructureState.CLIMBING_RETRACT)), climb.retract())
        .withName("SuperstructureClimbRetract");
  }

  /** Emergency stop all subsystems */
  public Command emergencyStop() {
    return Commands.parallel(
            Commands.runOnce(() -> setState(SuperstructureState.STOWED)),
            intake.stop(),
            intakePivot.stow(),
            conveyor.stop(),
            Commands.runOnce(() -> shooter.stop()),
            Commands.runOnce(() -> climb.stopMotor()))
        .withName("SuperstructureEmergencyStop");
  }

  /** Check if ready to shoot */
  public boolean readyToShoot() {
    return currentState == SuperstructureState.AIMING && shooter.atVelocity(60.0); // Adjust target
  }
}
