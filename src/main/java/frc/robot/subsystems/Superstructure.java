package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbState;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Superstructure coordinates all scoring/intake subsystems into coherent states. Every state
 * explicitly commands ALL subsystems so nothing is left in an undefined state.
 *
 * <p>Subsystems controlled: turret, hood, shooter, intakePivot, intake, conveyor, indexer, climb
 */
public class Superstructure extends SubsystemBase {

  public enum SuperstructureState {
    IDLE,
    ONLY_INTAKE,
    ONLY_AIMING,
    ONLY_SHOOTING,
    AIMING_WHILE_INTAKING,
    SHOOTING_WHILE_INTAKING,
    CLIMB_MODE,
    EMERGENCY
  }

  private final ShooterSubsystem shooter;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;
  private final IntakeSubsystem intake;
  private final IntakePivotSubsystem intakePivot;
  private final ConveyorSubsystem conveyor;
  private final IndexerSubsystem indexer;
  private final ClimbSubsystem climb;

  private SuperstructureState currentState = SuperstructureState.IDLE;

  public Superstructure(
      ShooterSubsystem shooter,
      TurretSubsystem turret,
      HoodSubsystem hood,
      IntakeSubsystem intake,
      IntakePivotSubsystem intakePivot,
      ConveyorSubsystem conveyor,
      IndexerSubsystem indexer,
      ClimbSubsystem climb) {
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;
    this.intake = intake;
    this.intakePivot = intakePivot;
    this.conveyor = conveyor;
    this.indexer = indexer;
    this.climb = climb;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure/State", currentState.toString());
  }

  public SuperstructureState getState() {
    return currentState;
  }

  private void setState(SuperstructureState state) {
    if (this.currentState != state) {
      Logger.recordOutput("Superstructure/StateTransition", currentState + " -> " + state);
      this.currentState = state;
    }
  }

  /**
   * Guard that prevents non-climb commands from running while in climb or emergency mode. Returns a
   * command that either runs the desired command or prints an error.
   */
  private Command guardState(Command desired, String name) {
    return Commands.either(
        desired,
        Commands.print(
            "[Superstructure] BLOCKED: Cannot run "
                + name
                + " while in "
                + currentState
                + " mode."),
        () ->
            currentState != SuperstructureState.CLIMB_MODE
                && currentState != SuperstructureState.EMERGENCY);
  }

  // ==================== Superstructure States ====================
  // Every state explicitly controls ALL subsystems.
  //
  //  State                    | Turret | Hood   | Shooter | IntakePivot | Intake | Conveyor |
  // Indexer
  //
  // -------------------------|--------|--------|---------|-------------|--------|----------|--------
  //  IDLE                     | stow   | stow   | stop    | stow        | stop   | stop     | stop
  //  ONLY_INTAKE              | stow   | stow   | stop    | deploy      | intake | stop     | stop
  //  ONLY_AIMING              | aim    | aim    | spinUp  | stow        | stop   | stop     | stop
  //  ONLY_SHOOTING            | aim    | aim    | spinUp  | stow        | stop   | feed     | feed
  //  AIMING_WHILE_INTAKING    | aim    | aim    | spinUp  | deploy      | intake | stop     | stop
  //  SHOOTING_WHILE_INTAKING  | aim    | aim    | spinUp  | deploy      | intake | feed     | feed

  /**
   * IDLE - everything stowed and stopped.
   *
   * <p>Turret: stow, Hood: stow, Shooter: stop, IntakePivot: stow, Intake: stop, Conveyor: stop,
   * Indexer: stop
   */
  public Command idle() {
    return guardState(
        Commands.parallel(
                Commands.runOnce(() -> setState(SuperstructureState.IDLE)),
                turret.stow(),
                hood.stow(),
                shooter.stopShooter(),
                intakePivot.stow(),
                intake.stop(),
                conveyor.stop(),
                indexer.stop())
            .withName("Superstructure_Idle"),
        "idle");
  }

  /**
   * ONLY_INTAKE - deploy intake, everything else stowed/stopped.
   *
   * <p>Turret: stow, Hood: stow, Shooter: stop, IntakePivot: deploy, Intake: intake, Conveyor:
   * stop, Indexer: stop
   */
  public Command onlyIntake() {
    return guardState(
        Commands.parallel(
                Commands.runOnce(() -> setState(SuperstructureState.ONLY_INTAKE)),
                turret.stow(),
                hood.stow(),
                shooter.stopShooter(),
                intakePivot.deploy(),
                intake.intake(),
                conveyor.stop(),
                indexer.stop())
            .withName("Superstructure_OnlyIntake"),
        "onlyIntake");
  }

  /**
   * ONLY_AIMING - turret/hood/shooter aim at target, intake stowed, no feeding.
   *
   * <p>Turret: aim, Hood: aim, Shooter: spinUp, IntakePivot: stow, Intake: stop, Conveyor: stop,
   * Indexer: stop
   */
  public Command onlyAiming() {
    return guardState(
        Commands.parallel(
                Commands.runOnce(() -> setState(SuperstructureState.ONLY_AIMING)),
                turret.aiming(),
                hood.aimHub(),
                shooter.spinUp(),
                intakePivot.stow(),
                intake.stop(),
                conveyor.stop(),
                indexer.stop())
            .withName("Superstructure_OnlyAiming"),
        "onlyAiming");
  }

  /**
   * ONLY_SHOOTING - aim at target and feed game pieces.
   *
   * <p>Turret: aim, Hood: aim, Shooter: spinUp, IntakePivot: stow, Intake: stop, Conveyor: feed,
   * Indexer: feed
   */
  public Command onlyShooting() {
    return guardState(
        Commands.parallel(
                Commands.runOnce(() -> setState(SuperstructureState.ONLY_SHOOTING)),
                turret.aiming(),
                hood.aimHub(),
                shooter.spinUp(),
                intakePivot.stow(),
                intake.stop(),
                conveyor.goToShooter(),
                indexer.toShooter())
            .withName("Superstructure_OnlyShooting"),
        "onlyShooting");
  }

  /**
   * AIMING_WHILE_INTAKING - intake deployed and running while turret/hood/shooter aim. No feeding
   * yet.
   *
   * <p>Turret: aim, Hood: aim, Shooter: spinUp, IntakePivot: deploy, Intake: intake, Conveyor:
   * stop, Indexer: stop
   */
  public Command aimingWhileIntaking() {
    return guardState(
        Commands.parallel(
                Commands.runOnce(() -> setState(SuperstructureState.AIMING_WHILE_INTAKING)),
                turret.aiming(),
                hood.aimHub(),
                shooter.spinUp(),
                intakePivot.deploy(),
                intake.intake(),
                conveyor.stop(),
                indexer.stop())
            .withName("Superstructure_AimingWhileIntaking"),
        "aimingWhileIntaking");
  }

  /**
   * SHOOTING_WHILE_INTAKING - intake deployed and running while aiming and feeding.
   *
   * <p>Turret: aim, Hood: aim, Shooter: spinUp, IntakePivot: deploy, Intake: intake, Conveyor:
   * feed, Indexer: feed
   */
  public Command shootingWhileIntaking() {
    return guardState(
        Commands.parallel(
                Commands.runOnce(() -> setState(SuperstructureState.SHOOTING_WHILE_INTAKING)),
                turret.aiming(),
                hood.aimHub(),
                shooter.spinUp(),
                intakePivot.deploy(),
                intake.intake(),
                conveyor.goToShooter(),
                indexer.toShooter())
            .withName("Superstructure_ShootingWhileIntaking"),
        "shootingWhileIntaking");
  }

  /**
   * EJECT - reverse intake to spit out game pieces. Returns to IDLE when finished.
   *
   * <p>Turret: stow, Hood: stow, Shooter: stop, IntakePivot: deploy, Intake: outtake, Conveyor:
   * reverse, Indexer: stop
   */
  public Command eject() {
    return guardState(
        Commands.parallel(
                Commands.runOnce(() -> setState(SuperstructureState.IDLE)),
                turret.stow(),
                hood.stow(),
                shooter.stopShooter(),
                intakePivot.deploy(),
                intake.outtake(),
                conveyor.goToBucket(),
                indexer.stop())
            .finallyDo(() -> setState(SuperstructureState.IDLE))
            .withName("Superstructure_Eject"),
        "eject");
  }

  // ==================== Climb Mode Management ====================

  /**
   * Enter climb mode - stow all superstructure components and prepare for climb. This is the entry
   * point for climb operations.
   *
   * @return Command to enter climb mode
   */
  public Command enterClimbMode() {
    return Commands.sequence(
            Commands.runOnce(() -> setState(SuperstructureState.CLIMB_MODE)),
            // Stow all subsystems
            Commands.parallel(
                turret.stow(),
                hood.stow(),
                shooter.stopShooter(),
                intakePivot.stow(),
                intake.stop(),
                conveyor.stop(),
                indexer.stop()),
            climb.releaseHooksCommand(),
            Commands.runOnce(
                () ->
                    Logger.recordOutput(
                        "Superstructure/ClimbMode", "CLIMB MODE ACTIVE - Hooks Released")))
        .withName("Superstructure_EnterClimbMode");
  }

  /**
   * Exit climb mode and return to idle.
   *
   * @return Command to exit climb mode
   */
  public Command exitClimbMode() {
    return Commands.sequence(
            climb.setStateCommand(ClimbState.STOWED),
            idle(),
            Commands.runOnce(
                () -> Logger.recordOutput("Superstructure/ClimbMode", "Climb mode deactivated")))
        .withName("Superstructure_ExitClimbMode");
  }

  /**
   * Check if superstructure is in climb mode.
   *
   * @return true if in climb mode
   */
  public boolean isInClimbMode() {
    return currentState == SuperstructureState.CLIMB_MODE;
  }

  /**
   * Check if all subsystems are stowed and ready for climb operations.
   *
   * @return true if all subsystems are at stow positions
   */
  public boolean isReadyForClimb() {
    return isInClimbMode()
        && turret.atSetpoint()
        && hood.atSetpoint()
        && shooter.atSetpoint()
        && intakePivot.isStowed()
        && intake.atSetpoint()
        && conveyor.atSetpoint()
        && indexer.atSetpoint()
        && climb.getState() == ClimbState.STOWED;
  }

  /**
   * Advance to next climb state. Only works if already in climb mode.
   *
   * @return Command to advance climb state
   */
  public Command nextClimbState() {
    Command cmd =
        Commands.either(
            climb.nextState(),
            Commands.print("ERROR: Not in climb mode! Press button to enter climb mode first."),
            this::isInClimbMode);
    cmd.setName("Superstructure_NextClimbState");
    return cmd;
  }

  /**
   * Go back to previous climb state. Only works if already in climb mode.
   *
   * @return Command to go to previous climb state
   */
  public Command previousClimbState() {
    Command cmd =
        Commands.either(
            climb.previousState(),
            Commands.print("ERROR: Not in climb mode!"),
            this::isInClimbMode);
    cmd.setName("Superstructure_PreviousClimbState");
    return cmd;
  }

  /** Emergency stop - immediately stow everything and stop all motors including climb. */
  public Command emergencyStop() {
    return Commands.parallel(
            Commands.runOnce(() -> setState(SuperstructureState.EMERGENCY)),
            turret.stow(),
            hood.stow(),
            shooter.stopShooter(),
            intakePivot.stow(),
            intake.stop(),
            conveyor.stop(),
            indexer.stop(),
            Commands.runOnce(() -> climb.stopMotors()))
        .withName("Superstructure_EmergencyStop");
  }

  // ==================== Status Queries ====================

  /** Check if the shooter is ready and we're in an aiming state. */
  public boolean isReadyToShoot() {
    return shooter.isReady()
        && (currentState == SuperstructureState.ONLY_AIMING
            || currentState == SuperstructureState.AIMING_WHILE_INTAKING);
  }

  /** Check if the superstructure is in a state that allows intaking. */
  public boolean canIntake() {
    return currentState != SuperstructureState.CLIMB_MODE
        && currentState != SuperstructureState.EMERGENCY;
  }

  /** Get a 0-1 readiness value for the shooter (for dashboard/LED use). */
  public double getShooterReadiness() {
    return shooter.isReady() ? 1.0 : 0.5;
  }

  // ==================== Legacy Compatibility ====================
  // These delegate to the new state names for backward compatibility with auto code.

  /**
   * @deprecated Use {@link #onlyIntake()} instead.
   */
  @Deprecated
  public Command intakeFromGround() {
    return onlyIntake();
  }

  /**
   * @deprecated Use {@link #onlyAiming()} instead.
   */
  @Deprecated
  public Command aimHubFromAllianceZone() {
    return onlyAiming();
  }

  /**
   * @deprecated Use {@link #onlyShooting()} instead.
   */
  @Deprecated
  public Command scoreHubFromAllianceZone() {
    return onlyShooting();
  }
}
