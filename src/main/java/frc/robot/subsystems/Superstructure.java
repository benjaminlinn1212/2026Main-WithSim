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

public class Superstructure extends SubsystemBase {

  public enum SuperstructureState {
    IDLE,
    INTAKING,
    AIMING_HUB_FROM_ALLIANCE_ZONE,
    SCORING_HUB_FROM_ALLIANCE_ZONE,
    INTAKE_WHILE_AIMING_FOR_PASS,
    INTAKE_WHILE_PASSING,
    INTAKE_WHILE_AIMING_HUB,
    INTAKE_WHILE_SCORING_HUB,
    CLIMB_MODE, // Superstructure in climb mode - all other subsystems safe
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

  public Command idle() {
    return guardState(
        Commands.parallel(
                Commands.runOnce(() -> setState(SuperstructureState.IDLE)),
                turret.stow(),
                hood.stow(),
                shooter.stopShooter(),
                intake.stop(),
                intakePivot.stow(),
                conveyor.stop(),
                indexer.stop())
            .withName("Superstructure_Idle"),
        "idle");
  }

  public Command intakeFromGround() {
    return guardState(
        Commands.parallel(
                Commands.runOnce(() -> setState(SuperstructureState.INTAKING)),
                intakePivot.deploy(),
                intake.intake(),
                conveyor.stop(),
                indexer.stop())
            .withName("Superstructure_IntakeGround"),
        "intakeFromGround");
  }

  public Command aimHubFromAllianceZone() {
    return guardState(
        Commands.parallel(
                Commands.runOnce(() -> setState(SuperstructureState.AIMING_HUB_FROM_ALLIANCE_ZONE)),
                turret.aiming(),
                hood.aimHub(),
                shooter.spinUp(),
                intakePivot.stow(),
                intake.stop(),
                conveyor.stop(),
                indexer.stop())
            .withName("Superstructure_AimHubAllianceZone"),
        "aimHubFromAllianceZone");
  }

  public Command scoreHubFromAllianceZone() {
    return guardState(
        Commands.sequence(
                Commands.runOnce(
                    () -> setState(SuperstructureState.SCORING_HUB_FROM_ALLIANCE_ZONE)),
                Commands.waitUntil(shooter::isReady),
                Commands.parallel(conveyor.goToShooter(), indexer.toShooter()))
            .withName("Superstructure_ScoreHubAllianceZone"),
        "scoreHubFromAllianceZone");
  }

  public Command intakeWhileAimingForPass() {
    return guardState(
        Commands.parallel(
                Commands.runOnce(() -> setState(SuperstructureState.INTAKE_WHILE_AIMING_FOR_PASS)),
                intakePivot.deploy(),
                intake.intake(),
                turret.aiming(),
                hood.aimHub(),
                shooter.spinUp())
            .withName("Superstructure_IntakeWhileAimingForPass"),
        "intakeWhileAimingForPass");
  }

  public Command intakeWhilePassing() {
    return guardState(
        Commands.sequence(
                Commands.runOnce(() -> setState(SuperstructureState.INTAKE_WHILE_PASSING)),
                Commands.waitUntil(shooter::isReady),
                Commands.parallel(
                    intakePivot.deploy(),
                    intake.intake(),
                    conveyor.goToShooter(),
                    indexer.toShooter()))
            .withName("Superstructure_IntakeWhilePassing"),
        "intakeWhilePassing");
  }

  public Command intakeWhileAimingHub() {
    return guardState(
        Commands.parallel(
                Commands.runOnce(() -> setState(SuperstructureState.INTAKE_WHILE_AIMING_HUB)),
                intakePivot.deploy(),
                intake.intake(),
                indexer.stop(),
                conveyor.stop(),
                turret.aiming(),
                hood.aimHub(),
                shooter.spinUp())
            .withName("Superstructure_IntakeWhileAimingHub"),
        "intakeWhileAimingHub");
  }

  public Command intakeWhileScoringHub() {
    return guardState(
        Commands.sequence(
                Commands.runOnce(() -> setState(SuperstructureState.INTAKE_WHILE_SCORING_HUB)),
                Commands.waitUntil(shooter::isReady),
                Commands.parallel(
                    intakePivot.deploy(),
                    intake.intake(),
                    conveyor.goToShooter(),
                    turret.aiming(),
                    hood.aimHub(),
                    shooter.spinUp(),
                    indexer.toShooter()))
            .withName("Superstructure_IntakeWhileScoringHub"),
        "intakeWhileScoringHub");
  }

  public Command eject() {
    return guardState(
        Commands.parallel(intakePivot.deploy(), intake.outtake(), conveyor.goToBucket())
            .finallyDo(
                () -> {
                  setState(SuperstructureState.IDLE);
                })
            .withName("Superstructure_Eject"),
        "eject");
  }

  // ==================== Climb Mode Management (254-Style) ====================

  /**
   * Enter climb mode - stow all superstructure components and prepare for climb. This is the entry
   * point for climb operations.
   *
   * @return Command to enter climb mode
   */
  public Command enterClimbMode() {
    return Commands.sequence(
            Commands.runOnce(() -> setState(SuperstructureState.CLIMB_MODE)),
            // Stow all subsystems using proper parallel composition (not .schedule())
            Commands.parallel(
                turret.stow(),
                hood.stow(),
                intakePivot.stow(),
                shooter.stopShooter(),
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
   * <p>Note: This immediately moves to STOWED using position control. For a safer path-following
   * retract, use POV Left (previousState) repeatedly to retrace the climb path in reverse.
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
        && turret.atSetpoint() // Turret at stow
        && hood.atSetpoint() // Hood at stow
        && shooter.atSetpoint() // Shooter stopped
        && intakePivot.isStowed() // Intake pivot stowed
        && intake.atSetpoint() // Intake stopped
        && conveyor.atSetpoint() // Conveyor stopped
        && indexer.atSetpoint() // Indexer stopped
        && climb.getState() == ClimbState.STOWED; // Climb at stowed state
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

  public Command emergencyStop() {
    return Commands.parallel(
            Commands.runOnce(() -> setState(SuperstructureState.EMERGENCY)),
            turret.stow(),
            hood.stow(),
            shooter.stopShooter(),
            intake.stop(),
            conveyor.stop(),
            indexer.stop(),
            Commands.runOnce(() -> climb.stopMotors()))
        .withName("Superstructure_EmergencyStop");
  }

  public boolean isReadyToShoot() {
    return shooter.isReady()
        && (currentState == SuperstructureState.AIMING_HUB_FROM_ALLIANCE_ZONE
            || currentState == SuperstructureState.INTAKE_WHILE_AIMING_FOR_PASS
            || currentState == SuperstructureState.INTAKE_WHILE_AIMING_HUB);
  }

  /** Check if the superstructure is in a state that allows intaking. */
  public boolean canIntake() {
    return currentState != SuperstructureState.CLIMB_MODE;
  }

  public double getShooterReadiness() {
    return shooter.isReady() ? 1.0 : 0.5;
  }
}
