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
    CLIMB_EXTENDING, // Deprecated - use CLIMB_MODE with ClimbState
    CLIMB_RETRACTING, // Deprecated - use CLIMB_MODE with ClimbState
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

  public Command idle() {
    return Commands.parallel(
            Commands.runOnce(() -> setState(SuperstructureState.IDLE)),
            turret.stow(),
            hood.stow(),
            shooter.stopShooter(),
            intake.stop(),
            intakePivot.stow(),
            conveyor.stop(),
            indexer.stop())
        .withName("Superstructure_Idle");
  }

  public Command intakeFromGround() {
    return Commands.parallel(
            Commands.runOnce(() -> setState(SuperstructureState.INTAKING)),
            intakePivot.deploy(),
            intake.intake(),
            conveyor.stop(),
            indexer.stop())
        .withName("Superstructure_IntakeGround");
  }

  public Command aimHubFromAllianceZone() {
    return Commands.parallel(
            Commands.runOnce(() -> setState(SuperstructureState.AIMING_HUB_FROM_ALLIANCE_ZONE)),
            turret.aiming(),
            hood.aimHub(),
            shooter.spinUp(),
            intakePivot.stow(),
            intake.stop(),
            conveyor.stop(),
            indexer.stop())
        .withName("Superstructure_AimHubAllianceZone");
  }

  public Command scoreHubFromAllianceZone() {
    return Commands.sequence(
            Commands.runOnce(() -> setState(SuperstructureState.SCORING_HUB_FROM_ALLIANCE_ZONE)),
            // Commands.waitUntil(shooter::readyForHub),
            Commands.parallel(conveyor.goToShooter(), indexer.toShooter()))
        .withName("Superstructure_ScoreHubAllianceZone");
  }

  public Command intakeWhileAimingForPass() {
    return Commands.parallel(
            Commands.runOnce(() -> setState(SuperstructureState.INTAKE_WHILE_AIMING_FOR_PASS)),
            intakePivot.deploy(),
            intake.intake(),
            turret.aiming(),
            hood.aimHub(),
            shooter.spinUp())
        .withName("Superstructure_IntakeWhileAimingForPass");
  }

  public Command intakeWhilePassing() {
    return Commands.sequence(
            Commands.runOnce(() -> setState(SuperstructureState.INTAKE_WHILE_PASSING)),
            Commands.waitUntil(shooter::isReady),
            Commands.parallel(
                intakePivot.deploy(), intake.intake(), conveyor.goToShooter(), indexer.toShooter()))
        .withName("Superstructure_IntakeWhilePassing");
  }

  public Command intakeWhileAimingHub() {
    return Commands.parallel(
            Commands.runOnce(() -> setState(SuperstructureState.INTAKE_WHILE_AIMING_HUB)),
            intakePivot.deploy(),
            intake.intake(),
            indexer.stop(),
            conveyor.stop(),
            turret.aiming(),
            hood.aimHub(),
            shooter.spinUp())
        .withName("Superstructure_IntakeWhileAimingHub");
  }

  public Command intakeWhileScoringHub() {
    return Commands.sequence(
            Commands.runOnce(() -> setState(SuperstructureState.INTAKE_WHILE_SCORING_HUB)),
            // Commands.waitUntil(shooter::readyForHub),
            Commands.parallel(
                intakePivot.deploy(),
                intake.intake(),
                conveyor.goToShooter(),
                turret.aiming(),
                hood.aimHub(),
                shooter.spinUp(),
                indexer.toShooter()))
        .withName("Superstructure_IntakeWhileScoringHub");
  }

  public Command eject() {
    return Commands.parallel(intakePivot.deploy(), intake.outtake(), conveyor.goToBucket())
        .finallyDo(
            () -> {
              setState(SuperstructureState.IDLE);
            })
        .withName("Superstructure_Eject");
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
            Commands.parallel(
                Commands.runOnce(() -> setState(SuperstructureState.CLIMB_MODE)),
                turret.stow(),
                hood.stow(),
                shooter.stopShooter(),
                intakePivot.stow(),
                intake.stop(),
                conveyor.stop(),
                indexer.stop()),
            Commands.waitSeconds(0.2), // Brief delay to ensure subsystems are stowed
            climb.releaseHooksCommand(), // Release passive hooks
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
   * Advance to next climb state. Only works if already in climb mode.
   *
   * @return Command to advance climb state
   */
  public Command nextClimbState() {
    return Commands.either(
            climb.nextState(),
            Commands.print("ERROR: Not in climb mode! Press button to enter climb mode first."),
            this::isInClimbMode)
        .withName("Superstructure_NextClimbState");
  }

  /**
   * Go back to previous climb state. Only works if already in climb mode.
   *
   * @return Command to go to previous climb state
   */
  public Command previousClimbState() {
    return Commands.either(
            climb.previousState(), Commands.print("ERROR: Not in climb mode!"), this::isInClimbMode)
        .withName("Superstructure_PreviousClimbState");
  }

  /**
   * Get current climb subsystem for direct access if needed. Prefer using superstructure commands
   * for proper state management.
   *
   * @return ClimbSubsystem reference
   */
  public ClimbSubsystem getClimbSubsystem() {
    return climb;
  }

  // ==================== Deprecated Climb Methods ====================
  // Use enterClimbMode() + nextClimbState() instead

  /**
   * @deprecated Use enterClimbMode() instead
   */
  @Deprecated
  public Command prepareClimb() {
    return enterClimbMode();
  }

  /**
   * @deprecated Use climb.setStateCommand(ClimbState.REACH_L1) instead
   */
  @Deprecated
  public Command extendClimb() {
    return Commands.sequence(
            Commands.runOnce(() -> setState(SuperstructureState.CLIMB_EXTENDING)),
            climb.setStateCommand(ClimbState.REACH_L1))
        .withName("Superstructure_ExtendClimb");
  }

  /**
   * @deprecated Use climb.setStateCommand(ClimbState.PULL_L1) instead
   */
  @Deprecated
  public Command retractClimb() {
    return Commands.sequence(
            Commands.runOnce(() -> setState(SuperstructureState.CLIMB_RETRACTING)),
            climb.setStateCommand(ClimbState.PULL_L1))
        .withName("Superstructure_RetractClimb");
  }

  /**
   * @deprecated Use enterClimbMode() + nextClimbState() sequence instead
   */
  @Deprecated
  public Command fullClimbSequence() {
    return Commands.sequence(
            prepareClimb(),
            extendClimb(),
            Commands.waitSeconds(1.0), // Give time to position under bar
            retractClimb())
        .withName("Superstructure_FullClimb");
  }

  public Command enableEmergencyOverride() {
    return Commands.runOnce(
        () -> {
          setState(SuperstructureState.EMERGENCY);
          Logger.recordOutput("Superstructure/Warning", "EMERGENCY OVERRIDE ENABLED");
        });
  }

  public Command emergencyStop() {
    return Commands.parallel(
            Commands.runOnce(() -> setState(SuperstructureState.IDLE)),
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

  public boolean canIntake() {
    return currentState != SuperstructureState.CLIMB_EXTENDING
        && currentState != SuperstructureState.CLIMB_RETRACTING;
  }

  public double getShooterReadiness() {
    return shooter.isReady() ? 1.0 : 0.5;
  }
}
