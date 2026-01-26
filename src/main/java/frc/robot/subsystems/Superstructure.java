package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  public enum SuperstructureState {
    IDLE,
    INTAKE,
    PREPARE_SHOOT,
    SHOOT,
    EMERGENCY
  }

  private final ShooterSubsystem shooter;
  private final HoodSubsystem hood;
  private final IntakeSubsystem intake;
  private final IntakePivotSubsystem intakePivot;
  private final ConveyorSubsystem conveyor;

  private SuperstructureState currentState = SuperstructureState.IDLE;

  public Superstructure(
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      IntakeSubsystem intake,
      IntakePivotSubsystem intakePivot,
      ConveyorSubsystem conveyor) {
    this.shooter = shooter;
    this.hood = hood;
    this.intake = intake;
    this.intakePivot = intakePivot;
    this.conveyor = conveyor;
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
            hood.stow(),
            shooter.stopShooter(),
            intake.stop(),
            intakePivot.stow(),
            conveyor.stop())
        .withName("Superstructure_Idle");
  }

  public Command intake() {
    return Commands.parallel(
            Commands.runOnce(() -> setState(SuperstructureState.INTAKE)),
            intakePivot.deploy(),
            intake.intake(),
            conveyor.stop(),
            shooter.stopShooter(),
            hood.stow())
        .withName("Superstructure_Intake");
  }

  public Command prepareShoot() {
    return Commands.parallel(
            Commands.runOnce(() -> setState(SuperstructureState.PREPARE_SHOOT)),
            intakePivot.stow(),
            intake.stop(),
            hood.stow(),
            shooter.spinUpForHub())
        .withName("Superstructure_PrepareShoot");
  }

  public Command shoot() {
    return Commands.sequence(
            Commands.runOnce(() -> setState(SuperstructureState.SHOOT)),
            Commands.waitUntil(shooter::readyForHub),
            conveyor.goToShooter())
        .withName("Superstructure_Shoot");
  }

  public Command eject() {
    return Commands.parallel(intakePivot.deploy(), intake.outtake(), conveyor.goToBucket())
        .finallyDo(
            () -> {
              setState(SuperstructureState.IDLE);
            })
        .withName("Superstructure_Eject");
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
            hood.stow(),
            shooter.stopShooter(),
            intake.stop(),
            conveyor.stop())
        .withName("Superstructure_EmergencyStop");
  }

  public boolean isReadyToShoot() {
    return shooter.readyForHub() && currentState == SuperstructureState.PREPARE_SHOOT;
  }

  public boolean canIntake() {
    return true; // No climb, so always can intake
  }

  public double getShooterReadiness() {
    return shooter.readyForHub() ? 1.0 : 0.5;
  }
}
