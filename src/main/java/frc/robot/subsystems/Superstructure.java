package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.auto.dashboard.FieldConstants;
import frc.robot.subsystems.climb.ClimbState;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Superstructure coordinates all scoring/intake subsystems into coherent states using the 254-style
 * "wanted state" pattern.
 *
 * <p><b>Key architecture:</b> {@link #periodic()} runs every 20ms and continuously applies the
 * {@code wantedState} by calling direct void methods on each subsystem. This means:
 *
 * <ul>
 *   <li>State transitions are instant: {@link #setWantedState(SuperstructureState)} is a {@code
 *       runOnce} command that finishes immediately.
 *   <li>Closed-loop control (turret, hood, shooter) is maintained automatically — no need for
 *       never-ending {@code run()} commands in sequences.
 *   <li>Auto and teleop both use the same API — no deadline/withTimeout hacks needed.
 * </ul>
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
    EJECT,
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
  private SuperstructureState wantedState = SuperstructureState.IDLE;

  /**
   * Robot pose supplier — used to detect when the robot is near a TRENCH so we can override the
   * hood to stow (the trench is only 22.25in tall, hood must be down to fit).
   */
  private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

  /** Whether the robot is currently near/inside a trench (updated each periodic cycle). */
  private boolean inTrenchZone = false;

  /**
   * Hysteresis latch for trench exit. Once the robot enters the trench zone, we keep {@link
   * #inTrenchZone} true for at least {@link #TRENCH_EXIT_HOLDOFF_SECONDS} after the geometric check
   * returns false. This prevents turret stutter from boundary flicker (pose noise at the zone edge
   * causing rapid stow↔aim oscillations).
   */
  private double trenchExitTimestamp = 0.0;

  private static final double TRENCH_EXIT_HOLDOFF_SECONDS = 0.15;

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

  // ==================== Periodic — Continuously Apply Wanted State ====================
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
  //  EJECT                    | stow   | stow   | stop    | deploy      | outtake| reverse  | stop

  @Override
  public void periodic() {
    // Update current state to match wanted
    if (currentState != wantedState) {
      Logger.recordOutput("Superstructure/StateTransition", currentState + " -> " + wantedState);
      currentState = wantedState;
    }
    Logger.recordOutput("Superstructure/State", currentState.toString());

    // ===== Trench Zone Detection (with exit hysteresis) =====
    // Compute trench zone FIRST so the main switch can use it to avoid calling
    // turret.applyAiming() when the turret must stay stowed. This eliminates the
    // wasteful applyAiming() → applyStow() double-call each cycle, which caused
    // turret state flip-flopping inside the trench.
    //
    // Exit hysteresis: once inTrenchZone goes true, keep it latched for
    // TRENCH_EXIT_HOLDOFF_SECONDS after the geometric check goes false. This prevents
    // turret stutter from pose noise at the trench boundary causing rapid stow↔aim
    // oscillation on exit.
    Pose2d robotPose = robotPoseSupplier.get();
    boolean geometricTrench = FieldConstants.isNearTrench(robotPose.getTranslation());
    double now = Timer.getFPGATimestamp();

    if (geometricTrench) {
      // Inside trench — keep latched and reset the exit timer
      inTrenchZone = true;
      trenchExitTimestamp = now;
    } else if (inTrenchZone) {
      // Just left the geometric zone — hold the latch for the holdoff period
      if (now - trenchExitTimestamp >= TRENCH_EXIT_HOLDOFF_SECONDS) {
        inTrenchZone = false; // Holdoff expired, safe to release
      }
      // else: still within holdoff, keep inTrenchZone = true
    }
    Logger.recordOutput("Superstructure/InTrenchZone", inTrenchZone);

    // When in the trench zone, aiming states force turret/hood to stow instead of aiming.
    // The trench is only 22.25in (56.5cm) tall — hood MUST be stowed to fit through.
    boolean trenchStow =
        inTrenchZone
            && wantedState != SuperstructureState.IDLE
            && wantedState != SuperstructureState.CLIMB_MODE
            && wantedState != SuperstructureState.EMERGENCY;

    // Apply the wanted state to all subsystems every cycle.
    // CLIMB_MODE and EMERGENCY are handled specially — they don't continuously apply
    // because climb has its own state machine.
    switch (wantedState) {
      case IDLE:
        turret.applyStow();
        hood.applyStow();
        shooter.stop();
        intakePivot.applyStow();
        intake.stopMotor();
        conveyor.stopMotor();
        indexer.stopMotor();
        break;

      case ONLY_INTAKE:
        turret.applyStow();
        hood.applyStow();
        shooter.stop();
        intakePivot.applyDeploy();
        intake.applyIntake();
        conveyor.stopMotor();
        indexer.stopMotor();
        break;

      case ONLY_AIMING:
        if (trenchStow) {
          turret.applyStow();
          hood.applyStow();
        } else {
          turret.applyAiming();
          hood.applyAiming();
        }
        shooter.applySpinUp();
        intakePivot.applyStow();
        intake.stopMotor();
        conveyor.stopMotor();
        indexer.stopMotor();
        break;

      case ONLY_SHOOTING:
        if (trenchStow) {
          turret.applyStow();
          hood.applyStow();
        } else {
          turret.applyAiming();
          hood.applyAiming();
        }
        shooter.applySpinUp();
        intakePivot.applyStow();
        intake.stopMotor();
        conveyor.applyFeedToShooter();
        indexer.applyFeedToShooter();
        break;

      case AIMING_WHILE_INTAKING:
        if (trenchStow) {
          turret.applyStow();
          hood.applyStow();
        } else {
          turret.applyAiming();
          hood.applyAiming();
        }
        shooter.applySpinUp();
        intakePivot.applyDeploy();
        intake.applyIntake();
        conveyor.stopMotor();
        indexer.stopMotor();
        break;

      case SHOOTING_WHILE_INTAKING:
        if (trenchStow) {
          turret.applyStow();
          hood.applyStow();
        } else {
          turret.applyAiming();
          hood.applyAiming();
        }
        shooter.applySpinUp();
        intakePivot.applyDeploy();
        intake.applyIntake();
        conveyor.applyFeedToShooter();
        indexer.applyFeedToShooter();
        break;

      case EJECT:
        turret.applyStow();
        hood.applyStow();
        shooter.stop();
        intakePivot.applyDeploy();
        intake.applyOuttake();
        conveyor.applyFeedToBucket();
        indexer.stopMotor();
        break;

      case CLIMB_MODE:
      case EMERGENCY:
        // These states are entered via specific commands and don't continuously
        // re-apply — the climb subsystem manages its own state machine.
        break;
    }
  }

  public SuperstructureState getState() {
    return currentState;
  }

  /** Whether the robot is currently in or approaching a trench zone. */
  public boolean isInTrenchZone() {
    return inTrenchZone;
  }

  /** Wire the robot pose supplier so Superstructure can detect trench proximity. */
  public void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
    this.robotPoseSupplier = supplier;
  }

  // ==================== Instant State Commands (254-style) ====================
  // These commands finish immediately. The periodic() loop takes over and
  // continuously applies the new state to all subsystems.

  /**
   * Instantly set the superstructure to the given state and return. The {@link #periodic()} method
   * will continuously apply this state to all subsystems.
   *
   * <p>This is the primary API for both teleop and auto. In teleop, bind with {@code
   * button.onTrue(superstructure.setWantedState(ONLY_AIMING))}. In auto, use directly in sequences
   * — the command finishes instantly so the sequence continues.
   *
   * @param state The desired superstructure state
   * @return An instant command that sets the wanted state
   */
  public Command setWantedState(SuperstructureState state) {
    // Guard against transitioning out of CLIMB_MODE / EMERGENCY
    // (those must be exited through their specific commands)
    if (state != SuperstructureState.CLIMB_MODE && state != SuperstructureState.EMERGENCY) {
      return Commands.either(
          Commands.runOnce(() -> wantedState = state).withName("SetState_" + state),
          Commands.print(
              "[Superstructure] BLOCKED: Cannot set "
                  + state
                  + " while in "
                  + currentState
                  + " mode."),
          () ->
              currentState != SuperstructureState.CLIMB_MODE
                  && currentState != SuperstructureState.EMERGENCY);
    }
    return Commands.runOnce(() -> wantedState = state).withName("SetState_" + state);
  }

  // ===== Convenience Aliases (for readability in auto/teleop) =====

  /** Set state to IDLE (everything stowed and stopped). Instant. */
  public Command idle() {
    return setWantedState(SuperstructureState.IDLE);
  }

  /** Set state to ONLY_INTAKE (deploy intake, everything else stowed). Instant. */
  public Command onlyIntake() {
    return setWantedState(SuperstructureState.ONLY_INTAKE);
  }

  /** Set state to ONLY_AIMING (turret/hood/shooter aim, intake stowed). Instant. */
  public Command onlyAiming() {
    return setWantedState(SuperstructureState.ONLY_AIMING);
  }

  /** Set state to ONLY_SHOOTING (aim + feed game pieces). Instant. */
  public Command onlyShooting() {
    return setWantedState(SuperstructureState.ONLY_SHOOTING);
  }

  /** Set state to AIMING_WHILE_INTAKING (intake + aim simultaneously). Instant. */
  public Command aimingWhileIntaking() {
    return setWantedState(SuperstructureState.AIMING_WHILE_INTAKING);
  }

  /** Set state to SHOOTING_WHILE_INTAKING (intake + aim + feed simultaneously). Instant. */
  public Command shootingWhileIntaking() {
    return setWantedState(SuperstructureState.SHOOTING_WHILE_INTAKING);
  }

  /** Set state to EJECT (reverse intake, conveyor to bucket). Instant. */
  public Command eject() {
    return setWantedState(SuperstructureState.EJECT);
  }

  // ==================== Climb Mode Management ====================

  /**
   * Enter climb mode - stow all superstructure components and prepare for climb. Sets wanted state
   * to IDLE first (so periodic stows everything), waits for subsystems to reach stow, then
   * transitions to CLIMB_MODE.
   *
   * @return Command to enter climb mode
   */
  public Command enterClimbMode() {
    return Commands.sequence(
            // First idle to stow everything
            Commands.runOnce(() -> wantedState = SuperstructureState.IDLE),
            // Wait a beat for stow to take effect
            Commands.waitSeconds(0.5),
            // Now enter climb mode (periodic won't re-apply — CLIMB_MODE is a no-op)
            Commands.runOnce(
                () -> {
                  wantedState = SuperstructureState.CLIMB_MODE;
                  currentState = SuperstructureState.CLIMB_MODE;
                }),
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
            // Force back to IDLE (bypasses guard since we're setting wantedState directly)
            Commands.runOnce(
                () -> {
                  wantedState = SuperstructureState.IDLE;
                  currentState = SuperstructureState.IDLE;
                }),
            Commands.runOnce(
                () -> Logger.recordOutput("Superstructure/ClimbMode", "Climb mode deactivated")))
        .withName("Superstructure_ExitClimbMode");
  }

  /**
   * Force the superstructure to IDLE, bypassing the climb/emergency guard. Call this at the start
   * of autonomous to ensure a clean slate regardless of what state a previous auto run left the
   * robot in.
   */
  public void forceIdleState() {
    Logger.recordOutput("Superstructure/StateTransition", currentState + " -> IDLE (forced)");
    this.wantedState = SuperstructureState.IDLE;
    this.currentState = SuperstructureState.IDLE;
  }

  /**
   * Directly set the wanted state from a non-command context (e.g. inside a {@code Commands.run()}
   * lambda that needs to switch states every cycle based on sensor data).
   *
   * <p>This bypasses the climb/emergency guard — callers must not use this while in CLIMB_MODE or
   * EMERGENCY. Prefer the Command-returning {@link #setWantedState(SuperstructureState)} API for
   * one-shot state changes in sequences. Use this only for continuous polling loops.
   *
   * @param state The desired superstructure state
   */
  public void forceWantedState(SuperstructureState state) {
    this.wantedState = state;
  }

  /**
   * Check if the intake pivot has physically reached the stowed position. Used by auto to gate
   * climb extend — the arms must not extend while the intake is still deployed.
   *
   * @return true if intake pivot is at stow position (within tolerance)
   */
  public boolean isIntakeStowed() {
    return intakePivot.isStowed();
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
    return Commands.runOnce(
            () -> {
              wantedState = SuperstructureState.EMERGENCY;
              currentState = SuperstructureState.EMERGENCY;
              // Directly stop all motors immediately — don't wait for periodic
              turret.applyStow();
              hood.applyStow();
              shooter.stop();
              intakePivot.applyStow();
              intake.stopMotor();
              conveyor.stopMotor();
              indexer.stopMotor();
              climb.stopMotors();
            })
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

  /**
   * Get the stator current of the upper intake roller (amps). Pass-through to {@link
   * IntakeSubsystem#getUpperCurrentAmps()} for current-based FUEL pickup detection in auto.
   */
  public double getIntakeUpperCurrentAmps() {
    return intake.getUpperCurrentAmps();
  }

  /**
   * Get the stator current of the shooter motor (amps). Pass-through to {@link
   * ShooterSubsystem#getCurrentAmps()} for current-based shot completion detection in auto.
   */
  public double getShooterCurrentAmps() {
    return shooter.getCurrentAmps();
  }
}
