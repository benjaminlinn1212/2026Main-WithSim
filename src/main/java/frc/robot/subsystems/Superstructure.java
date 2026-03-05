package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.auto.dashboard.AutoTuning;
import frc.robot.auto.dashboard.FieldConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.MechanismVisualizer;
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
    AIMING_WHILE_INTAKING,
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
  private final MechanismVisualizer mechanismViz = new MechanismVisualizer();

  private SuperstructureState currentState = SuperstructureState.IDLE;
  private SuperstructureState wantedState = SuperstructureState.IDLE;

  /**
   * Whether feeding is active (conveyor + indexer push FUEL to the shooter). When true and the
   * superstructure is in an aiming state (ONLY_AIMING or AIMING_WHILE_INTAKING), {@code periodic()}
   * also runs the conveyor and indexer. Set/cleared via {@link #setFeedingRequested(boolean)} in
   * teleop (RT held) or directly in auto code.
   */
  private boolean feedingRequested = false;

  /**
   * Whether the intake pivot should use the outpost position instead of full/half deploy. Auto-only
   * flag — set when intaking at the OUTPOST human player CHUTE. When true, {@link
   * #applyIntakePivotDeploy()} uses the outpost position and {@link #applyIntakeRollers()} stops
   * the rollers (FUEL is gravity-fed from the CHUTE). Other subsystems (turret, hood, shooter,
   * conveyor, indexer) continue operating normally per the current state. Cleared on {@link
   * #forceIdleState()}.
   */
  private boolean intakeOutpostMode = false;

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

  // ==================== Shooter Detection State ====================
  // Centralized current-based detection for shot completion. Runs every periodic() cycle
  // so both auto commands and teleop logic can simply read boolean getters.

  /**
   * Whether the shooter has finished firing all FUEL. True when the conveyor current stays below
   * {@link AutoTuning#CONVEYOR_NO_FUEL_CURRENT_THRESHOLD_AMPS} for {@link
   * AutoTuning#SHOOTER_DONE_TIME_SECONDS}. Reset via {@link #resetShooterDetection()}.
   */
  private boolean shooterFinishedFiring = false;

  /**
   * Timestamp when the conveyor current last dropped below the no-fuel threshold. Used to enforce
   * the sustained low-current duration before declaring shooter done.
   */
  private double conveyorLowCurrentStart = 0.0;

  /**
   * Whether the conveyor has ever seen high current since the last reset. This prevents falsely
   * detecting "shooter done" before the conveyor has even started feeding (motor accel time).
   */
  private boolean conveyorEverSawFuel = false;

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
  //  State                    | Turret | Hood   | Shooter | IntakePivot | Intake  | Conveyor |
  // Indexer
  //
  // -------------------------|--------|--------|---------|-------------|---------|----------|--------
  //  IDLE                     | stow   | stow   | stop    | stow        | stop   | stop     | stop
  //  ONLY_INTAKE              | stow   | stow   | stop    | deploy*     | intake* | stop     | stop
  //  ONLY_AIMING              | aim    | aim    | spinUp  | stow        | stop   | †        | †
  //  AIMING_WHILE_INTAKING    | aim    | aim    | spinUp  | deploy*     | intake* | †        | †
  //
  //  † Conveyor/Indexer: feed when feedingRequested=true, stop when false.
  //
  //  * IntakePivot/Intake vary by flags: intakeOutpostMode → outpost pos + stop,
  //    shouldJiggle (feeding in AIMING_WHILE_INTAKING) → jiggle + lower roller only,
  //    else → full deploy + both rollers.

  @Override
  public void periodic() {
    // Update current state to match wanted
    if (currentState != wantedState) {
      Logger.recordOutput("Superstructure/StateTransition", currentState + " -> " + wantedState);
      currentState = wantedState;
    }

    Logger.recordOutput("Superstructure/FeedingRequested", feedingRequested);
    Logger.recordOutput("Superstructure/State", currentState.toString());

    // ===== Trench Zone Detection (with exit hysteresis) =====
    // Compute trench zone FIRST so the main switch can stow the turret/hood as needed.
    // Exit hysteresis prevents turret stutter from pose noise at the trench boundary.
    // Skip trench computation for IDLE and EMERGENCY — turret/hood are stowed anyway.
    double now = Timer.getFPGATimestamp();
    boolean trenchStowHood = false;
    if (wantedState != SuperstructureState.IDLE && wantedState != SuperstructureState.EMERGENCY) {
      Pose2d robotPose = robotPoseSupplier.get();
      boolean geometricTrench =
          FieldConstants.isNearTrench(
              robotPose.getTranslation(), Constants.DriveConstants.TrenchAssist.HOOD_STOW_BUFFER);

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

      // When in the trench zone, stow the hood (trench is 22.25in tall)
      trenchStowHood = inTrenchZone;
    }

    // ===== Shooter Detection (current-based, gated by feeding) =====
    if (feedingRequested
        && (wantedState == SuperstructureState.ONLY_AIMING
            || wantedState == SuperstructureState.AIMING_WHILE_INTAKING)) {
      updateShooterDetection(now);
    }

    // Determine whether conveyor/indexer should feed this cycle
    boolean shouldFeed =
        feedingRequested
            && (wantedState == SuperstructureState.ONLY_AIMING
                || wantedState == SuperstructureState.AIMING_WHILE_INTAKING);

    // Determine whether the intake pivot should jiggle this cycle.
    // Jiggle only activates during AUTO stop-and-shoot (AIMING_WHILE_INTAKING + feeding) —
    // oscillating the pivot dislodges FUEL stuck at the intake/conveyor boundary.
    // In teleop, normal deploy is used so the driver doesn't see unexpected pivot motion.
    boolean shouldJiggle =
        shouldFeed
            && wantedState == SuperstructureState.AIMING_WHILE_INTAKING
            && DriverStation.isAutonomous();

    // Apply the wanted state to all subsystems every cycle
    switch (wantedState) {
      case IDLE:
        turret.applyStow();
        hood.applyStow();
        shooter.stopMotor();
        conveyor.stopMotor();
        indexer.stopMotor();
        intakePivot.applyStow();
        intake.stopMotor();
        break;

      case ONLY_INTAKE:
        turret.applyStow();
        hood.applyStow();
        shooter.stopMotor();
        conveyor.stopMotor();
        indexer.stopMotor();
        applyIntakePivotDeploy(shouldJiggle);
        applyIntakeRollers(shouldJiggle);
        break;

      case ONLY_AIMING:
        turret.applyAiming();
        if (trenchStowHood) {
          hood.applyStow();
        } else {
          hood.applyAiming();
        }
        shooter.applySpinUp();
        if (shouldFeed) {
          conveyor.applyFeedToShooter();
          indexer.applyFeedToShooter();
        } else {
          conveyor.stopMotor();
          indexer.stopMotor();
        }
        intakePivot.applyStow();
        intake.stopMotor();
        break;

      case AIMING_WHILE_INTAKING:
        turret.applyAiming();
        if (trenchStowHood) {
          hood.applyStow();
        } else {
          hood.applyAiming();
        }
        shooter.applySpinUp();
        if (shouldFeed) {
          conveyor.applyFeedToShooter();
          indexer.applyFeedToShooter();
        } else {
          conveyor.stopMotor();
          indexer.stopMotor();
        }
        applyIntakePivotDeploy(shouldJiggle);
        applyIntakeRollers(shouldJiggle);
        break;

      case EMERGENCY:
        // Emergency is entered via emergencyStop() and doesn't continuously re-apply.
        break;
    }

    // --- 3D Mechanism Visualization (AdvantageScope component poses) ---
    mechanismViz.update(
        turret.getCurrentPosition(), // turret yaw (rad)
        hood.getCurrentPosition(), // hood pitch (rad from horizontal)
        intakePivot.getPosition()); // intake pivot (motor rotations)
  }

  public SuperstructureState getState() {
    return currentState;
  }

  /** Whether the robot is currently in or approaching a trench zone. */
  public boolean isInTrenchZone() {
    return inTrenchZone;
  }

  /**
   * Whether the hood is at its current setpoint (within tolerance). Used by auto commands to gate
   * feeding — the robot should only feed FUEL when the hood has reached its aiming position, not
   * while it's still deploying after leaving the trench.
   */
  public boolean isHoodAtSetpoint() {
    return hood.atSetpoint();
  }

  /** Wire the robot pose supplier so Superstructure can detect trench proximity. */
  public void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
    this.robotPoseSupplier = supplier;
  }

  // ==================== Intake Pivot Deploy Helper ====================

  /**
   * Apply the correct intake pivot position. Priority (highest → lowest):
   *
   * <ol>
   *   <li>{@link #intakeOutpostMode} → outpost position (auto OUTPOST CHUTE)
   *   <li>{@code shouldJiggle} → jiggle between two positions to dislodge stuck FUEL (only when
   *       actively feeding in AIMING_WHILE_INTAKING)
   *   <li>Default → full deploy
   * </ol>
   */
  private void applyIntakePivotDeploy(boolean shouldJiggle) {
    if (intakeOutpostMode) {
      intakePivot.applyOutpostDeploy();
    } else if (shouldJiggle) {
      intakePivot.applyJiggle();
    } else {
      intakePivot.applyDeploy();
    }
  }

  /**
   * Apply the correct intake roller output. Same priority as {@link
   * #applyIntakePivotDeploy(boolean)}:
   *
   * <ol>
   *   <li>{@link #intakeOutpostMode} → stop all rollers (FUEL gravity-fed from CHUTE)
   *   <li>{@code shouldJiggle} → lower roller only (jiggling)
   *   <li>Pivot still deploying → upper roller forward, lower roller reverse (prevent FUEL fallout)
   *   <li>Default → both rollers forward
   * </ol>
   */
  private void applyIntakeRollers(boolean shouldJiggle) {
    if (intakeOutpostMode) {
      intake.stopMotor();
    } else if (shouldJiggle) {
      intake.applyIntakeLowerOnly();
    } else if (!intakePivot.isDeployed()) {
      intake.applyDeployReverse();
    } else {
      intake.applyIntake();
    }
  }

  /** Whether the intake is in an intaking/deployed state (for gating controls). */
  public boolean isIntakeDeployed() {
    return isIntakeDeployingState(currentState);
  }

  /** Check if a given state deploys the intake pivot. */
  private static boolean isIntakeDeployingState(SuperstructureState state) {
    return state == SuperstructureState.ONLY_INTAKE
        || state == SuperstructureState.AIMING_WHILE_INTAKING;
  }

  /**
   * Set whether the driver is requesting feeding (right trigger held). When enabled and the
   * superstructure is in an aiming state (ONLY_AIMING or AIMING_WHILE_INTAKING), the conveyor and
   * indexer will feed FUEL to the shooter. Also resets shooter detection when enabling so
   * current-spike detection starts fresh.
   *
   * @param requested true while the driver holds the feed trigger
   */
  public void setFeedingRequested(boolean requested) {
    if (requested && !this.feedingRequested) {
      resetShooterDetection();
    }
    this.feedingRequested = requested;
    Logger.recordOutput("Superstructure/FeedingRequested", requested);
  }

  /** Get whether the driver is currently requesting feeding. */
  public boolean isFeedingRequested() {
    return feedingRequested;
  }

  /**
   * Enable or disable outpost intake mode. When enabled, any intake-deploying state uses the
   * outpost pivot position and stops the rollers (FUEL is gravity-fed from the human player CHUTE).
   * All other subsystems continue operating per the current state. Auto-only — set when approaching
   * the OUTPOST, cleared when leaving.
   *
   * @param enabled true to use outpost pivot position and stop rollers
   */
  public void setIntakeOutpostMode(boolean enabled) {
    this.intakeOutpostMode = enabled;
    Logger.recordOutput("Superstructure/IntakeOutpostMode", enabled);
  }

  /** Whether the intake is in outpost mode (outpost pivot position, rollers stopped). */
  public boolean isIntakeOutpostMode() {
    return intakeOutpostMode;
  }

  // ==================== Release from Auto Climb ====================

  /**
   * Release from auto climb L1. After auto ends with the robot latched on L1 (RETRACT_L1_AUTO),
   * this reverses the retract path back to the extended position (EXTEND_L1_AUTO). Use POV Down
   * (stowClimb) afterward to stow the arm back to STOWED.
   */
  public Command releaseFromAutoClimbL1() {
    return Commands.sequence(
            climb.releaseFromAutoL1(),
            Commands.runOnce(
                () ->
                    Logger.recordOutput(
                        "Climb/OperatorAction", "Released from auto climb L1 → EXTEND_L1_AUTO")))
        .withName("ReleaseFromAutoClimbL1");
  }

  /**
   * Stow the climb arms back to STOWED by following reverse paths from the current state. Handles
   * both teleop states (EXTEND_L1 → STOWED, RETRACT_L1 → EXTEND_L1 → STOWED, etc.) and auto states
   * (RETRACT_L1_AUTO → EXTEND_L1_AUTO → STOWED). No teleporting — the arm follows smooth paths the
   * entire way.
   */
  public Command stowClimb() {
    return climb.stowFromCurrentState().withName("StowClimb");
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
    // Guard against transitioning out of EMERGENCY
    // (must be exited through its specific command)
    if (state != SuperstructureState.EMERGENCY) {
      return Commands.either(
          Commands.runOnce(
                  () -> {
                    wantedState = state;
                  })
              .withName("SetState_" + state),
          Commands.print(
              "[Superstructure] BLOCKED: Cannot set "
                  + state
                  + " while in "
                  + currentState
                  + " mode."),
          () -> currentState != SuperstructureState.EMERGENCY);
    }
    return Commands.runOnce(() -> wantedState = state).withName("SetState_" + state);
  }

  // ===== Convenience Aliases (for readability in auto/teleop) =====

  /** Set state to IDLE (everything stowed and stopped). Also clears feeding flag. Instant. */
  public Command idle() {
    return Commands.runOnce(
            () -> {
              feedingRequested = false;
              wantedState = SuperstructureState.IDLE;
            })
        .withName("SetState_IDLE");
  }

  /** Set state to ONLY_INTAKE (deploy intake, everything else stowed). Instant. */
  public Command onlyIntake() {
    return setWantedState(SuperstructureState.ONLY_INTAKE);
  }

  /** Set state to ONLY_AIMING (turret/hood/shooter aim, intake stowed). Instant. */
  public Command onlyAiming() {
    return setWantedState(SuperstructureState.ONLY_AIMING);
  }

  /** Set state to AIMING_WHILE_INTAKING (intake + aim simultaneously). Instant. */
  public Command aimingWhileIntaking() {
    return setWantedState(SuperstructureState.AIMING_WHILE_INTAKING);
  }

  // ==================== Climb (Independent Subsystem) ====================
  // Climb operates independently from superstructure — RobotContainer binds directly to
  // climb.nextClimbStep(), climb.previousState(), etc.
  // Superstructure continues running (intaking, aiming, etc.) in parallel.

  /**
   * Force the superstructure to IDLE, bypassing the emergency guard. Call this at the start of
   * autonomous to ensure a clean slate regardless of what state a previous auto run left the robot
   * in.
   */
  public void forceIdleState() {
    Logger.recordOutput("Superstructure/StateTransition", currentState + " -> IDLE (forced)");
    this.wantedState = SuperstructureState.IDLE;
    this.currentState = SuperstructureState.IDLE;
    this.feedingRequested = false;
    this.intakeOutpostMode = false;
  }

  /**
   * Directly set the wanted state from a non-command context (e.g. inside a {@code Commands.run()}
   * lambda that needs to switch states every cycle based on sensor data).
   *
   * <p>This bypasses the emergency guard — callers must not use this while in EMERGENCY. Prefer the
   * Command-returning {@link #setWantedState(SuperstructureState)} API for one-shot state changes
   * in sequences. Use this only for continuous polling loops.
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

  /** Emergency stop - immediately stow everything and stop all motors including climb. */
  public Command emergencyStop() {
    return Commands.runOnce(
            () -> {
              wantedState = SuperstructureState.EMERGENCY;
              currentState = SuperstructureState.EMERGENCY;
              feedingRequested = false;
              // Directly stop all motors immediately — don't wait for periodic
              turret.applyStow();
              hood.applyStow();
              shooter.stopMotor();
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
    return currentState != SuperstructureState.EMERGENCY;
  }

  /** Get a 0-1 readiness value for the shooter (for dashboard/LED use). */
  public double getShooterReadiness() {
    return shooter.isReady() ? 1.0 : 0.5;
  }

  // ==================== Shooter Detection Logic ====================
  // Centralized current-based detection that runs every periodic() cycle.
  // Auto commands and teleop logic just read the boolean getters.

  /**
   * Update the shooter completion detection. Logic: the conveyor motor draws more current when FUEL
   * is being pushed into the shooter. If the conveyor current drops below the threshold and stays
   * low for the configured duration, all FUEL has been fired. Motor accel time is accounted for by
   * requiring the current to have been high at least once before starting the low-current timer.
   */
  private void updateShooterDetection(double now) {
    double conveyorCurrent = conveyor.getCurrentAmps();

    // Track whether we've ever seen fuel (current above threshold) since last reset
    if (conveyorCurrent >= AutoTuning.CONVEYOR_NO_FUEL_CURRENT_THRESHOLD_AMPS) {
      conveyorEverSawFuel = true;
      conveyorLowCurrentStart = now; // Reset the low-current timer
      shooterFinishedFiring = false;
    }

    // Only check for "done" after we've seen fuel at least once (motor accel time)
    if (conveyorEverSawFuel
        && conveyorCurrent < AutoTuning.CONVEYOR_NO_FUEL_CURRENT_THRESHOLD_AMPS) {
      if (now - conveyorLowCurrentStart >= AutoTuning.SHOOTER_DONE_TIME_SECONDS) {
        shooterFinishedFiring = true;
      }
    }

    Logger.recordOutput("Superstructure/ShooterDetect/ConveyorCurrentAmps", conveyorCurrent);
    Logger.recordOutput("Superstructure/ShooterDetect/Finished", shooterFinishedFiring);
    Logger.recordOutput("Superstructure/ShooterDetect/EverSawFuel", conveyorEverSawFuel);
    Logger.recordOutput("Superstructure/ShooterDetect/LowDuration", now - conveyorLowCurrentStart);
  }

  /**
   * Whether the shooter has finished firing all FUEL. Updated every periodic() cycle by monitoring
   * the conveyor motor current. Becomes true when conveyor current stays below threshold for 0.5s
   * after having been above (all FUEL has exited).
   *
   * <p>Auto commands use this to detect shot completion. Call {@link #resetShooterDetection()}
   * before starting a new shooting sequence.
   */
  public boolean isShooterFinishedFiring() {
    return shooterFinishedFiring;
  }

  /**
   * Reset the shooter completion detection state. Call this before starting a new shooting sequence
   * so the detection starts fresh.
   */
  public void resetShooterDetection() {
    shooterFinishedFiring = false;
    conveyorEverSawFuel = false;
    conveyorLowCurrentStart = Timer.getFPGATimestamp();
    Logger.recordOutput("Superstructure/ShooterDetect/Reset", true);
  }
}
