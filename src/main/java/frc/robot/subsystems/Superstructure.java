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
 * 254-style "wanted state" coordinator. {@link #periodic()} applies the wanted state to all
 * subsystems every 20ms via void calls. State transitions are instant commands.
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

  /** Whether feeding is active (conveyor+indexer push FUEL to shooter). */
  private boolean feedingRequested = false;

  /** Whether the driver is requesting intake roller eject (reverse rollers while deployed). */
  private boolean intakeEjectRequested = false;

  /** Use outpost pivot position instead of full deploy. Auto-only, cleared on forceIdleState(). */
  private boolean intakeOutpostMode = false;

  /** Jiggle between outpost positions to dislodge FUEL. Auto-only, cleared on forceIdleState(). */
  private boolean intakeOutpostJiggleMode = false;

  /** Robot pose supplier for trench proximity detection. */
  private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

  /** Whether the robot is currently near/inside a trench (updated each periodic cycle). */
  private boolean inTrenchZone = false;

  /**
   * Hysteresis latch for trench exit. Holds inTrenchZone true for a holdoff period after leaving
   * the geometric zone to prevent turret stutter from pose noise.
   */
  private double trenchExitTimestamp = 0.0;

  private static final double TRENCH_EXIT_HOLDOFF_SECONDS = 0.15;

  // ==================== Shooter Detection State ====================
  // Centralized current-based detection for shot completion. Runs every periodic() cycle
  // so both auto commands and teleop logic can simply read boolean getters.

  /** True when conveyor current stays low for the configured duration after feeding. */
  private boolean shooterFinishedFiring = false;

  /** Timestamp when conveyor current last dropped below the no-fuel threshold. */
  private double conveyorLowCurrentStart = 0.0;

  /**
   * Whether the conveyor has seen high current since last reset (prevents false "done" on startup).
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
    Logger.recordOutput("Superstructure/IntakeEjectRequested", intakeEjectRequested);
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

  /** Whether the hood is at its current setpoint (for gating feeding after trench exit). */
  public boolean isHoodAtSetpoint() {
    return hood.atSetpoint();
  }

  /** Wire the robot pose supplier so Superstructure can detect trench proximity. */
  public void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
    this.robotPoseSupplier = supplier;
  }

  // ==================== Intake Pivot Deploy Helper ====================

  /**
   * Apply correct intake pivot position. Priority: outpostJiggle > outpost > jiggle > full deploy.
   */
  private void applyIntakePivotDeploy(boolean shouldJiggle) {
    if (intakeOutpostJiggleMode) {
      intakePivot.applyOutpostJiggle();
    } else if (intakeOutpostMode) {
      intakePivot.applyOutpostDeploy();
    } else if (shouldJiggle) {
      intakePivot.applyJiggle();
    } else {
      intakePivot.applyDeploy();
    }
  }

  /**
   * Apply correct intake roller output. Priority: eject (driver LT) > outpost (stop) > jiggle
   * (lower only) > deploying (upper fwd, lower rev) > full intake.
   */
  private void applyIntakeRollers(boolean shouldJiggle) {
    if (intakeEjectRequested) {
      intake.applyEject();
    } else if (intakeOutpostMode) {
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

  /** Set whether the driver is requesting feeding. Resets shooter detection when enabling. */
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

  /** Set whether the driver is requesting intake roller eject (reverse while deployed). */
  public void setIntakeEjectRequested(boolean requested) {
    this.intakeEjectRequested = requested;
    Logger.recordOutput("Superstructure/IntakeEjectRequested", requested);
  }

  /** Enable/disable outpost intake mode (outpost pivot position, rollers stopped). Auto-only. */
  public void setIntakeOutpostMode(boolean enabled) {
    this.intakeOutpostMode = enabled;
    Logger.recordOutput("Superstructure/IntakeOutpostMode", enabled);
  }

  /** Whether the intake is in outpost mode (outpost pivot position, rollers stopped). */
  public boolean isIntakeOutpostMode() {
    return intakeOutpostMode;
  }

  /** Enable/disable outpost jiggle mode. Takes priority over normal outpost mode. Auto-only. */
  public void setIntakeOutpostJiggleMode(boolean enabled) {
    this.intakeOutpostJiggleMode = enabled;
    Logger.recordOutput("Superstructure/IntakeOutpostJiggleMode", enabled);
  }

  /** Whether the intake is in outpost jiggle mode. */
  public boolean isIntakeOutpostJiggleMode() {
    return intakeOutpostJiggleMode;
  }

  // ==================== Release from Auto Climb ====================

  /** Release from auto climb L1 — reverses retract path back to extended position. */
  public Command releaseFromAutoClimbL1() {
    return Commands.sequence(
            climb.releaseFromAutoL1(),
            Commands.runOnce(
                () ->
                    Logger.recordOutput(
                        "Climb/OperatorAction", "Released from auto climb L1 → EXTEND_L1_AUTO")))
        .withName("ReleaseFromAutoClimbL1");
  }

  /** Stow climb arms by following reverse paths from current state. */
  public Command stowClimb() {
    return climb.stowFromCurrentState().withName("StowClimb");
  }

  // ==================== Instant State Commands (254-style) ====================
  // These commands finish immediately. The periodic() loop takes over and
  // continuously applies the new state to all subsystems.

  /** Instantly set the wanted state. periodic() continuously applies it to all subsystems. */
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

  /** Force IDLE, bypassing emergency guard. Call at auto start for a clean slate. */
  public void forceIdleState() {
    Logger.recordOutput("Superstructure/StateTransition", currentState + " -> IDLE (forced)");
    this.wantedState = SuperstructureState.IDLE;
    this.currentState = SuperstructureState.IDLE;
    this.feedingRequested = false;
    this.intakeOutpostMode = false;
    this.intakeOutpostJiggleMode = false;
  }

  /** Set wanted state from non-command context (e.g. polling loop). Bypasses emergency guard. */
  public void forceWantedState(SuperstructureState state) {
    this.wantedState = state;
  }

  /** Whether the intake pivot is at stow position. Used by auto to gate climb extend. */
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
   * Update shooter completion detection based on conveyor current. Drops below threshold for the
   * configured duration = all FUEL fired. Requires current to have been high at least once first.
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

  /** Whether all FUEL has been fired (conveyor current stayed low after feeding). */
  public boolean isShooterFinishedFiring() {
    return shooterFinishedFiring;
  }

  /** Reset shooter detection state. Call before starting a new shooting sequence. */
  public void resetShooterDetection() {
    shooterFinishedFiring = false;
    conveyorEverSawFuel = false;
    conveyorLowCurrentStart = Timer.getFPGATimestamp();
    Logger.recordOutput("Superstructure/ShooterDetect/Reset", true);
  }
}
