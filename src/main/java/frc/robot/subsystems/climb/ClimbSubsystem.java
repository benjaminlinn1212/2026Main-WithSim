package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.util.ClimbIK;
import frc.robot.subsystems.climb.util.ClimbIK.ClimbIKResult;
import frc.robot.subsystems.climb.util.ClimbPathPlanner;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Climb subsystem: ClimbState waypoints ClimbIK math TalonFX motors
 *
 * <p>CONTROL STRATEGY (Hybrid Position + Velocity): - VELOCITY CONTROL: Used during path following
 * for smooth dynamic motion - Numerical Jacobian converts end effector velocity → motor velocity -
 * Similar to drivetrain: chassis speeds → wheel speeds - Enables accurate trajectory tracking with
 * feedforward
 *
 * <p>- POSITION CONTROL: Used for holding static positions - Motion Magic to final positions after
 * path completes - Used for MANUAL and EMERGENCY_STOP states - Provides stable position hold at
 * target locations
 */
public class ClimbSubsystem extends SubsystemBase {

  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private ClimbState currentState = ClimbState.STOWED;
  private Translation2d leftTargetPosition = ClimbState.STOWED.getTargetPosition();
  private Translation2d rightTargetPosition = ClimbState.STOWED.getTargetPosition();

  // Mechanism2d for visualization
  private final Mechanism2d mechanism;
  private final MechanismLigament2d leftArm;
  private final MechanismLigament2d rightArm;

  public ClimbSubsystem(ClimbIO io) {
    this.io = io;

    // Create Mechanism2d (1m wide x 1m tall canvas)
    mechanism = new Mechanism2d(1.0, 1.0);

    // Create roots and ligaments for left and right side visualization
    MechanismRoot2d leftRoot = mechanism.getRoot("LeftBase", 0.25, 0.0);
    MechanismRoot2d rightRoot = mechanism.getRoot("RightBase", 0.75, 0.0);
    leftArm =
        leftRoot.append(new MechanismLigament2d("LeftArm", 0.3, 90, 6, new Color8Bit(Color.kRed)));
    rightArm =
        rightRoot.append(
            new MechanismLigament2d("RightArm", 0.3, 90, 6, new Color8Bit(Color.kRed)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    // State is logged by commands (setState, runPath)
    Logger.recordOutput("Climb/LeftTargetPosition", leftTargetPosition);
    Logger.recordOutput("Climb/RightTargetPosition", rightTargetPosition);

    // Visualize end effector as Pose2d (for AdvantageScope Points view)
    Logger.recordOutput("Climb/LeftEndEffector", new Pose2d(leftTargetPosition, new Rotation2d()));
    Logger.recordOutput(
        "Climb/RightEndEffector", new Pose2d(rightTargetPosition, new Rotation2d()));

    // Update mechanism arms to show current positions
    double leftLength = Math.hypot(leftTargetPosition.getX(), leftTargetPosition.getY());
    double leftAngle =
        Math.toDegrees(Math.atan2(leftTargetPosition.getY(), leftTargetPosition.getX()));
    double rightLength = Math.hypot(rightTargetPosition.getX(), rightTargetPosition.getY());
    double rightAngle =
        Math.toDegrees(Math.atan2(rightTargetPosition.getY(), rightTargetPosition.getX()));

    leftArm.setLength(leftLength);
    leftArm.setAngle(leftAngle);
    rightArm.setLength(rightLength);
    rightArm.setAngle(rightAngle);

    // Log Mechanism2d for AdvantageScope Mechanism view
    SmartDashboard.putData("Climb/Mechanism", mechanism);
  }

  // =============================================================================
  // STATE MANAGEMENT
  // =============================================================================

  public void setState(ClimbState state) {
    this.currentState = state;
    this.leftTargetPosition = state.getTargetPosition();
    this.rightTargetPosition = state.getTargetPosition();
    moveToTargetPositions();
    Logger.recordOutput("Climb/CurrentState", state.getName());
  }

  public ClimbState getState() {
    return currentState;
  }

  /** Advance to the next climb state with path following. */
  public Command nextState() {
    return runPath(
        () -> {
          ClimbState next = currentState.getNextState();
          if (next == null || !next.hasPrePlannedPath()) {
            if (next != null) setState(next);
            return null;
          }
          currentState = next;
          Logger.recordOutput("Climb/CurrentState", next.getName());
          return new PathParams(
              next.getPrePlannedWaypoints(), next.getDefaultDuration(), next.isPulling());
        },
        "ClimbNextState");
  }

  /** Go back to the previous climb state using reversed path. */
  public Command previousState() {
    return runPath(
        () -> {
          ClimbState prev = currentState.getPreviousState();
          if (prev == null || !currentState.hasPrePlannedPath()) {
            if (prev != null) setState(prev);
            return null;
          }
          List<Translation2d> reversed = currentState.getReversedWaypoints();
          double duration = currentState.getDefaultDuration();
          boolean isPulling = !currentState.isPulling();
          currentState = prev;
          Logger.recordOutput("Climb/CurrentState", prev.getName());
          return new PathParams(reversed, duration, isPulling);
        },
        "ClimbPreviousState");
  }

  /** Transition to a specific state. Safe to chain in Commands.sequence(). */
  public Command setStateCommand(ClimbState state) {
    return runPath(
        () -> {
          currentState = state;
          Logger.recordOutput("Climb/CurrentState", state.getName());
          if (!state.hasPrePlannedPath()) {
            setState(state);
            return null;
          }
          return new PathParams(
              state.getPrePlannedWaypoints(), state.getDefaultDuration(), state.isPulling());
        },
        "ClimbSetState_" + state.name());
  }

  // -- Helper: reusable path-following command --

  private record PathParams(List<Translation2d> waypoints, double duration, boolean isPulling) {}

  /**
   * Creates a command that calls the supplier on initialize to get path params, then follows the
   * path. If supplier returns null, finishes immediately (no path to run).
   */
  private Command runPath(java.util.function.Supplier<PathParams> paramSupplier, String name) {
    return new Command() {
      private ClimbPathPlanner.PathExecutor executor;
      private PathParams params;

      {
        addRequirements(ClimbSubsystem.this);
        setName(name);
      }

      @Override
      public void initialize() {
        executor = null;
        params = paramSupplier.get();
        if (params == null || params.waypoints().size() < 2) return;

        try {
          var path =
              ClimbPathPlanner.createMultiBezierPath(
                  params.waypoints(), 0.15, params.duration(), params.isPulling());
          if (!ClimbPathPlanner.isPathValid(path)) return;
          executor = new ClimbPathPlanner.PathExecutor(path, path);
          executor.start();
        } catch (Exception e) {
          System.out.println("[Climb] Path generation failed: " + e.getMessage());
        }
      }

      @Override
      public void execute() {
        if (executor != null) {
          Translation2d[] targets = executor.getCurrentTargets();
          Translation2d[] velocities = executor.getCurrentVelocities();
          setTargetVelocitiesInternal(
              targets[0], targets[1], velocities[0], velocities[1], params.isPulling());
        }
      }

      @Override
      public boolean isFinished() {
        return executor == null || executor.isFinished();
      }

      @Override
      public void end(boolean interrupted) {
        if (executor != null) executor.stop();
        if (!interrupted && params != null && !params.waypoints().isEmpty()) {
          Translation2d finalPos = params.waypoints().get(params.waypoints().size() - 1);
          setTargetPositionsInternal(finalPos, finalPos);
        } else {
          moveToTargetPositions();
        }
      }
    };
  }

  // ===========================================================================
  // POSITION CONTROL (Motion Magic for static positions)
  // ===========================================================================

  private void moveToTargetPositions() {
    ClimbIKResult ikResult = ClimbIK.calculateBothSides(leftTargetPosition, rightTargetPosition);
    if (ikResult.isValid()) {
      io.setLeftFrontPosition(ikResult.leftSide.frontMotorRotations);
      io.setLeftBackPosition(ikResult.leftSide.backMotorRotations);
      io.setRightFrontPosition(ikResult.rightSide.frontMotorRotations);
      io.setRightBackPosition(ikResult.rightSide.backMotorRotations);
      Logger.recordOutput("Climb/IK/Valid", true);

      // Log joint positions for mechanism visualization
      Logger.recordOutput(
          "Climb/IK/LeftJoint", new double[] {ikResult.leftSide.jointX, ikResult.leftSide.jointY});
      Logger.recordOutput(
          "Climb/IK/RightJoint",
          new double[] {ikResult.rightSide.jointX, ikResult.rightSide.jointY});
    } else {
      Logger.recordOutput("Climb/IK/Valid", false);
    }
  }

  // Internal method for path following (doesn't change state to MANUAL)
  private void setTargetPositionsInternal(Translation2d leftPosition, Translation2d rightPosition) {
    this.leftTargetPosition = leftPosition;
    this.rightTargetPosition = rightPosition;
    moveToTargetPositions();
  }

  // ===========================================================================
  // VELOCITY CONTROL (Jacobian-based for path following)
  // ===========================================================================

  /**
   * Set motor velocities using numerical Jacobian transformation.
   *
   * <p>Flow: (x, y, xdot, ydot) → IK → (l1, l2) → Jacobian → (l1dot, l2dot) → motors
   *
   * @param leftPosition Current left end effector position (m)
   * @param rightPosition Current right end effector position (m)
   * @param leftVelocity Desired left end effector velocity (m/s)
   * @param rightVelocity Desired right end effector velocity (m/s)
   * @param isPulling Whether this is a pulling motion (adds extra feedforward)
   */
  private void setTargetVelocitiesInternal(
      Translation2d leftPosition,
      Translation2d rightPosition,
      Translation2d leftVelocity,
      Translation2d rightVelocity,
      boolean isPulling) {
    this.leftTargetPosition = leftPosition;
    this.rightTargetPosition = rightPosition;

    Translation2d measuredLeftPosition =
        ClimbIK.estimateEndEffectorPosition(
            inputs.leftFrontPositionRotations,
            inputs.leftBackPositionRotations,
            leftTargetPosition);
    Translation2d measuredRightPosition =
        ClimbIK.estimateEndEffectorPosition(
            inputs.rightFrontPositionRotations,
            inputs.rightBackPositionRotations,
            rightTargetPosition);

    if (measuredLeftPosition == null) {
      measuredLeftPosition = leftTargetPosition;
    }
    if (measuredRightPosition == null) {
      measuredRightPosition = rightTargetPosition;
    }

    // Use numerical Jacobian to calculate motor velocities from end effector velocities
    ClimbIK.ClimbVelocityResult velocityResult =
        ClimbIK.calculateVelocityIKBothSides(
            measuredLeftPosition, measuredRightPosition, leftVelocity, rightVelocity);

    if (!velocityResult.isValid()) {
      Logger.recordOutput("Climb/VelocityIK/Valid", false);
      // Fallback to position control if velocity IK fails
      moveToTargetPositions();
      return;
    }

    Logger.recordOutput("Climb/VelocityIK/Valid", true);

    double leftFrontVel =
        MathUtil.clamp(
            velocityResult.leftSide.frontMotorVelocity,
            -frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY,
            frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY);
    double leftBackVel =
        MathUtil.clamp(
            velocityResult.leftSide.backMotorVelocity,
            -frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY,
            frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY);
    double rightFrontVel =
        MathUtil.clamp(
            velocityResult.rightSide.frontMotorVelocity,
            -frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY,
            frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY);
    double rightBackVel =
        MathUtil.clamp(
            velocityResult.rightSide.backMotorVelocity,
            -frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY,
            frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY);

    // Apply gravity compensation feedforward using Jacobian transpose if pulling
    // J^T * F_gravity gives geometrically-correct torque distribution across motors
    double leftFrontFF = 0.0;
    double leftBackFF = 0.0;
    double rightFrontFF = 0.0;
    double rightBackFF = 0.0;
    if (isPulling) {
      double[] leftGravFF =
          ClimbIK.calculateGravityFeedforward(
              measuredLeftPosition, frc.robot.Constants.ClimbConstants.VELOCITY_KG_PULLING);
      double[] rightGravFF =
          ClimbIK.calculateGravityFeedforward(
              measuredRightPosition, frc.robot.Constants.ClimbConstants.VELOCITY_KG_PULLING);
      leftFrontFF = leftGravFF[0];
      leftBackFF = leftGravFF[1];
      rightFrontFF = rightGravFF[0];
      rightBackFF = rightGravFF[1];
    }

    // Send velocity commands to all 4 motors
    io.setLeftFrontVelocity(leftFrontVel, leftFrontFF);
    io.setLeftBackVelocity(leftBackVel, leftBackFF);
    io.setRightFrontVelocity(rightFrontVel, rightFrontFF);
    io.setRightBackVelocity(rightBackVel, rightBackFF);

    // Log velocities for debugging
    Logger.recordOutput("Climb/VelocityIK/LeftFrontVel", leftFrontVel);
    Logger.recordOutput("Climb/VelocityIK/LeftBackVel", leftBackVel);
    Logger.recordOutput("Climb/VelocityIK/RightFrontVel", rightFrontVel);
    Logger.recordOutput("Climb/VelocityIK/RightBackVel", rightBackVel);

    // Log end effector velocities
    Logger.recordOutput("Climb/VelocityIK/LeftVelX", leftVelocity.getX());
    Logger.recordOutput("Climb/VelocityIK/LeftVelY", leftVelocity.getY());
    Logger.recordOutput("Climb/VelocityIK/RightVelX", rightVelocity.getX());
    Logger.recordOutput("Climb/VelocityIK/RightVelY", rightVelocity.getY());
    Logger.recordOutput("Climb/VelocityIK/LeftMeasured", measuredLeftPosition);
    Logger.recordOutput("Climb/VelocityIK/RightMeasured", measuredRightPosition);

    // Still calculate IK for position logging
    ClimbIKResult ikResult = ClimbIK.calculateBothSides(leftPosition, rightPosition);
    if (ikResult.isValid()) {
      Logger.recordOutput("Climb/IK/Valid", true);
      Logger.recordOutput(
          "Climb/IK/LeftJoint", new double[] {ikResult.leftSide.jointX, ikResult.leftSide.jointY});
      Logger.recordOutput(
          "Climb/IK/RightJoint",
          new double[] {ikResult.rightSide.jointX, ikResult.rightSide.jointY});
    }
  }

  // ===========================================================================
  // GETTERS
  // ===========================================================================

  public Translation2d getLeftTargetPosition() {
    return leftTargetPosition;
  }

  public Translation2d getRightTargetPosition() {
    return rightTargetPosition;
  }

  // ===========================================================================
  // PASSIVE HOOKS
  // ===========================================================================

  /** Release passive hooks (called when entering climb mode). */
  public void releaseHooks() {
    io.setLeftHookPosition(frc.robot.Constants.ClimbConstants.HOOK_RELEASED_POSITION);
    io.setRightHookPosition(frc.robot.Constants.ClimbConstants.HOOK_RELEASED_POSITION);
    Logger.recordOutput("Climb/HooksReleased", true);
  }

  /** Stow passive hooks (lock them). */
  public void stowHooks() {
    io.setLeftHookPosition(frc.robot.Constants.ClimbConstants.HOOK_STOWED_POSITION);
    io.setRightHookPosition(frc.robot.Constants.ClimbConstants.HOOK_STOWED_POSITION);
    Logger.recordOutput("Climb/HooksReleased", false);
  }

  /** Command to release passive hooks. */
  public Command releaseHooksCommand() {
    Command cmd = runOnce(this::releaseHooks);
    cmd.setName("ClimbReleaseHooks");
    return cmd;
  }

  /** Command to stow passive hooks. */
  public Command stowHooksCommand() {
    Command cmd = runOnce(this::stowHooks);
    cmd.setName("ClimbStowHooks");
    return cmd;
  }

  // ===========================================================================
  // EMERGENCY
  // ===========================================================================

  /** Immediately stop all motors and enter emergency state. */
  public void stopMotors() {
    io.stop();
    currentState = ClimbState.EMERGENCY_STOP;
  }

  /** Command to emergency-stop the climb subsystem. */
  public Command emergencyStop() {
    Command cmd = runOnce(this::stopMotors);
    cmd.setName("ClimbEmergencyStop");
    return cmd;
  }
}
