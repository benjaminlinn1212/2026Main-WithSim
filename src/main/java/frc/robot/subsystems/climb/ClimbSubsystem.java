package frc.robot.subsystems.climb;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.util.ClimbIK;
import frc.robot.subsystems.climb.util.ClimbIK.ClimbIKResult;
import frc.robot.subsystems.climb.util.ClimbPathPlanner;
import java.util.List;
import java.util.Set;
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
  private Translation2d leftTargetPosition = new Translation2d();
  private Translation2d rightTargetPosition = new Translation2d();

  // Mechanism2d for visualization
  private final Mechanism2d mechanism;
  private final MechanismRoot2d leftRoot;
  private final MechanismRoot2d rightRoot;
  private final MechanismLigament2d leftFrontArm;
  private final MechanismLigament2d leftBackArm;
  private final MechanismLigament2d rightFrontArm;
  private final MechanismLigament2d rightBackArm;

  public ClimbSubsystem(ClimbIO io) {
    this.io = io;

    // Create Mechanism2d (1m wide x 1m tall canvas)
    mechanism = new Mechanism2d(1.0, 1.0);

    // Create roots at base positions (left and right sides)
    leftRoot = mechanism.getRoot("LeftBase", 0.25, 0.0);
    rightRoot = mechanism.getRoot("RightBase", 0.75, 0.0);

    // Create ligaments (arms) - will be updated in periodic()
    leftFrontArm =
        leftRoot.append(
            new MechanismLigament2d("LeftFront", 0.3, 90, 6, new Color8Bit(Color.kRed)));
    leftBackArm =
        leftRoot.append(
            new MechanismLigament2d("LeftBack", 0.3, 90, 6, new Color8Bit(Color.kBlue)));
    rightFrontArm =
        rightRoot.append(
            new MechanismLigament2d("RightFront", 0.3, 90, 6, new Color8Bit(Color.kRed)));
    rightBackArm =
        rightRoot.append(
            new MechanismLigament2d("RightBack", 0.3, 90, 6, new Color8Bit(Color.kBlue)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    // State is logged by commands (setState, followWaypointPath)
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

    leftFrontArm.setLength(leftLength);
    leftFrontArm.setAngle(leftAngle);
    rightFrontArm.setLength(rightLength);
    rightFrontArm.setAngle(rightAngle);

    // Log Mechanism2d for AdvantageScope Mechanism view
    SmartDashboard.putData("Climb/Mechanism", mechanism);
  }

  // =============================================================================
  // STATE MANAGEMENT
  // =============================================================================

  public void setState(ClimbState state) {
    this.currentState = state;
    this.leftTargetPosition = state.getLeftTargetPosition();
    this.rightTargetPosition = state.getRightTargetPosition();
    moveToTargetPositions();

    // Log detailed state (e.g., "REACHED_L1" when holding position)
    Logger.recordOutput("Climb/CurrentState", "REACHED_" + state.getName());
  }

  public ClimbState getState() {
    return currentState;
  }

  public Command nextState() {
    return Commands.either(
            // If next state has a path, return the full path-following command
            Commands.defer(
                () -> {
                  ClimbState next = currentState.getNextState();
                  if (next != null && next.hasPrePlannedPath()) {
                    this.currentState = next;
                    return followWaypointPath(
                        next.getPrePlannedWaypoints(), next.getDefaultDuration(), next.isPulling());
                  } else if (next != null) {
                    return runOnce(() -> setState(next));
                  } else {
                    return Commands.none();
                  }
                },
                Set.of(this)),
            Commands.none(),
            () -> currentState.getNextState() != null)
        .withName("ClimbNextState");
  }

  public Command previousState() {
    return Commands.either(
            // If previous state has a path, return the full path-following command
            Commands.defer(
                () -> {
                  ClimbState previous = currentState.getPreviousState();
                  if (previous != null && previous.hasPrePlannedPath()) {
                    this.currentState = previous;
                    return followWaypointPath(
                        previous.getPrePlannedWaypoints(),
                        previous.getDefaultDuration(),
                        previous.isPulling());
                  } else if (previous != null) {
                    return runOnce(() -> setState(previous));
                  } else {
                    return Commands.none();
                  }
                },
                Set.of(this)),
            Commands.none(),
            () -> currentState.getPreviousState() != null)
        .withName("ClimbPreviousState");
  }

  /** Go to previous state using REVERSED path of current state (for smooth backwards motion) */
  public Command previousStateReversed() {
    return Commands.either(
            // Use reversed waypoints of CURRENT state to go back
            Commands.defer(
                () -> {
                  ClimbState previous = currentState.getPreviousState();
                  if (previous != null && currentState.hasPrePlannedPath()) {
                    List<Translation2d> reversedWaypoints = currentState.getReversedWaypoints();
                    double duration =
                        currentState.getDefaultDuration(); // Save duration BEFORE changing state
                    boolean isPulling =
                        currentState
                            .isPulling(); // Reversed path has opposite pulling (true -> false)
                    this.currentState = previous;
                    return followWaypointPath(reversedWaypoints, duration, !isPulling);
                  } else if (previous != null) {
                    return runOnce(() -> setState(previous));
                  } else {
                    return Commands.none();
                  }
                },
                Set.of(this)),
            Commands.none(),
            () -> currentState.getPreviousState() != null)
        .withName("ClimbPreviousStateReversed");
  }

  public Command setStateCommand(ClimbState state) {
    if (state.hasPrePlannedPath()) {
      return Commands.sequence(
              runOnce(() -> this.currentState = state),
              followWaypointPath(
                  state.getPrePlannedWaypoints(), state.getDefaultDuration(), state.isPulling()))
          .withName("ClimbSetState_" + state.getName());
    } else {
      return runOnce(() -> setState(state)).withName("ClimbSetState_" + state.getName());
    }
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

  public void setTargetPositions(Translation2d leftPosition, Translation2d rightPosition) {
    this.leftTargetPosition = leftPosition;
    this.rightTargetPosition = rightPosition;
    this.currentState = ClimbState.MANUAL;
    moveToTargetPositions();
  }

  public void setSymmetricTargetPosition(Translation2d position) {
    setTargetPositions(position, position);
  }

  // Internal method for path following (doesn't change state to MANUAL)
  private void setTargetPositionsInternal(Translation2d leftPosition, Translation2d rightPosition) {
    this.leftTargetPosition = leftPosition;
    this.rightTargetPosition = rightPosition;
    moveToTargetPositions();
  }

  // ===========================================================================
  // DIRECT MOTOR POSITION CONTROL (Test Mode)
  // ===========================================================================

  /** Set left front motor position directly (bypasses IK) - for test mode. */
  public void setLeftFrontMotorPosition(double positionRotations) {
    io.setLeftFrontPosition(positionRotations);
  }

  /** Set right front motor position directly (bypasses IK) - for test mode. */
  public void setRightFrontMotorPosition(double positionRotations) {
    io.setRightFrontPosition(positionRotations);
  }

  /** Set left back motor position directly (bypasses IK) - for test mode. */
  public void setLeftBackMotorPosition(double positionRotations) {
    io.setLeftBackPosition(positionRotations);
  }

  /** Set right back motor position directly (bypasses IK) - for test mode. */
  public void setRightBackMotorPosition(double positionRotations) {
    io.setRightBackPosition(positionRotations);
  }

  // ===========================================================================
  // DIRECT MOTOR VOLTAGE CONTROL (Test Mode)
  // ===========================================================================

  /** Set left front motor voltage directly - for test mode. */
  public void setLeftFrontMotorVoltage(double volts) {
    io.setLeftFrontVoltage(volts);
  }

  /** Set right front motor voltage directly - for test mode. */
  public void setRightFrontMotorVoltage(double volts) {
    io.setRightFrontVoltage(volts);
  }

  /** Set left back motor voltage directly - for test mode. */
  public void setLeftBackMotorVoltage(double volts) {
    io.setLeftBackVoltage(volts);
  }

  /** Set right back motor voltage directly - for test mode. */
  public void setRightBackMotorVoltage(double volts) {
    io.setRightBackVoltage(volts);
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

    // Use numerical Jacobian to calculate motor velocities from end effector velocities
    ClimbIK.ClimbVelocityResult velocityResult =
        ClimbIK.calculateVelocityIKBothSides(
            leftPosition, rightPosition, leftVelocity, rightVelocity);

    if (!velocityResult.isValid()) {
      Logger.recordOutput("Climb/VelocityIK/Valid", false);
      // Fallback to position control if velocity IK fails
      moveToTargetPositions();
      return;
    }

    Logger.recordOutput("Climb/VelocityIK/Valid", true);

    // Apply extra feedforward if this is a pulling motion to overcome gravity/load
    double feedforward = isPulling ? frc.robot.Constants.ClimbConstants.VELOCITY_KG_PULLING : 0.0;

    // Send velocity commands to all 4 motors
    io.setLeftFrontVelocity(velocityResult.leftSide.frontMotorVelocity, feedforward);
    io.setLeftBackVelocity(velocityResult.leftSide.backMotorVelocity, feedforward);
    io.setRightFrontVelocity(velocityResult.rightSide.frontMotorVelocity, feedforward);
    io.setRightBackVelocity(velocityResult.rightSide.backMotorVelocity, feedforward);

    // Log velocities for debugging
    Logger.recordOutput(
        "Climb/VelocityIK/LeftFrontVel", velocityResult.leftSide.frontMotorVelocity);
    Logger.recordOutput("Climb/VelocityIK/LeftBackVel", velocityResult.leftSide.backMotorVelocity);
    Logger.recordOutput(
        "Climb/VelocityIK/RightFrontVel", velocityResult.rightSide.frontMotorVelocity);
    Logger.recordOutput(
        "Climb/VelocityIK/RightBackVel", velocityResult.rightSide.backMotorVelocity);

    // Log end effector velocities
    Logger.recordOutput("Climb/VelocityIK/LeftVelX", leftVelocity.getX());
    Logger.recordOutput("Climb/VelocityIK/LeftVelY", leftVelocity.getY());
    Logger.recordOutput("Climb/VelocityIK/RightVelX", rightVelocity.getX());
    Logger.recordOutput("Climb/VelocityIK/RightVelY", rightVelocity.getY());

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
  // PATH FOLLOWING (Waypoint-based trajectory execution)
  // ===========================================================================

  /**
   * Follow a smooth path through waypoints using velocity control.
   *
   * @param waypoints List of waypoints to pass through
   * @param durationSeconds Total time to complete the path
   * @param isPulling Whether this is a pulling motion (for extra feedforward)
   * @return Command that executes the path
   */
  public Command followWaypointPath(
      List<Translation2d> waypoints, double durationSeconds, boolean isPulling) {
    return new Command() {
      private ClimbPathPlanner.PathExecutor executor;

      @Override
      public void initialize() {
        // Log that we're actively moving (reaching or pulling)
        String action = isPulling ? "PULLING" : "REACHING";
        Logger.recordOutput("Climb/CurrentState", action + "_" + currentState.getName());

        System.out.println(
            "[ClimbPath] Initializing path with "
                + waypoints.size()
                + " waypoints, isPulling="
                + isPulling);
        System.out.println("[ClimbPath] First waypoint: " + waypoints.get(0));
        System.out.println("[ClimbPath] Current left position: " + leftTargetPosition);
        System.out.println("[ClimbPath] Current right position: " + rightTargetPosition);
        if (waypoints.size() < 2) {
          Logger.recordOutput("Climb/Path/Error", "Need at least 2 waypoints");
          System.out.println("[ClimbPath] ERROR: Need at least 2 waypoints");
          cancel();
          return;
        }
        // Use multi-Bezier path for smooth continuous motion through waypoints
        // Lower tension (0.15) creates gentler curves that Motion Magic can follow smoothly
        // Pass isPulling to control end velocity: pulling = maintain speed, reaching = decelerate
        ClimbPathPlanner.ClimbPath path =
            ClimbPathPlanner.createMultiBezierPath(waypoints, 0.15, durationSeconds, isPulling);
        if (!ClimbPathPlanner.isPathValid(path)) {
          Logger.recordOutput("Climb/Path/Error", "Invalid waypoint path");
          System.out.println("[ClimbPath] ERROR: Invalid waypoint path");
          // Debug: check each waypoint
          for (int i = 0; i < path.getWaypoints().size(); i++) {
            Translation2d point = path.getWaypoints().get(i);
            boolean reachable = ClimbIK.isPositionReachable(point);
            boolean inWorkspace = ClimbPathPlanner.isWithinWorkspace(point);
            System.out.println(
                "[ClimbPath] Waypoint "
                    + i
                    + ": "
                    + point
                    + " reachable="
                    + reachable
                    + " inWorkspace="
                    + inWorkspace);
          }
          cancel();
          return;
        }
        executor = new ClimbPathPlanner.PathExecutor(path, path);
        executor.start();
        System.out.println("[ClimbPath] Path started successfully");
        Logger.recordOutput("Climb/Path/WaypointCount", waypoints.size());

        // Log all waypoints for visualization in AdvantageScope
        double[] waypointXs = waypoints.stream().mapToDouble(Translation2d::getX).toArray();
        double[] waypointYs = waypoints.stream().mapToDouble(Translation2d::getY).toArray();
        Logger.recordOutput("Climb/Path/WaypointsX", waypointXs);
        Logger.recordOutput("Climb/Path/WaypointsY", waypointYs);

        // Log generated path points for smooth curve visualization
        List<Translation2d> pathPoints = path.getWaypoints();
        double[] pathXs = pathPoints.stream().mapToDouble(Translation2d::getX).toArray();
        double[] pathYs = pathPoints.stream().mapToDouble(Translation2d::getY).toArray();
        Logger.recordOutput("Climb/Path/GeneratedX", pathXs);
        Logger.recordOutput("Climb/Path/GeneratedY", pathYs);
      }

      @Override
      public void execute() {
        if (executor != null) {
          Translation2d[] targets = executor.getCurrentTargets();
          Translation2d[] velocities = executor.getCurrentVelocities();
          // Use velocity control for smooth feedforward motion
          setTargetVelocitiesInternal(
              targets[0], targets[1], velocities[0], velocities[1], isPulling);
        }
      }

      @Override
      public boolean isFinished() {
        boolean finished = executor != null && executor.isFinished();
        if (finished) {
          System.out.println("[ClimbPath] Path finished");
        }
        return finished;
      }

      @Override
      public void end(boolean interrupted) {
        System.out.println("[ClimbPath] Path ended, interrupted=" + interrupted);
        if (executor != null) executor.stop();
        if (!interrupted) {
          Translation2d finalPos = waypoints.get(waypoints.size() - 1);
          setTargetPositionsInternal(finalPos, finalPos);
          // Log that we've reached the target position
          Logger.recordOutput("Climb/CurrentState", "REACHED_" + currentState.getName());
        }
      }
    }.withName("ClimbFollowWaypointPath");
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

  public double getRightFrontPosition() {
    return inputs.rightFrontPositionRotations;
  }

  public double getRightBackPosition() {
    return inputs.rightBackPositionRotations;
  }

  public double getLeftFrontPosition() {
    return inputs.leftFrontPositionRotations;
  }

  public double getLeftBackPosition() {
    return inputs.leftBackPositionRotations;
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

  public Command releaseHooksCommand() {
    return runOnce(this::releaseHooks).withName("ClimbReleaseHooks");
  }

  public Command stowHooksCommand() {
    return runOnce(this::stowHooks).withName("ClimbStowHooks");
  }

  // ===========================================================================
  // EMERGENCY
  // ===========================================================================

  public void stopMotors() {
    io.stop();
    currentState = ClimbState.EMERGENCY_STOP;
  }

  public Command emergencyStop() {
    return runOnce(this::stopMotors).withName("ClimbEmergencyStop");
  }
}
