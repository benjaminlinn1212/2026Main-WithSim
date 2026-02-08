package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.util.ClimbIK;
import frc.robot.subsystems.climb.util.ClimbIK.ClimbIKResult;
import frc.robot.subsystems.climb.util.ClimbPathPlanner;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/** Climb subsystem: ClimbState waypoints ClimbIK math TalonFX motors */
public class ClimbSubsystem extends SubsystemBase {

  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private ClimbState currentState = ClimbState.STOWED;
  private Translation2d leftTargetPosition = new Translation2d();
  private Translation2d rightTargetPosition = new Translation2d();

  public ClimbSubsystem(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    Logger.recordOutput("Climb/CurrentState", currentState.toString());
    Logger.recordOutput("Climb/LeftTargetPosition", leftTargetPosition);
    Logger.recordOutput("Climb/RightTargetPosition", rightTargetPosition);

    // Visualize end effector as 2D pose (for AdvantageScope 2D field view)
    Logger.recordOutput(
        "Climb/LeftEndEffector",
        new double[] {leftTargetPosition.getX(), leftTargetPosition.getY()});
    Logger.recordOutput(
        "Climb/RightEndEffector",
        new double[] {rightTargetPosition.getX(), rightTargetPosition.getY()});
  }

  // STATE MANAGEMENT

  public void setState(ClimbState state) {
    this.currentState = state;
    this.leftTargetPosition = state.getLeftTargetPosition();
    this.rightTargetPosition = state.getRightTargetPosition();
    moveToTargetPositions();
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
                        next.getPrePlannedWaypoints(), next.getDefaultDuration());
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
                        previous.getPrePlannedWaypoints(), previous.getDefaultDuration());
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

  public Command setStateCommand(ClimbState state) {
    return runOnce(
            () -> {
              if (state.hasPrePlannedPath()) {
                this.currentState = state;
                followWaypointPath(state.getPrePlannedWaypoints(), state.getDefaultDuration())
                    .schedule();
              } else {
                setState(state);
              }
            })
        .withName("ClimbSetState_" + state.getName());
  }

  // CORE CONTROL

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

  // MANUAL POSITION CONTROL

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

  // PATH FOLLOWING

  public Command followWaypointPath(List<Translation2d> waypoints, double durationSeconds) {
    return new Command() {
      private ClimbPathPlanner.PathExecutor executor;

      @Override
      public void initialize() {
        if (waypoints.size() < 2) {
          Logger.recordOutput("Climb/Path/Error", "Need at least 2 waypoints");
          cancel();
          return;
        }
        ClimbPathPlanner.ClimbPath path =
            ClimbPathPlanner.createSmoothPath(waypoints, durationSeconds);
        if (!ClimbPathPlanner.isPathValid(path)) {
          Logger.recordOutput("Climb/Path/Error", "Invalid waypoint path");
          cancel();
          return;
        }
        executor = new ClimbPathPlanner.PathExecutor(path, path);
        executor.start();
        Logger.recordOutput("Climb/Path/WaypointCount", waypoints.size());

        // Log all waypoints for visualization in AdvantageScope
        double[] waypointXs = waypoints.stream().mapToDouble(Translation2d::getX).toArray();
        double[] waypointYs = waypoints.stream().mapToDouble(Translation2d::getY).toArray();
        Logger.recordOutput("Climb/Path/WaypointsX", waypointXs);
        Logger.recordOutput("Climb/Path/WaypointsY", waypointYs);
      }

      @Override
      public void execute() {
        if (executor != null) {
          Translation2d[] targets = executor.getCurrentTargets();
          setTargetPositionsInternal(targets[0], targets[1]); // Position control
        }
      }

      @Override
      public boolean isFinished() {
        return executor != null && executor.isFinished();
      }

      @Override
      public void end(boolean interrupted) {
        if (executor != null) executor.stop();
        if (!interrupted) {
          Translation2d finalPos = waypoints.get(waypoints.size() - 1);
          setTargetPositionsInternal(finalPos, finalPos);
        }
      }
    }.withName("ClimbFollowWaypointPath");
  }

  // GETTERS

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

  // PASSIVE HOOK RELEASE

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

  // EMERGENCY

  public void stopMotors() {
    io.stop();
    currentState = ClimbState.EMERGENCY_STOP;
  }

  public Command emergencyStop() {
    return runOnce(this::stopMotors).withName("ClimbEmergencyStop");
  }
}
