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

/** Climb subsystem: ClimbState waypoints ClimbIK math TalonFX motors */
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
    Logger.recordOutput("Climb/CurrentState", currentState.toString());
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
                    this.currentState = previous;
                    return followWaypointPath(reversedWaypoints, duration);
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

  // Internal method for velocity-based path following
  private void setTargetVelocitiesInternal(
      Translation2d leftPosition,
      Translation2d rightPosition,
      Translation2d leftVelocity,
      Translation2d rightVelocity) {
    this.leftTargetPosition = leftPosition;
    this.rightTargetPosition = rightPosition;

    // Calculate IK to get motor rotations
    ClimbIKResult ikResult = ClimbIK.calculateBothSides(leftPosition, rightPosition);
    if (!ikResult.isValid()) {
      Logger.recordOutput("Climb/IK/Valid", false);
      return;
    }

    // Calculate motor velocities from Cartesian velocities using numerical differentiation
    // dθ/dt = (∂θ/∂x) * dx/dt + (∂θ/∂y) * dy/dt (chain rule)
    // For simplicity, we'll use the velocity magnitude scaled by the path curvature

    double leftSpeed = leftVelocity.getNorm(); // m/s
    double rightSpeed = rightVelocity.getNorm(); // m/s

    // Convert Cartesian speed to motor rotation speed (rough approximation)
    // Assuming typical arm velocity, scale by drum circumference and gear ratio
    double drumCircumference =
        Math.PI * frc.robot.Constants.ClimbConstants.CABLE_DRUM_DIAMETER_METERS;

    // Front motors
    double leftFrontVel =
        leftSpeed / (drumCircumference * frc.robot.Constants.ClimbConstants.FRONT_GEAR_RATIO);
    double rightFrontVel =
        rightSpeed / (drumCircumference * frc.robot.Constants.ClimbConstants.FRONT_GEAR_RATIO);

    // Back motors
    double leftBackVel =
        leftSpeed / (drumCircumference * frc.robot.Constants.ClimbConstants.BACK_GEAR_RATIO);
    double rightBackVel =
        rightSpeed / (drumCircumference * frc.robot.Constants.ClimbConstants.BACK_GEAR_RATIO);

    io.setLeftFrontVelocity(leftFrontVel);
    io.setLeftBackVelocity(leftBackVel);
    io.setRightFrontVelocity(rightFrontVel);
    io.setRightBackVelocity(rightBackVel);

    Logger.recordOutput("Climb/IK/Valid", true);
    Logger.recordOutput("Climb/Velocity/LeftSpeed", leftSpeed);
    Logger.recordOutput("Climb/Velocity/RightSpeed", rightSpeed);
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
        // Use multi-Bezier path for smooth continuous motion through waypoints
        // Lower tension (0.15) creates gentler curves that Motion Magic can follow smoothly
        ClimbPathPlanner.ClimbPath path =
            ClimbPathPlanner.createMultiBezierPath(waypoints, 0.15, durationSeconds);
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
          Translation2d[] velocities = executor.getCurrentVelocities();
          // Use velocity control for smooth feedforward motion
          setTargetVelocitiesInternal(targets[0], targets[1], velocities[0], velocities[1]);
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
