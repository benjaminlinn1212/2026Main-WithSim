package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.util.ClimbPathPlanner;
import frc.robot.subsystems.climb.util.ClimbPathPlanner.PathExecutor;
import org.littletonrobotics.junction.Logger;

/** Executes pre-planned climb paths with trapezoidal velocity profiles */
public class FollowPathCommand extends Command {
  private final ClimbSubsystem climbSubsystem;
  private final Translation2d leftTarget;
  private final Translation2d rightTarget;
  private final double maxVelocity;
  private final double maxAcceleration;

  private PathExecutor pathExecutor;
  private Translation2d leftStartPosition;
  private Translation2d rightStartPosition;

  public FollowPathCommand(
      ClimbSubsystem climbSubsystem,
      Translation2d leftTarget,
      Translation2d rightTarget,
      double maxVelocity,
      double maxAcceleration) {
    this.climbSubsystem = climbSubsystem;
    this.leftTarget = leftTarget;
    this.rightTarget = rightTarget;
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    leftStartPosition = climbSubsystem.getLeftTargetPosition();
    rightStartPosition = climbSubsystem.getRightTargetPosition();

    ClimbPathPlanner.ClimbPath leftPath =
        ClimbPathPlanner.createTrapezoidPath(
            leftStartPosition, leftTarget, maxVelocity, maxAcceleration);
    ClimbPathPlanner.ClimbPath rightPath =
        ClimbPathPlanner.createTrapezoidPath(
            rightStartPosition, rightTarget, maxVelocity, maxAcceleration);

    if (!ClimbPathPlanner.isPathValid(leftPath) || !ClimbPathPlanner.isPathValid(rightPath)) {
      Logger.recordOutput("Climb/Path/Error", "Invalid path - unreachable");
      cancel();
      return;
    }

    pathExecutor = new PathExecutor(leftPath, rightPath);
    pathExecutor.start();
  }

  @Override
  public void execute() {
    if (pathExecutor == null) return;

    Translation2d[] targets = pathExecutor.getCurrentTargets();
    climbSubsystem.setTargetPositions(targets[0], targets[1]);

    Logger.recordOutput("Climb/Path/Progress", pathExecutor.getProgress());
    Logger.recordOutput("Climb/Path/ElapsedTime", pathExecutor.getElapsedTime());
    Logger.recordOutput("Climb/Path/LeftTarget", targets[0]);
    Logger.recordOutput("Climb/Path/RightTarget", targets[1]);
  }

  @Override
  public boolean isFinished() {
    return pathExecutor != null && pathExecutor.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (pathExecutor != null) {
      pathExecutor.stop();
    }

    if (interrupted) {
      Logger.recordOutput("Climb/Path/Status", "Interrupted");
      climbSubsystem.stopMotors();
    } else {
      Logger.recordOutput("Climb/Path/Status", "Completed");
      climbSubsystem.setTargetPositions(leftTarget, rightTarget);
    }
  }
}
