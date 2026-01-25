// Copyright (c) 2021-2026 Littleton Robotics
// Adapted from Team 2910/Amped Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Command that drives with joystick input while automatically aiming at a target position. Driver
 * maintains full control of translation (X/Y movement) while the robot automatically rotates to
 * face the target.
 */
public class DriveAimingAtTarget extends Command {
  private final DriveSwerveDrivetrain drive;
  private final Supplier<Translation3d> targetSupplier;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final PIDController headingController;

  // Track previous angle for feedforward calculation
  private double previousTargetHeading = 0.0;
  private long previousTime = System.nanoTime();

  /**
   * Creates a new DriveAimingAtTarget command.
   *
   * @param drive The swerve drivetrain subsystem
   * @param targetSupplier Supplier for the target position to aim at (e.g., hub coordinates)
   * @param xSupplier Supplier for X velocity (forward/backward) from joystick
   * @param ySupplier Supplier for Y velocity (left/right) from joystick
   */
  public DriveAimingAtTarget(
      DriveSwerveDrivetrain drive,
      Supplier<Translation3d> targetSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    this.drive = drive;
    this.targetSupplier = targetSupplier;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    // Create PID controller for smooth tracking
    this.headingController =
        new PIDController(
            Constants.DriveConstants.AutoAim.HEADING_KP,
            Constants.DriveConstants.AutoAim.HEADING_KI,
            Constants.DriveConstants.AutoAim.HEADING_KD);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void execute() {
    var currentPose = drive.getPose();

    // Get the current target from the supplier (allows dynamic target selection)
    Translation3d target = targetSupplier.get();

    // Calculate angle to target
    double dx = target.getX() - currentPose.getX();
    double dy = target.getY() - currentPose.getY();
    double targetHeading = Math.atan2(dy, dx);

    // Calculate PID output for rotational velocity
    double currentHeading = currentPose.getRotation().getRadians();

    // Calculate how fast the target angle is changing (feedforward)
    long currentTime = System.nanoTime();
    double dt = (currentTime - previousTime) / 1e9; // Convert to seconds
    double targetHeadingVelocity = 0.0;

    if (dt > 0.001) { // Avoid division by very small numbers
      double dTheta = targetHeading - previousTargetHeading;
      // Normalize angle change to [-PI, PI]
      while (dTheta > Math.PI) dTheta -= 2 * Math.PI;
      while (dTheta < -Math.PI) dTheta += 2 * Math.PI;
      targetHeadingVelocity = dTheta / dt;
    }

    previousTargetHeading = targetHeading;
    previousTime = currentTime;

    // PID output + Feedforward
    double pidOutput = headingController.calculate(currentHeading, targetHeading);
    double feedforwardOutput = Constants.DriveConstants.AutoAim.HEADING_KV * targetHeadingVelocity;
    double rotationalVelocity = pidOutput + feedforwardOutput;

    // Calculate error for logging
    double headingError = targetHeading - currentHeading;
    // Normalize to [-PI, PI] for logging
    while (headingError > Math.PI) headingError -= 2 * Math.PI;
    while (headingError < -Math.PI) headingError += 2 * Math.PI;

    // Apply driver inputs for translation, PID output for rotation
    drive.applyRequest(
        driveRequest
            .withVelocityX(xSupplier.getAsDouble())
            .withVelocityY(ySupplier.getAsDouble())
            .withRotationalRate(rotationalVelocity));

    // Log for debugging
    Logger.recordOutput("Drive/AutoAim/TargetX", target.getX());
    Logger.recordOutput("Drive/AutoAim/TargetY", target.getY());
    Logger.recordOutput("Drive/AutoAim/CurrentHeading", Math.toDegrees(currentHeading));
    Logger.recordOutput("Drive/AutoAim/TargetHeading", Math.toDegrees(targetHeading));
    Logger.recordOutput("Drive/AutoAim/HeadingError", Math.toDegrees(headingError));
    Logger.recordOutput("Drive/AutoAim/PIDOutput", pidOutput);
    Logger.recordOutput("Drive/AutoAim/FeedforwardOutput", feedforwardOutput);
    Logger.recordOutput(
        "Drive/AutoAim/TargetHeadingVelocity", Math.toDegrees(targetHeadingVelocity));
    Logger.recordOutput("Drive/AutoAim/RotationalVelocity", rotationalVelocity);
    Logger.recordOutput("Drive/AutoAim/Active", true);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    Logger.recordOutput("Drive/AutoAim/Active", false);
  }
}
