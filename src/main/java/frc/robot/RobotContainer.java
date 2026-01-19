// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveAimingAtTarget;
import frc.robot.commands.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import frc.robot.util.sim.MapleSimSwerveDrivetrain;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Robot state (matches 254)
  private final RobotState robotState = new RobotState();

  // Subsystems
  private final DriveSwerveDrivetrain swerveIO;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Regulate module constants for simulation (254's approach)
    if (Constants.currentMode == Constants.Mode.SIM) {
      MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(
          new com.ctre.phoenix6.swerve.SwerveModuleConstants<?, ?, ?>[] {
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
          });
    }

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, use CTRE SwerveDrivetrain directly (254's approach)
        swerveIO =
            new DriveSwerveDrivetrain(
                new DriveIOHardware(
                    robotState,
                    TunerConstants.DrivetrainConstants,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight),
                robotState);
        break;

      case SIM:
        // Sim robot, use DriveIOSim with Maple-Sim integration (254's approach)
        swerveIO =
            new DriveSwerveDrivetrain(
                new DriveIOSim(
                    robotState,
                    TunerConstants.DrivetrainConstants,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight),
                robotState);
        break;

      default:
        // For replay, create minimal DriveIOHardware
        swerveIO =
            new DriveSwerveDrivetrain(
                new DriveIOHardware(
                    robotState,
                    TunerConstants.DrivetrainConstants,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight),
                robotState);
        break;
    }

    // Set up auto routines - disabled until DriveCommands are updated
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("Do Nothing", Commands.none());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, field-relative drive using 254's approach
    swerveIO.setDefaultCommand(
        Commands.run(
            () -> {
              double vxMetersPerSec = -controller.getLeftY() * 5.0; // Max 5 m/s
              double vyMetersPerSec = -controller.getLeftX() * 5.0;
              double omegaRadPerSec = -controller.getRightX() * Math.PI; // Max PI rad/s
              swerveIO.driveFieldRelative(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec);
            },
            swerveIO));

    // Drive to test pose when Y button is held
    controller.y().whileTrue(new DriveToPose(swerveIO, Constants.FieldPoses.TEST));

    // Auto-aim at speaker while left trigger is held (automatically selects based on alliance)
    controller
        .leftTrigger()
        .whileTrue(
            new DriveAimingAtTarget(
                swerveIO,
                () ->
                    edu.wpi.first.wpilibj.DriverStation.getAlliance()
                        .map(
                            alliance ->
                                alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
                                    ? Constants.FieldPoses.BLUE_AIM_TARGET
                                    : Constants.FieldPoses.RED_AIM_TARGET)
                        .orElse(Constants.FieldPoses.BLUE_AIM_TARGET),
                () -> -controller.getLeftY() * 5.0, // Max 5 m/s
                () -> -controller.getLeftX() * 5.0));

    // Reset pose to (1, 1, 0°) when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () -> swerveIO.setPose(new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0))),
                    swerveIO)
                .ignoringDisable(true));

    // Stop drivetrain when X button is pressed
    controller.x().onTrue(Commands.runOnce(swerveIO::stop, swerveIO));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
