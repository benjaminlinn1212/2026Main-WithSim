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
import frc.robot.commands.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.Superstructure;
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
  private final IntakeSubsystem intake;
  private final IntakePivotSubsystem intakePivot;
  private final HoodSubsystem hood;
  private final ShooterSubsystem shooter;
  private final ConveyorSubsystem conveyor;
  private final Superstructure superstructure;

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

    // Initialize subsystems
    intake = IntakeSubsystem.getInstance();
    intakePivot = IntakePivotSubsystem.getInstance();
    conveyor = ConveyorSubsystem.getInstance();
    shooter = ShooterSubsystem.getInstance();

    // Initialize hood based on mode
    switch (Constants.currentMode) {
      case REAL:
        hood = new HoodSubsystem(new HoodIOTalonFX());
        break;
      case SIM:
        hood = new HoodSubsystem(new HoodIOSim());
        break;
      default:
        hood = new HoodSubsystem(new HoodIO() {});
        break;
    }

    // Provide robot pose to hood for distance-based aiming
    hood.setRobotPoseSupplier(() -> swerveIO.getPose());

    // Initialize superstructure
    superstructure = new Superstructure(shooter, hood, intake, intakePivot, conveyor);

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
              // Apply deadband to prevent drift from joystick noise
              double leftY =
                  edu.wpi.first.math.MathUtil.applyDeadband(
                      -controller.getLeftY(), Constants.DriveConstants.JOYSTICK_DEADBAND);
              double leftX =
                  edu.wpi.first.math.MathUtil.applyDeadband(
                      -controller.getLeftX(), Constants.DriveConstants.JOYSTICK_DEADBAND);
              double rightX =
                  edu.wpi.first.math.MathUtil.applyDeadband(
                      -controller.getRightX(), Constants.DriveConstants.JOYSTICK_DEADBAND);

              double vxMetersPerSec = leftY * 5.0; // Max 5 m/s
              double vyMetersPerSec = leftX * 5.0;
              double omegaRadPerSec = rightX * Math.PI; // Max PI rad/s
              swerveIO.driveFieldRelative(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec);
            },
            swerveIO));

    // === SUPERSTRUCTURE CONTROLS ===
    // A button: Switch to intake mode
    controller.a().onTrue(superstructure.intake());

    // X button: Prepare to shoot (spin up shooter, aim hood, stow intake)
    controller.x().onTrue(superstructure.prepareShoot());

    // Right trigger: Shoot while held, return to prepare shoot when released
    controller
        .rightTrigger()
        .whileTrue(superstructure.shoot())
        .onFalse(superstructure.prepareShoot());

    // B button: Emergency stop
    controller.b().onTrue(superstructure.emergencyStop());

    // Y button: Stow everything (idle state)
    controller.y().onTrue(superstructure.idle());
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
