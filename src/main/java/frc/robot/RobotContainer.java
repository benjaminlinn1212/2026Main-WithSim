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
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.turret.TurretSubsystem;
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
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;

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

    // Initialize turret and hood based on mode
    switch (Constants.currentMode) {
      case REAL:
        turret = new TurretSubsystem(new TurretIOTalonFX());
        hood = new HoodSubsystem(new HoodIOTalonFX());
        break;
      case SIM:
        turret = new TurretSubsystem(new TurretIOSim());
        hood = new HoodSubsystem(new HoodIOSim());
        break;
      default:
        turret = new TurretSubsystem(new TurretIO() {});
        hood = new HoodSubsystem(new HoodIO() {});
        break;
    }

    // Provide robot pose and chassis speeds to turret for field-relative tracking
    turret.setRobotPoseSupplier(() -> swerveIO.getPose());
    turret.setChassisSpeedsSupplier(() -> robotState.getLatestMeasuredFieldRelativeChassisSpeeds());

    // Provide robot pose to hood for distance-based aiming
    hood.setRobotPoseSupplier(() -> swerveIO.getPose());

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

    // Drive to test pose when Y button is held
    controller.y().whileTrue(new DriveToPose(swerveIO, Constants.FieldPoses.TEST));

    // === INTAKE SYSTEM CONTROLS ===
    // Right bumper: Deploy intake pivot AND run intake rollers
    controller.rightBumper().onTrue(intakePivot.deploy()).onTrue(intake.intake());

    // Left bumper: Stow intake pivot AND stop intake rollers
    controller.leftBumper().onTrue(intakePivot.stow()).onTrue(intake.stop());

    // Reset pose to (1, 1, 0°) when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () -> swerveIO.setPose(new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0))),
                    swerveIO)
                .ignoringDisable(true));

    // Emergency stop: X button stops all intake system and drivetrain
    controller
        .x()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      swerveIO.stop();
                      intake.stopMotor();
                      intakePivot.stopMotor();
                    })
                .withName("EmergencyStop"));

    // Move intake pivot to stow position on emergency stop (X button)
    controller.x().onTrue(intakePivot.stow());

    // === SMART TURRET AIMING ===
    // Left trigger: Auto-aim turret intelligently
    // - If in neutral zone (crossed target X): shootBackFromNeutralZone
    // - Else: aimHub at hub
    // - Release trigger: stow
    controller
        .leftTrigger()
        .whileTrue(
            Commands.either(
                // If robot crossed into neutral zone (past target X), shoot back
                turret.shootBackFromNeutralZone(),
                // Otherwise aim at hub
                turret.aimHub(),
                // Condition: check if robot X is past the target X (neutral zone)
                () -> {
                  double robotX = swerveIO.getPose().getX();
                  var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
                  if (alliance.isPresent()
                      && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                    // Red alliance: past neutral zone if X < red target X
                    return robotX < Constants.FieldPoses.RED_AIM_TARGET.getX();
                  } else {
                    // Blue alliance: past neutral zone if X > blue target X
                    return robotX > Constants.FieldPoses.BLUE_AIM_TARGET.getX();
                  }
                }))
        .onFalse(turret.stow()); // Stow when trigger released
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
