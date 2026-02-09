// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.PathfindingAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.conveyor.ConveyorIO;
import frc.robot.subsystems.conveyor.ConveyorIOSim;
import frc.robot.subsystems.conveyor.ConveyorIOTalonFX;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakepivot.IntakePivotIO;
import frc.robot.subsystems.intakepivot.IntakePivotIOSim;
import frc.robot.subsystems.intakepivot.IntakePivotIOTalonFX;
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.ShooterSetpoint;
import frc.robot.util.sim.MapleSimSwerveDrivetrain;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
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
  private final ShooterSubsystem shooter;
  private final ConveyorSubsystem conveyor;
  private final IndexerSubsystem indexer;
  private final ClimbSubsystem climb;
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Test mode tuning variables
  private double testModeShooterRPS = 0.0;
  private double testModeHoodAngleRad = Constants.HoodConstants.STOW_POSITION;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Store PathfindingAuto instances for warmup (mapped by the command they create)
  private final Map<String, PathfindingAuto> pathfindingAutos = new HashMap<>();
  private Command lastSelectedCommand = null;
  private Command lastWarmupCommand = null;

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

    // Set up auto routines under SmartDashboard/Auto subtable
    autoChooser = new LoggedDashboardChooser<>("Auto/Auto Choices");
    autoChooser.addDefaultOption("Do Nothing", Commands.none());

    // Initialize subsystems with constructor injection
    switch (Constants.currentMode) {
      case REAL:
        intake = new IntakeSubsystem(new IntakeIOTalonFX());
        intakePivot = new IntakePivotSubsystem(new IntakePivotIOTalonFX());
        turret = new TurretSubsystem(new TurretIOTalonFX());
        hood = new HoodSubsystem(new HoodIOTalonFX());
        shooter = new ShooterSubsystem(new ShooterIOTalonFX());
        conveyor = new ConveyorSubsystem(new ConveyorIOTalonFX());
        indexer = new IndexerSubsystem(new IndexerIOTalonFX());
        climb = new ClimbSubsystem(new ClimbIOTalonFX());
        break;
      case SIM:
        intake = new IntakeSubsystem(new IntakeIOSim());
        intakePivot = new IntakePivotSubsystem(new IntakePivotIOSim());
        turret = new TurretSubsystem(new TurretIOSim());
        hood = new HoodSubsystem(new HoodIOSim());
        shooter = new ShooterSubsystem(new ShooterIOSim());
        conveyor = new ConveyorSubsystem(new ConveyorIOSim());
        indexer = new IndexerSubsystem(new IndexerIOSim());
        climb = new ClimbSubsystem(new ClimbIOSim());
        break;
      default:
        intake = new IntakeSubsystem(new IntakeIO() {});
        intakePivot = new IntakePivotSubsystem(new IntakePivotIO() {});
        turret = new TurretSubsystem(new TurretIO() {});
        hood = new HoodSubsystem(new HoodIO() {});
        shooter = new ShooterSubsystem(new ShooterIO() {});
        conveyor = new ConveyorSubsystem(new ConveyorIO() {});
        indexer = new IndexerSubsystem(new IndexerIO() {});
        climb = new ClimbSubsystem(new ClimbIO() {});
        break;
    }

    // Instantiate Superstructure with all subsystems
    superstructure =
        new Superstructure(shooter, turret, hood, intake, intakePivot, conveyor, indexer, climb);

    // Initialize SmartDashboard entry for auto sequence input (Team 254 style)
    // Put under Auto subtable for better organization
    SmartDashboard.putString("Auto/Sequence Input", "");

    // ===== CONFIGURE PATHPLANNER AUTOBUILDER =====
    // Must be done BEFORE creating any auto commands
    configureAutoBuilder();

    // ===== CONFIGURE AUTO CHOOSER =====
    // Add sequence-based autos to chooser (using 254-style static factory methods)
    // Use Commands.defer() to delay command creation until auto is selected
    // Store PathfindingAuto instances for warmup
    PathfindingAuto abcAuto = new PathfindingAuto(swerveIO, superstructure, robotState, "ABC");
    pathfindingAutos.put("ABC Sequence", abcAuto);
    autoChooser.addOption(
        "ABC Sequence",
        Commands.defer(
            () -> new PathfindingAuto(swerveIO, superstructure, robotState, "ABC"),
            Set.of(swerveIO)));

    PathfindingAuto abcdAuto = PathfindingAuto.allWaypoints(swerveIO, superstructure, robotState);
    pathfindingAutos.put("ABCD Full", abcdAuto);
    autoChooser.addOption(
        "ABCD Full",
        Commands.defer(
            () -> PathfindingAuto.allWaypoints(swerveIO, superstructure, robotState),
            Set.of(swerveIO)));

    PathfindingAuto abFastAuto = PathfindingAuto.fastSequence(swerveIO, superstructure, robotState);
    pathfindingAutos.put("AB Fast", abFastAuto);
    autoChooser.addOption(
        "AB Fast",
        Commands.defer(
            () -> PathfindingAuto.fastSequence(swerveIO, superstructure, robotState),
            Set.of(swerveIO)));

    PathfindingAuto withIntakesAuto =
        PathfindingAuto.withIntakes(swerveIO, superstructure, robotState);
    pathfindingAutos.put("With Intakes", withIntakesAuto);
    autoChooser.addOption(
        "With Intakes",
        Commands.defer(
            () -> PathfindingAuto.withIntakes(swerveIO, superstructure, robotState),
            Set.of(swerveIO)));

    // ===== TEST AUTOS =====
    PathfindingAuto testAuto = new PathfindingAuto(swerveIO, superstructure, robotState, "T");
    pathfindingAutos.put("Test Obstacle Avoidance", testAuto);
    autoChooser.addOption(
        "Test Obstacle Avoidance",
        Commands.defer(
            () -> {
              System.out.println("[RobotContainer] Creating Test Obstacle Avoidance command...");
              return new PathfindingAuto(swerveIO, superstructure, robotState, "T");
            },
            Set.of(swerveIO)));

    // ===== DASHBOARD INPUT AUTO (Team 254 Style) =====
    // This reads from SmartDashboard text input and generates PathfindingAuto dynamically
    autoChooser.addOption(
        "Dashboard Input",
        Commands.defer(
            () -> {
              String sequence =
                  SmartDashboard.getString("Auto/Sequence Input", "").toUpperCase().trim();
              if (sequence.isEmpty()) {
                System.err.println(
                    "[RobotContainer] No sequence entered in 'Auto/Sequence Input' on SmartDashboard!");
                return Commands.none();
              }
              System.out.println(
                  "[RobotContainer] Creating auto from dashboard input: " + sequence);
              return new PathfindingAuto(swerveIO, superstructure, robotState, sequence);
            },
            Set.of(swerveIO)));

    // ===== INTEGRATE SHOOTERSETPOINT UTILITY =====
    // Create a ShooterSetpoint supplier that uses robotState for calculations
    var shooterSetpointSupplier = ShooterSetpoint.speakerSetpointSupplier(robotState);

    // Connect all aiming subsystems to use the same ShooterSetpoint
    turret.setShooterSetpointSupplier(shooterSetpointSupplier);
    hood.setShooterSetpointSupplier(shooterSetpointSupplier);
    shooter.setShooterSetpointSupplier(shooterSetpointSupplier);

    // Provide robot pose for turret's field-relative tracking
    turret.setRobotPoseSupplier(() -> swerveIO.getPose());

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Configure PathPlanner AutoBuilder for pathfinding and auto paths. */
  private void configureAutoBuilder() {
    try {
      // Load RobotConfig from GUI settings (now that settings.json is complete)
      RobotConfig config = RobotConfig.fromGUISettings();
      System.out.println("[RobotContainer] Loaded RobotConfig from GUI settings");

      // Configure AutoBuilder with swerve drive
      AutoBuilder.configure(
          swerveIO::getPose, // Pose supplier
          swerveIO::setPose, // Pose reset consumer
          swerveIO::getChassisSpeeds, // ChassisSpeeds supplier
          (speeds, feedforwards) ->
              swerveIO.runVelocity(speeds), // Drive output consumer (ignore feedforwards)
          new PPHolonomicDriveController(
              new PIDConstants(
                  Constants.AutoConstants.kPLTEController, 0.0, 0.0), // Translation PID
              new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0) // Rotation PID
              ),
          config, // Robot configuration
          () ->
              false, // Should flip path based on alliance (set to true if you want red alliance to
          // mirror)
          swerveIO // Drive subsystem requirement
          );

      System.out.println("[RobotContainer] AutoBuilder configured successfully");
    } catch (Exception e) {
      System.err.println("[RobotContainer] CRITICAL: Failed to configure AutoBuilder!");
      System.err.println("  Pathfinding and auto commands will NOT work.");
      System.err.println("  Error: " + e.getMessage());
      e.printStackTrace();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ===== DRIVE CONTROLS =====
    // Default command, field-relative drive
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
              double omegaRadPerSec = rightX * Math.PI * 2;
              swerveIO.driveFieldRelative(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec);
            },
            swerveIO));

    // Right bumper: Reset pose
    controller
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                    () -> swerveIO.setPose(new Pose2d(8.0, 4.0, Rotation2d.fromDegrees(0))),
                    swerveIO)
                .ignoringDisable(true));

    // ===== CLIMB TEST CONTROLS =====

    // A button: Go to STOWED state
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  climb.setState(frc.robot.subsystems.climb.ClimbState.STOWED);
                  System.out.println("[Climb Test] Set to STOWED state");
                },
                climb));

    // B button: Test manual position control
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  climb.setSymmetricTargetPosition(
                      new edu.wpi.first.math.geometry.Translation2d(0.6, 0.8));
                  System.out.println("[Climb Test] Manual position (0.6, 0.8)");
                },
                climb));

    // X button: Cycle through climb states (forward)
    controller.x().onTrue(climb.nextState());

    // Y button: Emergency stop climb
    controller.y().onTrue(climb.emergencyStop());

    // D-Pad Right: Next climb state
    controller.povRight().onTrue(climb.nextState());

    // D-Pad Left: Previous climb state (using REVERSED path for smooth backwards motion)
    controller.povLeft().onTrue(climb.previousStateReversed());

    controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  System.out.println("[Test Mode] Starting ClimbWorkflow fullAutoClimb test...");
                  frc.robot.subsystems.climb.ClimbWorkflow.fullAutoClimb(superstructure, swerveIO)
                      .schedule();
                }));

    // Left bumper: Stop all climb motors
    controller.leftBumper().onTrue(Commands.runOnce(() -> climb.stopMotors(), climb));
  }

  /**
   * Configure button bindings for test mode. This allows tuning of shooter RPS and hood angle while
   * keeping drivetrain operational.
   */
  public void configureTestModeBindings() {
    // Cancel all running commands
    CommandScheduler.getInstance().cancelAll();

    System.out.println("[RobotContainer] Configuring test mode bindings...");

    // ===== DRIVE CONTROLS (same as teleop) =====
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
              double omegaRadPerSec = rightX * Math.PI * 2;
              swerveIO.driveFieldRelative(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec);
            },
            swerveIO));

    // ===== TEST MODE TUNING CONTROLS =====

    // A button: Increase Shooter RPS
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  testModeShooterRPS += Constants.ShooterConstants.TEST_MODE_RPS_INCREMENT;
                  shooter.setVelocity(testModeShooterRPS);
                  System.out.println("[Test Mode] Shooter RPS increased to: " + testModeShooterRPS);
                  SmartDashboard.putNumber("TestMode/ShooterRPS", testModeShooterRPS);
                }));

    // B button: Decrease Shooter RPS
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  testModeShooterRPS -= Constants.ShooterConstants.TEST_MODE_RPS_INCREMENT;
                  testModeShooterRPS = Math.max(0, testModeShooterRPS); // Don't go negative
                  shooter.setVelocity(testModeShooterRPS);
                  System.out.println("[Test Mode] Shooter RPS decreased to: " + testModeShooterRPS);
                  SmartDashboard.putNumber("TestMode/ShooterRPS", testModeShooterRPS);
                }));

    // X button: Increase Hood Angle
    controller
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  testModeHoodAngleRad += Constants.HoodConstants.TEST_MODE_ANGLE_INCREMENT;
                  testModeHoodAngleRad =
                      Math.min(
                          testModeHoodAngleRad,
                          Constants.HoodConstants.MAX_POSITION_RAD); // Don't exceed max position
                  hood.positionSetpointCommand(() -> testModeHoodAngleRad, () -> 0.0).schedule();
                  System.out.println(
                      "[Test Mode] Hood angle increased to: "
                          + Math.toDegrees(testModeHoodAngleRad)
                          + " degrees");
                  SmartDashboard.putNumber(
                      "TestMode/HoodAngleDeg", Math.toDegrees(testModeHoodAngleRad));
                }));

    // Y button: Decrease Hood Angle
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  testModeHoodAngleRad -= Constants.HoodConstants.TEST_MODE_ANGLE_INCREMENT;
                  testModeHoodAngleRad =
                      Math.max(
                          testModeHoodAngleRad,
                          Constants.HoodConstants.MIN_POSITION_RAD); // Don't go below min position
                  hood.positionSetpointCommand(() -> testModeHoodAngleRad, () -> 0.0).schedule();
                  System.out.println(
                      "[Test Mode] Hood angle decreased to: "
                          + Math.toDegrees(testModeHoodAngleRad)
                          + " degrees");
                  SmartDashboard.putNumber(
                      "TestMode/HoodAngleDeg", Math.toDegrees(testModeHoodAngleRad));
                }));

    // Left bumper: Stop shooter
    controller
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  testModeShooterRPS = 0.0;
                  shooter.stop();
                  System.out.println("[Test Mode] Shooter stopped");
                  SmartDashboard.putNumber("TestMode/ShooterRPS", testModeShooterRPS);
                }));

    // Right bumper: Reset hood to stow position
    controller
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  testModeHoodAngleRad = Constants.HoodConstants.STOW_POSITION;
                  hood.stow().schedule();
                  System.out.println("[Test Mode] Hood reset to stow position");
                  SmartDashboard.putNumber(
                      "TestMode/HoodAngleDeg", Math.toDegrees(testModeHoodAngleRad));
                }));

    // Initialize SmartDashboard values
    SmartDashboard.putNumber("TestMode/ShooterRPS", testModeShooterRPS);
    SmartDashboard.putNumber("TestMode/HoodAngleDeg", Math.toDegrees(testModeHoodAngleRad));

    System.out.println("[RobotContainer] Test mode bindings configured!");
    System.out.println("  A: Increase Shooter RPS");
    System.out.println("  B: Decrease Shooter RPS");
    System.out.println("  X: Increase Hood Angle");
    System.out.println("  Y: Decrease Hood Angle");
    System.out.println("  Left Bumper: Stop Shooter");
    System.out.println("  Right Bumper: Reset Hood to Stow");
    System.out.println("  Back: Test ClimbWorkflow Full Auto Climb");
    System.out.println("  Left Stick: Drive (field-relative)");
    System.out.println("  Right Stick X: Rotate");
  }

  /**
   * Sets the starting pose for the swerve drive (used in simulation).
   *
   * @param pose The starting pose
   */
  public void setSwerveStartingPose(edu.wpi.first.math.geometry.Pose2d pose) {
    swerveIO.setPose(pose);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selectedCommand = autoChooser.get();
    if (selectedCommand != null) {
      System.out.println("[RobotContainer] Auto command selected: " + selectedCommand.getName());
    } else {
      System.out.println("[RobotContainer] WARNING: No auto command selected!");
    }
    return selectedCommand;
  }

  /**
   * Run warmup for the selected autonomous command. Called during disabledPeriodic to preload
   * pathfinding computations while the robot is disabled.
   */
  public void runAutoWarmup() {
    // Get the currently selected command
    Command selectedCommand = autoChooser.get();

    // Only warmup if selection changed
    if (selectedCommand == null || selectedCommand == lastSelectedCommand) {
      return;
    }

    lastSelectedCommand = selectedCommand;

    // Find the matching PathfindingAuto by trying all stored ones
    // This is a workaround since we can't easily get the name from LoggedDashboardChooser
    for (Map.Entry<String, PathfindingAuto> entry : pathfindingAutos.entrySet()) {
      PathfindingAuto auto = entry.getValue();
      String autoName = entry.getKey();

      // Get warmup command and schedule it
      Command warmupCommand = auto.getWarmupCommand();
      if (warmupCommand != null) {
        // Cancel previous warmup if it's still running
        if (lastWarmupCommand != null && lastWarmupCommand.isScheduled()) {
          lastWarmupCommand.cancel();
        }

        // Schedule new warmup command (runs .ignoringDisable(true))
        System.out.println("[RobotContainer] Starting warmup for: " + autoName);
        CommandScheduler.getInstance().schedule(warmupCommand);
        lastWarmupCommand = warmupCommand;
        break; // Only warmup one auto at a time
      }
    }
  }
}
