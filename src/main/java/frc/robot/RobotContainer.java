// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.auto.PathfindingAuto;
// import frc.robot.auto.dashboard.DashboardAutoManager;
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
import frc.robot.subsystems.vision.VisionIOHardwareLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.ShooterSetpoint;
import frc.robot.util.sim.MapleSimSwerveDrivetrain;
// import java.util.Set;
// import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

  @SuppressWarnings("unused")
  private final VisionSubsystem vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Shooter tuning variables for teleop testing
  private double shooterTestRPS = 0.0;
  private double hoodTestAngleRad = Constants.HoodConstants.STOW_POSITION;

  // Dashboard inputs
  // private final LoggedDashboardChooser<Command> autoChooser;

  // Dashboard-driven autonomous system (254/6328 style)
  // private final DashboardAutoManager dashboardAutoManager;

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
    // autoChooser = new LoggedDashboardChooser<>("Auto/Auto Choices");
    // autoChooser.addDefaultOption("Do Nothing", Commands.none());

    // Initialize subsystems with constructor injection
    switch (Constants.currentMode) {
      case REAL:
        intake = new IntakeSubsystem(new IntakeIOTalonFX());
        intakePivot = new IntakePivotSubsystem(new IntakePivotIOTalonFX());
        turret = new TurretSubsystem(new TurretIOTalonFX(), robotState);
        hood = new HoodSubsystem(new HoodIOTalonFX());
        shooter = new ShooterSubsystem(new ShooterIOTalonFX());
        conveyor = new ConveyorSubsystem(new ConveyorIOTalonFX());
        indexer = new IndexerSubsystem(new IndexerIOTalonFX());
        climb = new ClimbSubsystem(new ClimbIOTalonFX());
        break;
      case SIM:
        intake = new IntakeSubsystem(new IntakeIOSim());
        intakePivot = new IntakePivotSubsystem(new IntakePivotIOSim());
        turret = new TurretSubsystem(new TurretIOSim(), robotState);
        hood = new HoodSubsystem(new HoodIOSim());
        shooter = new ShooterSubsystem(new ShooterIOSim());
        conveyor = new ConveyorSubsystem(new ConveyorIOSim());
        indexer = new IndexerSubsystem(new IndexerIOSim());
        climb = new ClimbSubsystem(new ClimbIOSim());
        break;
      default:
        intake = new IntakeSubsystem(new IntakeIO() {});
        intakePivot = new IntakePivotSubsystem(new IntakePivotIO() {});
        turret = new TurretSubsystem(new TurretIO() {}, robotState);
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

    // Instantiate Vision with pose consumer that feeds into the drive's pose estimator
    switch (Constants.currentMode) {
      case REAL:
        vision =
            new VisionSubsystem(
                new VisionIOHardwareLimelight(robotState),
                robotState,
                estimate ->
                    swerveIO
                        .getDriveIO()
                        .addVisionMeasurement(
                            estimate.getVisionRobotPoseMeters(),
                            com.ctre.phoenix6.Utils.fpgaToCurrentTime(
                                estimate.getTimestampSeconds()),
                            estimate.getVisionMeasurementStdDevs()));
        break;
      default:
        // SIM and REPLAY: no-op vision IO
        vision = new VisionSubsystem(inputs -> {}, robotState, estimate -> {});
        break;
    }

    // Initialize SmartDashboard entry for auto sequence input (Team 254 style)
    // Put under Auto subtable for better organization
    // SmartDashboard.putString("Auto/Sequence Input", "");

    // ===== CONFIGURE PATHPLANNER AUTOBUILDER =====
    // Must be done BEFORE creating any auto commands
    // configureAutoBuilder();

    // ===== CONFIGURE AUTO CHOOSER =====

    // ----- Legacy / test autos (PathfindingAuto with hardcoded positions) -----
    // These use placeholder coordinates and no real Superstructure integration.
    // Useful for pathfinding testing only. For matches, use "Dashboard Auto (Generated)".
    // autoChooser.addOption(
    //     "[Test] ABC Sequence",
    //     Commands.defer(
    //         () -> new PathfindingAuto(swerveIO, superstructure, robotState, "ABC"),
    //         Set.of(swerveIO)));

    // autoChooser.addOption(
    //     "[Test] ABCD Full",
    //     Commands.defer(
    //         () -> PathfindingAuto.allWaypoints(swerveIO, superstructure, robotState),
    //         Set.of(swerveIO)));

    // autoChooser.addOption(
    //     "[Test] AB Fast",
    //     Commands.defer(
    //         () -> PathfindingAuto.fastSequence(swerveIO, superstructure, robotState),
    //         Set.of(swerveIO)));

    // autoChooser.addOption(
    //     "[Test] With Intakes",
    //     Commands.defer(
    //         () -> PathfindingAuto.withIntakes(swerveIO, superstructure, robotState),
    //         Set.of(swerveIO)));

    // autoChooser.addOption(
    //     "[Test] Obstacle Avoidance",
    //     Commands.defer(
    //         () -> new PathfindingAuto(swerveIO, superstructure, robotState, "T"),
    //         Set.of(swerveIO)));

    // // Legacy dashboard input (reads a character string like "ABC" from SmartDashboard)
    // autoChooser.addOption(
    //     "[Test] Dashboard Input",
    //     Commands.defer(
    //         () -> {
    //           String sequence =
    //               SmartDashboard.getString("Auto/Sequence Input", "").toUpperCase().trim();
    //           if (sequence.isEmpty()) {
    //             System.err.println(
    //                 "[RobotContainer] No sequence entered in 'Auto/Sequence Input' on
    // SmartDashboard!");
    //             return Commands.none();
    //           }
    //           System.out.println(
    //               "[RobotContainer] Creating auto from dashboard input: " + sequence);
    //           return new PathfindingAuto(swerveIO, superstructure, robotState, sequence);
    //         },
    //         Set.of(swerveIO)));

    // // ----- Match auto: Dashboard-driven auto (REBUILT field-aware, generated at runtime) -----
    // // Configure settings in the "Auto Settings" Shuffleboard tab before each match.
    // dashboardAutoManager = new DashboardAutoManager(swerveIO, superstructure, robotState);
    // autoChooser.addOption(
    //     "Dashboard Auto (Generated)",
    //     Commands.defer(() -> dashboardAutoManager.getAutoCommand(), Set.of(swerveIO)));

    // ===== INTEGRATE SHOOTERSETPOINT UTILITY =====
    // Create a ShooterSetpoint supplier that uses robotState for calculations
    var shooterSetpointSupplier = ShooterSetpoint.createSupplier(robotState);

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
  // private void configureAutoBuilder() {
  //   try {
  //     // Load RobotConfig from GUI settings (now that settings.json is complete)
  //     RobotConfig config = RobotConfig.fromGUISettings();
  //     System.out.println("[RobotContainer] Loaded RobotConfig from GUI settings");

  //     // Configure AutoBuilder with swerve drive
  //     AutoBuilder.configure(
  //         swerveIO::getPose, // Pose supplier
  //         swerveIO::setPose, // Pose reset consumer
  //         swerveIO::getChassisSpeeds, // ChassisSpeeds supplier
  //         (speeds, feedforwards) ->
  //             swerveIO.runVelocity(speeds), // Drive output consumer (ignore feedforwards)
  //         new PPHolonomicDriveController(
  //             new PIDConstants(
  //                 Constants.AutoConstants.kPLTEController, 0.0, 0.0), // Translation PID
  //             new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0) // Rotation
  // PID
  //             ),
  //         config, // Robot configuration
  //         () -> {
  //           // Flip paths for red alliance
  //           var alliance = DriverStation.getAlliance();
  //           return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  //         },
  //         swerveIO // Drive subsystem requirement
  //         );

  //     System.out.println("[RobotContainer] AutoBuilder configured successfully");
  //   } catch (Exception e) {
  //     System.err.println("[RobotContainer] CRITICAL: Failed to configure AutoBuilder!");
  //     System.err.println("  Pathfinding and auto commands will NOT work.");
  //     System.err.println("  Error: " + e.getMessage());
  //     e.printStackTrace();
  //   }
  // }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureTeleopBindings();
  }

  /** Configure button bindings for teleop mode. This is the normal driving configuration. */
  private void configureTeleopBindings() {
    System.out.println("[RobotContainer] Configuring teleop bindings...");

    // ===== DRIVE CONTROLS =====
    // Default command, field-relative drive
    // In WPILib blue-origin coordinates, +X = toward red wall, +Y = toward left (from blue DS).
    // On red alliance the driver faces the opposite direction, so we negate vx and vy.
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

              // Flip field-relative directions for red alliance so "forward" on the
              // joystick always means "away from the driver" regardless of alliance.
              var alliance = DriverStation.getAlliance();
              boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
              double allianceFlip = isRed ? -1.0 : 1.0;

              double vxMetersPerSec =
                  leftY * Constants.DriveConstants.MAX_TELEOP_SPEED_MPS * allianceFlip;
              double vyMetersPerSec =
                  leftX * Constants.DriveConstants.MAX_TELEOP_SPEED_MPS * allianceFlip;
              double omegaRadPerSec =
                  rightX * Constants.DriveConstants.MAX_TELEOP_ANGULAR_SPEED_RAD_PER_SEC;
              swerveIO.driveFieldRelative(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec);
            },
            swerveIO));

    // Left bumper: Reset pose (alliance-aware)
    // In WPILib blue-origin coordinates:
    //   Blue side: robot near (0.33, 0.33) facing 0° (toward red wall)
    //   Red side:  robot near (FIELD_LENGTH - 0.33, FIELD_WIDTH - 0.33) facing 180° (toward blue
    // wall)
    controller
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      var alliance = DriverStation.getAlliance();
                      boolean isRed =
                          alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                      if (isRed) {
                        swerveIO.setPose(
                            new Pose2d(
                                Constants.FieldPoses.FIELD_LENGTH - 0.33,
                                Constants.FieldPoses.FIELD_WIDTH - 0.33,
                                Rotation2d.fromDegrees(180)));
                      } else {
                        swerveIO.setPose(Constants.AutoConstants.DEFAULT_RESET_POSE);
                      }
                    },
                    swerveIO)
                .ignoringDisable(true));

    // ===== SUPERSTRUCTURE CONTROLS =====

    // X button: Aiming while intaking
    controller.x().onTrue(superstructure.onlyAiming());

    controller.a().onTrue(superstructure.onlyIntake());

    // Y button: Stow (idle)
    controller.y().onTrue(superstructure.idle());

    // A button (hold): Shoot while held, return to aiming on release.
    // If in AIMING_WHILE_INTAKING → SHOOTING_WHILE_INTAKING, else → ONLY_SHOOTING
    // On release: return to corresponding aiming state
    controller
        .rightTrigger()
        .whileTrue(
            Commands.either(
                superstructure.shootingWhileIntaking(),
                superstructure.onlyShooting(),
                () ->
                    superstructure.getState()
                        == Superstructure.SuperstructureState.AIMING_WHILE_INTAKING))
        .onFalse(
            Commands.either(
                superstructure.aimingWhileIntaking(),
                superstructure.onlyAiming(),
                () ->
                    superstructure.getState()
                            == Superstructure.SuperstructureState.SHOOTING_WHILE_INTAKING
                        || superstructure.getState()
                            == Superstructure.SuperstructureState.AIMING_WHILE_INTAKING));

    // ===== CLIMB STATE CONTROLS =====

    // POV Up: Enter climb mode
    // controller.povUp().onTrue(superstructure.enterClimbMode());

    // POV Down: Exit climb mode
    // controller.povDown().onTrue(superstructure.exitClimbMode());

    // POV Right: Next climb state (path-following)
    // controller.povRight().onTrue(superstructure.nextClimbState());

    // POV Left: Previous climb state (path-following)
    // controller.povLeft().onTrue(superstructure.previousClimbState());

    // ===== CLIMB TEST CONTROLS =====

    // X button: Run path to reach vertically down 0.15m from stow
    // controller
    //    .x()
    //    .onTrue(
    //        climb.runPathToPosition(
    //            new Translation2d(
    //                Constants.ClimbConstants.START_POSITION_X_METERS,
    //                Constants.ClimbConstants.START_POSITION_Y_METERS - 0.15),
    //            2.0,
    //            false));

    // Y button: Stow climb (return to stowed position)
    // controller.y().onTrue(climb.setStateCommand(ClimbState.STOWED));
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
    // Command selectedCommand = autoChooser.get();
    // if (selectedCommand != null) {
    //   System.out.println("[RobotContainer] Auto command selected: " + selectedCommand.getName());
    // } else {
    //   System.out.println("[RobotContainer] WARNING: No auto command selected!");
    // }
    // return selectedCommand;
    return Commands.none();
  }

  /**
   * Called during disabled periodic to keep the dashboard auto manager up to date. Reads settings
   * from the dashboard and replans if anything changed. Also handles PathPlanner warmup.
   */
  public void runAutoWarmup() {
    // Update dashboard auto manager — reads settings and replans if changed
    // dashboardAutoManager.update();
  }
}
