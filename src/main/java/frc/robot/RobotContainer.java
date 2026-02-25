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
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.HardcodedAutos;
import frc.robot.auto.dashboard.DashboardAutoManager;
import frc.robot.auto.dashboard.FieldConstants;
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
import java.util.Set;
import org.littletonrobotics.junction.Logger;
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

  @SuppressWarnings("unused")
  private final VisionSubsystem vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Dashboard-driven autonomous system (254/6328 style)
  private DashboardAutoManager dashboardAutoManager;

  // Hardcoded fallback autos (one per lane)
  private HardcodedAutos hardcodedAutos;

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

    // ===== CONFIGURE PATHPLANNER AUTOBUILDER =====
    // Must be done BEFORE creating any auto commands
    configureAutoBuilder();

    // ===== CONFIGURE AUTO MODE SELECTOR (254-style) =====
    // Single source of truth: DashboardAutoManager owns all auto planning.
    // The planner reads field-aware settings from Shuffleboard, generates an
    // optimal action sequence, and builds the command tree at autonomousInit().
    // Configure settings in the "Auto Settings" Shuffleboard tab before each match.
    dashboardAutoManager = new DashboardAutoManager(swerveIO, superstructure, climb);
    autoChooser.addOption(
        "Dashboard Auto",
        Commands.defer(() -> dashboardAutoManager.getAutoCommand(), Set.of(swerveIO)));

    // Hardcoded fallback autos — pick lane via sub-chooser on dashboard
    hardcodedAutos = new HardcodedAutos(swerveIO, superstructure, robotState);
    autoChooser.addOption("Hardcoded Auto", hardcodedAutos.getCommand());

    // ===== INTEGRATE SHOOTERSETPOINT UTILITY =====
    // Create a ShooterSetpoint supplier that uses robotState for calculations
    var shooterSetpointSupplier = ShooterSetpoint.createSupplier(robotState);

    // Connect all aiming subsystems to use the same ShooterSetpoint
    turret.setShooterSetpointSupplier(shooterSetpointSupplier);
    hood.setShooterSetpointSupplier(shooterSetpointSupplier);
    shooter.setShooterSetpointSupplier(shooterSetpointSupplier);

    // Provide robot pose for turret's field-relative tracking
    turret.setRobotPoseSupplier(() -> swerveIO.getPose());

    // Provide robot pose for trench detection (hood stow override)
    superstructure.setRobotPoseSupplier(() -> swerveIO.getPose());

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Configure PathPlanner AutoBuilder for pathfinding and auto paths. */
  @SuppressWarnings("deprecation")
  private void configureAutoBuilder() {
    try {
      // Use AD* pathfinder (PathPlannerLib's LocalADStar) for real-time pathfinding
      // This uses the navgrid.json for obstacle avoidance
      Pathfinding.setPathfinder(new LocalADStar());
      System.out.println("[RobotContainer] AD* pathfinder set (LocalADStar)");

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
          () -> {
            // Flip paths for red alliance
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          swerveIO // Drive subsystem requirement
          );

      System.out.println("[RobotContainer] AutoBuilder configured successfully");

      // ===== Trench Heading Override =====
      // When the robot is inside/near a TRENCH, override PathPlanner's rotation target
      // to the nearest cardinal direction (0/90/180/270°). This ensures the robot
      // maintains the correct heading DURING transit through the 22.25in tunnel,
      // not just at the destination.
      PPHolonomicDriveController.setRotationTargetOverride(
          () -> {
            Pose2d pose = swerveIO.getPose();
            if (FieldConstants.isNearTrench(pose.getTranslation())) {
              return java.util.Optional.of(FieldConstants.snapToCardinal(pose.getRotation()));
            }
            return java.util.Optional.empty();
          });
      System.out.println("[RobotContainer] Trench rotation override configured");
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
    configureTeleopBindings();
    configureOperatorBindings();
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

              // ===== Trench Assist =====
              // Two effects when near a trench:
              // 1. Chassis orientation alignment — injects omega to rotate bumpers toward
              //    the nearest cardinal so the robot physically fits through the tunnel.
              // 2. Lateral centering — deflects velocity direction toward the trench's
              //    center Y line, guiding the travel path to the middle of the corridor.
              Translation2d bluePos = swerveIO.getPose().getTranslation();
              Rotation2d robotHeading = swerveIO.getPose().getRotation();
              // Convert to blue-origin if on red alliance (field geometry is defined in blue)
              if (isRed) {
                bluePos = FieldConstants.flipTranslation(bluePos);
                // Flip heading too: red heading is 180° offset from blue
                robotHeading = robotHeading.plus(Rotation2d.fromDegrees(180));
              }

              // 1. Orientation alignment — adds omega correction to rotate chassis to cardinal
              //    Reuses DriveToPose.ROTATION_KP — no separate heading gain needed.
              double trenchBuffer = Constants.DriveConstants.TrenchAssist.APPROACH_BUFFER;
              double orientationOmega =
                  FieldConstants.getTrenchOrientationOmega(
                      bluePos,
                      robotHeading,
                      vxMetersPerSec,
                      vyMetersPerSec,
                      Constants.DriveConstants.TrenchAssist.MAX_BLEND_FACTOR,
                      Constants.DriveConstants.TrenchAssist.MIN_SPEED_MPS,
                      Constants.DriveConstants.TrenchAssist.MAX_HEADING_ERROR_DEG,
                      Constants.DriveConstants.DriveToPose.ROTATION_KP,
                      Constants.DriveConstants.TrenchAssist.MAX_ORIENTATION_OMEGA_RAD_PER_SEC,
                      trenchBuffer);
              omegaRadPerSec += orientationOmega;

              // 2. Lateral centering — deflects velocity toward trench center Y
              //    + Wall avoidance — prevents bumpers from touching trench walls
              double[] assisted =
                  FieldConstants.applyTrenchAssist(
                      bluePos,
                      vxMetersPerSec,
                      vyMetersPerSec,
                      Constants.DriveConstants.TrenchAssist.MAX_BLEND_FACTOR,
                      Constants.DriveConstants.TrenchAssist.MIN_SPEED_MPS,
                      Constants.DriveConstants.TrenchAssist.MAX_HEADING_ERROR_DEG,
                      Constants.DriveConstants.TrenchAssist.CENTERING_DEG_PER_METER,
                      Constants.DriveConstants.TrenchAssist.MAX_CENTERING_DEG,
                      trenchBuffer,
                      Constants.DriveConstants.TrenchAssist.ROBOT_HALF_WIDTH_M,
                      Constants.DriveConstants.TrenchAssist.WALL_REPULSION_MPS_PER_METER,
                      Constants.DriveConstants.TrenchAssist.WALL_DANGER_ZONE_M);
              vxMetersPerSec = assisted[0];
              vyMetersPerSec = assisted[1];

              // Log trench assist telemetry
              double blendFactor =
                  FieldConstants.getTrenchBlendFactor(
                      bluePos,
                      Constants.DriveConstants.TrenchAssist.MAX_BLEND_FACTOR,
                      trenchBuffer);
              Logger.recordOutput("Drive/TrenchAssist/BlendFactor", blendFactor);
              Logger.recordOutput("Drive/TrenchAssist/Active", blendFactor > 1e-4);
              Logger.recordOutput("Drive/TrenchAssist/CenteringDeg", assisted[2]);
              Logger.recordOutput("Drive/TrenchAssist/OrientationOmega", orientationOmega);

              swerveIO.driveFieldRelative(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec);
            },
            swerveIO));

    // Left bumper: Reset orientation (REAL) or full pose (SIM)
    // SIM: reset to DEFAULT_RESET_POSE for repeatable testing
    // REAL/REPLAY: zero heading alliance-aware (0° blue, 180° red) but preserve XY
    controller
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (Constants.currentMode == Constants.Mode.SIM) {
                        swerveIO.setPose(Constants.AutoConstants.DEFAULT_RESET_POSE);
                      } else {
                        var alliance = DriverStation.getAlliance();
                        boolean isRed =
                            alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                        Pose2d current = swerveIO.getPose();
                        swerveIO.setPose(
                            new Pose2d(
                                current.getTranslation(),
                                Rotation2d.fromDegrees(isRed ? 180.0 : 0.0)));
                      }
                    },
                    swerveIO)
                .ignoringDisable(true));

    // ===== SUPERSTRUCTURE STATE CONTROLS =====

    // Right bumper: IDLE — stow everything, stop all scoring mechanisms
    controller.rightBumper().onTrue(superstructure.idle());

    // A button: ONLY_INTAKE — deploy intake, everything else stowed
    controller.a().onTrue(superstructure.onlyIntake());

    // Y button: ONLY_AIMING — turret/hood/shooter aim at target, intake stowed
    controller.y().onTrue(superstructure.onlyAiming());

    // X button: AIMING_WHILE_INTAKING — intake + aim simultaneously
    controller.x().onTrue(superstructure.aimingWhileIntaking());

    // Right trigger: While held, feed to shooter (convey + index) when in aiming modes.
    // ONLY_AIMING → ONLY_SHOOTING, AIMING_WHILE_INTAKING → SHOOTING_WHILE_INTAKING.
    // On release, reverts to the aiming-only state.
    controller
        .rightTrigger(0.2)
        .whileTrue(
            Commands.run(
                () -> {
                  var state = superstructure.getState();
                  if (state == Superstructure.SuperstructureState.ONLY_AIMING
                      || state == Superstructure.SuperstructureState.ONLY_SHOOTING) {
                    superstructure.forceWantedState(
                        Superstructure.SuperstructureState.ONLY_SHOOTING);
                  } else if (state == Superstructure.SuperstructureState.AIMING_WHILE_INTAKING
                      || state == Superstructure.SuperstructureState.SHOOTING_WHILE_INTAKING) {
                    superstructure.forceWantedState(
                        Superstructure.SuperstructureState.SHOOTING_WHILE_INTAKING);
                  }
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  var state = superstructure.getState();
                  if (state == Superstructure.SuperstructureState.ONLY_SHOOTING) {
                    superstructure.forceWantedState(Superstructure.SuperstructureState.ONLY_AIMING);
                  } else if (state == Superstructure.SuperstructureState.SHOOTING_WHILE_INTAKING) {
                    superstructure.forceWantedState(
                        Superstructure.SuperstructureState.AIMING_WHILE_INTAKING);
                  }
                }));
  }

  /**
   * Configure button bindings for the operator controller (port 1). Controls climb state machine,
   * intake shake (dislodge stuck fuel), and climb calibration mode.
   *
   * <p>Climb is an independent subsystem — operator can control it at any time without entering a
   * special mode. Superstructure continues running (intake, aiming, etc.) in parallel.
   *
   * <p>CALIBRATION MODE (left bumper enter, right bumper exit): Allows individual motor voltage
   * control for cable tension adjustment. On exit, encoder positions are re-seeded to match the
   * initial STOWED end-effector pose.
   *
   * <p>Calibration controls (while in calibration mode):
   *
   * <ul>
   *   <li>POV Up/Down: Left front motor ±3V
   *   <li>POV Left/Right: Left back motor ±3V
   *   <li>Y/A: Right front motor ±3V
   *   <li>X/B: Right back motor ±3V
   * </ul>
   */
  private void configureOperatorBindings() {
    System.out.println("[RobotContainer] Configuring operator bindings...");

    // ===== CLIMB CALIBRATION MODE =====
    // Left bumper: Enter calibration mode — stops motors, enables individual voltage control
    operator
        .leftBumper()
        .onTrue(
            Commands.either(
                Commands.none(), climb.enterCalibrationMode(), climb::isInCalibrationMode));

    // Right bumper: Exit calibration mode — stops motors, recalibrates encoders, returns to STOWED
    operator
        .rightBumper()
        .onTrue(
            Commands.either(
                climb.exitCalibrationMode(), Commands.none(), climb::isInCalibrationMode));

    // ===== CALIBRATION MOTOR CONTROLS (only active in calibration mode) =====
    // POV Up/Down: Left front motor ±3V
    operator
        .povUp()
        .whileTrue(
            Commands.either(
                climb.calibrationLeftFrontForward(),
                climb.releaseFromAutoL1(),
                climb::isInCalibrationMode));

    operator
        .povDown()
        .whileTrue(
            Commands.either(
                climb.calibrationLeftFrontReverse(),
                climb.stowFromCurrentState(),
                climb::isInCalibrationMode));

    // POV Left/Right: Left back motor ±3V
    operator
        .povLeft()
        .whileTrue(
            Commands.either(
                climb.calibrationLeftBackReverse(),
                climb.previousState(),
                climb::isInCalibrationMode));

    operator
        .povRight()
        .whileTrue(
            Commands.either(
                climb.calibrationLeftBackForward(),
                climb.nextClimbStep(),
                climb::isInCalibrationMode));

    // Y/A: Right front motor ±3V
    operator
        .y()
        .whileTrue(
            Commands.either(
                climb.calibrationRightFrontForward(),
                Commands.either(
                    superstructure.setIntakeHalfDeploy(),
                    Commands.none(),
                    superstructure::isIntakeDeployed),
                climb::isInCalibrationMode));

    operator
        .a()
        .whileTrue(
            Commands.either(
                climb.calibrationRightFrontReverse(),
                Commands.either(
                    superstructure.setIntakeFullDeploy(),
                    Commands.none(),
                    superstructure::isIntakeDeployed),
                climb::isInCalibrationMode));

    // X/B: Right back motor ±3V (whileTrue) in calibration mode;
    //       X: all servos to 0 (onTrue) in normal mode
    operator.x().and(climb::isInCalibrationMode).whileTrue(climb.calibrationRightBackForward());
    operator.x().and(() -> !climb.isInCalibrationMode()).onTrue(climb.stowAllServosCommand());

    operator.b().and(climb::isInCalibrationMode).whileTrue(climb.calibrationRightBackReverse());
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
   * Check if the current odometry pose is close to a target pose (254-style). Used during disabled
   * to show a dashboard indicator confirming the robot is physically placed near the expected auto
   * starting position.
   *
   * @param pose The target pose to compare against
   * @return true if within 0.25m translation and 8° rotation
   */
  public boolean odometryCloseToPose(Pose2d pose) {
    Pose2d current = swerveIO.getPose();
    double distance = current.getTranslation().getDistance(pose.getTranslation());
    double rotation =
        Math.abs(current.getRotation().rotateBy(pose.getRotation().unaryMinus()).getDegrees());
    Logger.recordOutput("Auto/DistanceFromStartPose", distance);
    Logger.recordOutput("Auto/RotationFromStartPose", rotation);
    return distance < 0.25 && rotation < 8.0;
  }

  /** Get the drive subsystem (for Robot.java lifecycle access). */
  public DriveSwerveDrivetrain getDriveSubsystem() {
    return swerveIO;
  }

  /** Get the climb subsystem (for Robot.java sim-reset access). */
  public ClimbSubsystem getClimbSubsystem() {
    return climb;
  }

  /** Get the dashboard auto manager (for Robot.java pre-seeding). */
  public DashboardAutoManager getDashboardAutoManager() {
    return dashboardAutoManager;
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
   * Called during disabled periodic to keep the dashboard auto manager up to date. Reads settings
   * from the dashboard and replans if anything changed. Also handles PathPlanner warmup.
   */
  public void runAutoWarmup() {
    // Update dashboard auto manager — reads settings and replans if changed
    dashboardAutoManager.update();
  }
}
