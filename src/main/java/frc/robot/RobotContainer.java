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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.HardcodedAutos;
import frc.robot.auto.SweepAuto;
import frc.robot.auto.dashboard.DashboardAutoManager;
import frc.robot.auto.dashboard.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.climb.ClimbState;
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
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.orchestra.OrchestraManager;
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
 * Subsystem construction (REAL/SIM/REPLAY switch), auto chooser, PathPlanner config, and button
 * bindings.
 */
public class RobotContainer {
  private final RobotState robotState = new RobotState();

  // Subsystems
  private final DriveSwerveDrivetrain drive;
  private final IntakeSubsystem intake;
  private final IntakePivotSubsystem intakePivot;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;
  private final ShooterSubsystem shooter;
  private final ConveyorSubsystem conveyor;
  private final IndexerSubsystem indexer;
  private final ClimbSubsystem climb;
  private final LEDSubsystem leds;
  private final Superstructure superstructure;

  @SuppressWarnings("unused")
  private final VisionSubsystem vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Operator climb level chooser (L1 auto sequence vs full L2L3 teleop sequence)
  private final SendableChooser<ClimbSubsystem.OperatorClimbLevel> operatorClimbLevelChooser =
      new SendableChooser<>();

  // Dashboard-driven autonomous system (254/6328 style)
  private DashboardAutoManager dashboardAutoManager;

  // Hardcoded fallback autos (one per lane)
  private HardcodedAutos hardcodedAutos;

  // Orchestra manager for playing music through Kraken motors
  private OrchestraManager orchestraManager;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Regulate module constants for simulation
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
        drive =
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
        drive =
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
        drive =
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

    // Set up auto routines
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
        leds = new LEDSubsystem();
        break;
      case SIM:
        {
          // Get the SwerveDriveSimulation for maple-sim integration
          org.ironmaple.simulation.drivesims.SwerveDriveSimulation driveSim = null;
          if (Constants.DriveConstants.USE_MAPLE_SIM && drive.getDriveIO() instanceof DriveIOSim) {
            DriveIOSim driveIOSim = (DriveIOSim) drive.getDriveIO();
            if (driveIOSim.getMapleSimDrive() != null) {
              driveSim = driveIOSim.getMapleSimDrive().mapleSimDrive;
            }
          }

          // Create intake sim with maple-sim integration (or fallback)
          IntakeIOSim intakeIOSimInstance = null;
          if (driveSim != null) {
            intakeIOSimInstance = new IntakeIOSim(driveSim);
            intake = new IntakeSubsystem(intakeIOSimInstance);
          } else {
            intake = new IntakeSubsystem(new IntakeIO() {});
          }

          intakePivot = new IntakePivotSubsystem(new IntakePivotIOSim());
          turret = new TurretSubsystem(new TurretIOSim(), robotState);
          hood = new HoodSubsystem(new HoodIOSim());

          // Create shooter sim with maple-sim integration (or fallback)
          if (driveSim != null) {
            shooter =
                new ShooterSubsystem(
                    new ShooterIOSim(
                        driveSim, turret::getCurrentPosition, hood::getCurrentPosition));
          } else {
            shooter = new ShooterSubsystem(new ShooterIO() {});
          }

          // Create conveyor sim and wire it to intake sim for fuel transfer
          ConveyorIOSim conveyorIOSimInstance = new ConveyorIOSim();
          if (intakeIOSimInstance != null) {
            conveyorIOSimInstance.setIntakeIOSim(intakeIOSimInstance);
          }
          conveyor = new ConveyorSubsystem(conveyorIOSimInstance);

          indexer = new IndexerSubsystem(new IndexerIOSim());
          climb = new ClimbSubsystem(new ClimbIOSim());
          leds = new LEDSubsystem();
        }
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
        leds = new LEDSubsystem();
        break;
    }

    // Instantiate Superstructure with all subsystems
    superstructure =
        new Superstructure(
            shooter, turret, hood, intake, intakePivot, conveyor, indexer, climb, leds);

    // Operator climb level chooser
    operatorClimbLevelChooser.setDefaultOption("L2L3", ClimbSubsystem.OperatorClimbLevel.L2L3);
    operatorClimbLevelChooser.addOption("L1", ClimbSubsystem.OperatorClimbLevel.L1);
    SmartDashboard.putData("Operator/Climb Level", operatorClimbLevelChooser);
    climb.setOperatorClimbLevelSupplier(
        () -> {
          var selected = operatorClimbLevelChooser.getSelected();
          return selected != null ? selected : ClimbSubsystem.OperatorClimbLevel.L2L3;
        });

    // Wire IMU roll supplier for auto-level climb assist
    if (Constants.ClimbConstants.ImuAssist.ENABLED) {
      climb.setRollDegreesSupplier(
          () -> drive.getDriveIO().getPigeon2().getRoll().getValueAsDouble());
    }

    // Instantiate Vision with pose consumer feeding into the drive's pose estimator
    switch (Constants.currentMode) {
      case REAL:
        vision =
            new VisionSubsystem(
                new VisionIOHardwareLimelight(robotState),
                robotState,
                estimate ->
                    drive
                        .getDriveIO()
                        .addVisionMeasurement(
                            estimate.getVisionRobotPoseMeters(),
                            com.ctre.phoenix6.Utils.fpgaToCurrentTime(
                                estimate.getTimestampSeconds()),
                            estimate.getVisionMeasurementStdDevs()));
        break;
      case SIM:
        vision = new VisionSubsystem(inputs -> {}, robotState, estimate -> {});
        break;
      default:
        vision = new VisionSubsystem(inputs -> {}, robotState, estimate -> {});
        break;
    }

    // ===== CONFIGURE PATHPLANNER AUTOBUILDER =====
    configureAutoBuilder();

    // ===== CONFIGURE AUTO MODE SELECTOR =====
    dashboardAutoManager = new DashboardAutoManager(drive, superstructure, climb);
    autoChooser.addOption(
        "Dashboard Auto",
        Commands.defer(() -> dashboardAutoManager.getAutoCommand(), Set.of(drive)));

    // Hardcoded fallback autos
    hardcodedAutos = new HardcodedAutos(drive, superstructure, dashboardAutoManager);
    autoChooser.addOption("Hardcoded Auto", hardcodedAutos.getCommand());

    // Sweep auto
    SweepAuto sweepAuto = new SweepAuto(drive, superstructure, climb, dashboardAutoManager);
    autoChooser.addOption("Sweep Auto", sweepAuto.getCommand());

    // ===== ORCHESTRA (Play Music) AUTO =====
    if (Constants.currentMode == Constants.Mode.REAL) {
      orchestraManager = new OrchestraManager();
      autoChooser.addOption("Play Music", orchestraManager.playMusicCommand());
    }

    // ===== INTEGRATE SHOOTERSETPOINT UTILITY =====
    var shooterSetpointSupplier = ShooterSetpoint.createSupplier(robotState);

    // Connect all aiming subsystems to use the same ShooterSetpoint
    turret.setShooterSetpointSupplier(shooterSetpointSupplier);
    hood.setShooterSetpointSupplier(shooterSetpointSupplier);
    shooter.setShooterSetpointSupplier(shooterSetpointSupplier);

    turret.setRobotPoseSupplier(() -> drive.getPose());
    superstructure.setRobotPoseSupplier(() -> drive.getPose());

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Configure PathPlanner AutoBuilder for pathfinding and auto paths. */
  private void configureAutoBuilder() {
    try {
      Pathfinding.setPathfinder(new LocalADStar());

      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder with swerve drive
      AutoBuilder.configure(
          drive::getPose,
          drive::setPose,
          drive::getChassisSpeeds,
          (speeds, feedforwards) -> drive.runVelocity(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(Constants.AutoConstants.PATH_FOLLOWING_TRANSLATION_KP, 0.0, 0.0),
              new PIDConstants(Constants.AutoConstants.PATH_FOLLOWING_ROTATION_KP, 0.0, 0.0)),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          drive);

      // ===== Trench Heading Override =====
      // Override PathPlanner's rotation feedback inside trenches to snap heading to cardinal.
      @SuppressWarnings("resource")
      PIDController trenchRotationPID =
          new PIDController(Constants.AutoConstants.PATH_FOLLOWING_ROTATION_KP, 0, 0);
      trenchRotationPID.enableContinuousInput(-Math.PI, Math.PI);

      new edu.wpi.first.wpilibj2.command.button.Trigger(
              () -> FieldConstants.isNearTrench(drive.getPose().getTranslation()))
          .onTrue(
              Commands.runOnce(
                  () -> {
                    trenchRotationPID.reset();
                    PPHolonomicDriveController.overrideRotationFeedback(
                        () -> {
                          Pose2d pose = drive.getPose();
                          Rotation2d snapped =
                              superstructure.isIntakeDeployed()
                                  ? FieldConstants.snapToHorizontal(pose.getRotation())
                                  : FieldConstants.snapToCardinal(pose.getRotation());
                          return trenchRotationPID.calculate(
                              pose.getRotation().getRadians(), snapped.getRadians());
                        });
                  }))
          .onFalse(
              Commands.runOnce(() -> PPHolonomicDriveController.clearRotationFeedbackOverride()));
    } catch (Exception e) {
      System.err.println("[RobotContainer] CRITICAL: Failed to configure AutoBuilder!");
      System.err.println("  Pathfinding and auto commands will NOT work.");
      System.err.println("  Error: " + e.getMessage());
      e.printStackTrace();
    }
  }

  private void configureButtonBindings() {
    configureTeleopBindings();
    configureOperatorBindings();
  }

  /** Configure button bindings for teleop mode. */
  private void configureTeleopBindings() {
    System.out.println("[RobotContainer] Configuring teleop bindings...");

    // ===== DRIVE CONTROLS =====
    // Field-relative drive. On red alliance, flip vx/vy so "forward" = away from driver.
    drive.setDefaultCommand(
        Commands.run(
            () -> {
              // Apply deadband
              double leftY =
                  edu.wpi.first.math.MathUtil.applyDeadband(
                      -controller.getLeftY(), Constants.DriveConstants.JOYSTICK_DEADBAND);
              double leftX =
                  edu.wpi.first.math.MathUtil.applyDeadband(
                      -controller.getLeftX(), Constants.DriveConstants.JOYSTICK_DEADBAND);
              double rightX =
                  edu.wpi.first.math.MathUtil.applyDeadband(
                      -controller.getRightX(), Constants.DriveConstants.JOYSTICK_DEADBAND);

              // Flip field-relative directions for red alliance
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
              Pose2d currentPose = drive.getPose();
              Translation2d bluePos = currentPose.getTranslation();
              Rotation2d robotHeading = currentPose.getRotation();
              // Convert to blue-origin if on red alliance
              if (isRed) {
                bluePos = FieldConstants.flipTranslation(bluePos);
                robotHeading = robotHeading.plus(Rotation2d.fromDegrees(180));
              }

              // 1. Orientation alignment
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
                      trenchBuffer,
                      superstructure.isIntakeDeployed());
              omegaRadPerSec += orientationOmega;

              // 2. Lateral centering + wall avoidance
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

              drive.driveFieldRelative(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec);
            },
            drive));

    // Left bumper: Reset orientation (REAL) or full pose (SIM)
    controller
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (Constants.currentMode == Constants.Mode.SIM) {
                        drive.setPose(Constants.AutoConstants.DEFAULT_RESET_POSE);
                      } else {
                        var alliance = DriverStation.getAlliance();
                        boolean isRed =
                            alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                        Pose2d current = drive.getPose();
                        drive.setPose(
                            new Pose2d(
                                current.getTranslation(),
                                Rotation2d.fromDegrees(isRed ? 180.0 : 0.0)));
                      }
                    },
                    drive)
                .ignoringDisable(true));

    // ===== SUPERSTRUCTURE STATE CONTROLS =====

    // Right bumper: IDLE
    controller.rightBumper().onTrue(superstructure.idle());

    // A button: ONLY_INTAKE
    controller.a().onTrue(superstructure.onlyIntake());

    // Y button: ONLY_AIMING
    controller.y().onTrue(superstructure.onlyAiming());

    // X button: AIMING_WHILE_INTAKING
    controller.x().onTrue(superstructure.aimingWhileIntaking());

    // Right trigger: Feed to shooter while held (ONLY_AIMING→ONLY_SHOOTING, etc.)
    controller
        .rightTrigger(0.2)
        .onTrue(Commands.runOnce(superstructure::resetShooterDetection))
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
   * Configure operator controller bindings (port 1). Controls climb state machine, intake shake,
   * and climb calibration/manual modes. Climb is independent from Superstructure.
   */
  private void configureOperatorBindings() {
    System.out.println("[RobotContainer] Configuring operator bindings...");

    // ===== CLIMB CALIBRATION MODE =====
    operator
        .leftBumper()
        .onTrue(
            Commands.either(
                climb.exitCalibrationMode(),
                climb.enterCalibrationMode(),
                climb::isInCalibrationMode));

    // Right bumper: Toggle manual control mode
    operator
        .rightBumper()
        .onTrue(
            Commands.either(
                climb.exitManualMode(), climb.enterManualMode(), climb::isInManualMode));

    // ===== CALIBRATION MOTOR CONTROLS (only active in calibration mode) =====
    operator.povUp().and(climb::isInCalibrationMode).whileTrue(climb.calibrationLeftFrontForward());
    operator.povUp().and(() -> !climb.isInCalibrationMode()).onTrue(climb.releaseFromAutoL1());

    operator
        .povDown()
        .and(climb::isInCalibrationMode)
        .whileTrue(climb.calibrationLeftFrontReverse());
    // POV Down (normal mode): Stow climb
    operator
        .povDown()
        .and(() -> !climb.isInCalibrationMode())
        .onTrue(
            Commands.either(
                climb.stowPathOnly(),
                climb.stowFromCurrentState(),
                () ->
                    climb.getOperatorClimbLevel() == ClimbSubsystem.OperatorClimbLevel.L1
                        && (climb.getState() == ClimbState.EXTEND_L1_AUTO
                            || climb.getState() == ClimbState.RETRACT_L1_AUTO
                            || climb.getState() == ClimbState.STOWED)));

    operator
        .povLeft()
        .and(climb::isInCalibrationMode)
        .whileTrue(climb.calibrationLeftBackReverse());
    operator.povLeft().and(() -> !climb.isInCalibrationMode()).onTrue(climb.previousClimbStep());

    operator
        .povRight()
        .and(climb::isInCalibrationMode)
        .whileTrue(climb.calibrationLeftBackForward());
    operator.povRight().and(() -> !climb.isInCalibrationMode()).onTrue(climb.nextClimbStep());

    // Y/A: Right front motor (cal mode) / intake shake toggle (normal mode)
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

    // X/B: Right back motor (cal mode) / angle servo stow/release (normal mode)
    operator.x().and(climb::isInCalibrationMode).whileTrue(climb.calibrationRightBackForward());
    operator.x().and(() -> !climb.isInCalibrationMode()).onTrue(climb.stowAngleServosCommand());

    operator.b().and(climb::isInCalibrationMode).whileTrue(climb.calibrationRightBackReverse());
    operator.b().and(() -> !climb.isInCalibrationMode()).onTrue(climb.releaseAngleServosCommand());

    // LT/RT: Hardstop servo stow/release (normal) / all servos stow/release (cal mode)
    operator
        .leftTrigger(0.3)
        .and(() -> !climb.isInCalibrationMode())
        .onTrue(climb.stowHardstopServosCommand());
    operator.leftTrigger(0.3).and(climb::isInCalibrationMode).onTrue(climb.stowAllServosCommand());
    operator
        .rightTrigger(0.3)
        .and(() -> !climb.isInCalibrationMode())
        .onTrue(climb.releaseHardstopServosCommand());
    operator
        .rightTrigger(0.3)
        .and(climb::isInCalibrationMode)
        .onTrue(climb.releaseAllServosCommand());

    // ===== MANUAL CONTROL: mushroom heads control EE velocity in manual mode =====
    // Left stick → left climb arm, Right stick → right climb arm
    // Stick up → EE up (+Y), Stick right → EE forward (+X)
    new edu.wpi.first.wpilibj2.command.button.Trigger(climb::isInManualMode)
        .whileTrue(
            Commands.run(
                () -> {
                  double maxVel = frc.robot.Constants.ClimbConstants.PATH_MAX_VELOCITY_MPS;
                  double db = Constants.DriveConstants.JOYSTICK_DEADBAND;

                  double leftEeX =
                      edu.wpi.first.math.MathUtil.applyDeadband(operator.getLeftX(), db) * maxVel;
                  double leftEeY =
                      edu.wpi.first.math.MathUtil.applyDeadband(-operator.getLeftY(), db) * maxVel;

                  double rightEeX =
                      edu.wpi.first.math.MathUtil.applyDeadband(operator.getRightX(), db) * maxVel;
                  double rightEeY =
                      edu.wpi.first.math.MathUtil.applyDeadband(-operator.getRightY(), db) * maxVel;

                  climb.manualControl(
                      new Translation2d(leftEeX, leftEeY), new Translation2d(rightEeX, rightEeY));
                },
                climb));
  }

  /**
   * Sets the starting pose for the swerve drive (used in simulation).
   *
   * @param pose The starting pose
   */
  public void setSwerveStartingPose(edu.wpi.first.math.geometry.Pose2d pose) {
    drive.setPose(pose);
  }

  /** Check if the current odometry pose is close to a target pose (254-style). */
  public boolean odometryCloseToPose(Pose2d pose) {
    Pose2d current = drive.getPose();
    double distance = current.getTranslation().getDistance(pose.getTranslation());
    double rotation =
        Math.abs(current.getRotation().rotateBy(pose.getRotation().unaryMinus()).getDegrees());
    Logger.recordOutput("Auto/DistanceFromStartPose", distance);
    Logger.recordOutput("Auto/RotationFromStartPose", rotation);
    return distance < Constants.ODOMETRY_CLOSE_TRANSLATION_METERS
        && rotation < Constants.ODOMETRY_CLOSE_ROTATION_DEGREES;
  }

  /** Get the drive subsystem (for Robot.java lifecycle access). */
  public DriveSwerveDrivetrain getDriveSubsystem() {
    return drive;
  }

  /** Get the climb subsystem (for Robot.java sim-reset access). */
  public ClimbSubsystem getClimbSubsystem() {
    return climb;
  }

  /** Get the dashboard auto manager (for Robot.java pre-seeding). */
  public DashboardAutoManager getDashboardAutoManager() {
    return dashboardAutoManager;
  }

  /** Get the autonomous command selected on the dashboard. */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Called during disabled periodic to update dashboard auto manager and handle PathPlanner warmup.
   */
  public void runAutoWarmup() {
    dashboardAutoManager.update();
  }
}
