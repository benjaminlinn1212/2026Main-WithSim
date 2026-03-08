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
import frc.robot.auto.OutpostAuto;
import frc.robot.auto.SweepAuto;
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
import frc.robot.util.TrenchAssistController;
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
  private final Superstructure superstructure;

  @SuppressWarnings("unused")
  private final VisionSubsystem vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Climb level chooser (L1 quick climb vs full L2L3 teleop sequence)
  private final SendableChooser<ClimbSubsystem.ClimbLevel> climbLevelChooser =
      new SendableChooser<>();

  // Dashboard-driven autonomous system (254/6328 style)
  private DashboardAutoManager dashboardAutoManager;

  // Hardcoded auto chooser (select specific hardcoded auto routines)
  private final LoggedDashboardChooser<String> hardcodedAutoChooser =
      new LoggedDashboardChooser<>("Auto/Hardcoded Auto Choices");

  // Hardcoded auto instances (built once, getCommand() defers internally)
  private OutpostAuto outpostAuto;

  // Orchestra manager for playing music through Kraken motors
  private OrchestraManager orchestraManager;

  // Teleop trench assist controller (orientation + lateral centering PID)
  private TrenchAssistController trenchAssist;

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
        break;
    }

    // Instantiate Superstructure with all subsystems
    superstructure =
        new Superstructure(shooter, turret, hood, intake, intakePivot, conveyor, indexer, climb);

    // Trench assist controller — always snaps to nearest cardinal heading
    trenchAssist = new TrenchAssistController();

    // Climb level chooser (L1 quick climb vs L2L3 full sequence)
    climbLevelChooser.setDefaultOption("L2L3", ClimbSubsystem.ClimbLevel.L2L3);
    climbLevelChooser.addOption("L1", ClimbSubsystem.ClimbLevel.L1);
    SmartDashboard.putData("Climb/Level", climbLevelChooser);
    climb.setClimbLevelSupplier(
        () -> {
          var selected = climbLevelChooser.getSelected();
          return selected != null ? selected : ClimbSubsystem.ClimbLevel.L2L3;
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

    // Hardcoded auto chooser — individual named hardcoded autos
    outpostAuto = new OutpostAuto(drive, superstructure);
    hardcodedAutoChooser.addDefaultOption("Outpost", "Outpost");
    autoChooser.addOption(
        "Hardcoded Auto", Commands.defer(() -> getHardcodedAutoCommand(), Set.of(drive)));

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
      // Uses the same approach buffer as the teleop trench assist so the zone is consistent.
      @SuppressWarnings("resource")
      PIDController trenchRotationPID =
          new PIDController(Constants.AutoConstants.PATH_FOLLOWING_ROTATION_KP, 0, 0);
      trenchRotationPID.enableContinuousInput(-Math.PI, Math.PI);

      new edu.wpi.first.wpilibj2.command.button.Trigger(
              () ->
                  DriverStation.isTeleopEnabled()
                      && FieldConstants.isNearTrench(
                          drive.getPose().getTranslation(),
                          Constants.DriveConstants.TrenchAssist.APPROACH_BUFFER))
          .onTrue(
              Commands.runOnce(
                  () -> {
                    trenchRotationPID.reset();
                    PPHolonomicDriveController.overrideRotationFeedback(
                        () -> {
                          Pose2d pose = drive.getPose();
                          Rotation2d snapped = FieldConstants.snapToCardinal(pose.getRotation());
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
              TrenchAssistController.Result trenchResult =
                  trenchAssist.calculate(
                      drive.getPose(), vxMetersPerSec, vyMetersPerSec, omegaRadPerSec);
              vxMetersPerSec = trenchResult.vx();
              vyMetersPerSec = trenchResult.vy();
              omegaRadPerSec = trenchResult.omega();

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
    // Mode buttons set the base state. The feeding flag (RT) promotes aiming→shooting
    // automatically in Superstructure.periodic() — no RT checks needed here.

    // B button: IDLE (also clears feeding flag)
    controller.b().onTrue(superstructure.idle());

    // A button: ONLY_INTAKE
    controller.a().onTrue(superstructure.onlyIntake());

    // Y button: ONLY_AIMING
    controller.y().onTrue(superstructure.onlyAiming());

    // X button: AIMING_WHILE_INTAKING
    controller.x().onTrue(superstructure.aimingWhileIntaking());

    // Right trigger: Set feeding flag while held. Superstructure.periodic() handles
    // promoting aiming → shooting and demoting when released.
    controller
        .rightTrigger(0.2)
        .onTrue(Commands.runOnce(() -> superstructure.setFeedingRequested(true)))
        .onFalse(Commands.runOnce(() -> superstructure.setFeedingRequested(false)));

    // ===== CLIMB STATE CONTROLS (driver POV) =====
    // POV Left: Previous climb step (handles release, stow, and all reverse transitions)
    controller.povLeft().onTrue(climb.previousClimbStep());

    // POV Right: Next climb step
    controller.povRight().onTrue(climb.nextClimbStep());

    // Right bumper: TEST — pathfind to climb pose only (no climb)
    controller
        .rightBumper()
        .onTrue(
            Commands.defer(
                () ->
                    AutoBuilder.pathfindToPose(
                        dashboardAutoManager.getSettings().getClimbPose().getPose(),
                        drive.getPathConstraints(),
                        0.0),
                Set.of(drive)));
  }

  /**
   * Configure operator controller bindings (port 1). Controls climb calibration and manual modes.
   * Climb state stepping is on the driver POV. Climb is independent from Superstructure.
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

    // ===== POV + A/Y: Calibration mode → climb motors =====
    // POV Up: left front forward
    operator.povUp().and(climb::isInCalibrationMode).whileTrue(climb.calibrationLeftFrontForward());

    // POV Down: left front reverse
    operator
        .povDown()
        .and(climb::isInCalibrationMode)
        .whileTrue(climb.calibrationLeftFrontReverse());

    // POV Left: left back reverse
    operator
        .povLeft()
        .and(climb::isInCalibrationMode)
        .whileTrue(climb.calibrationLeftBackReverse());

    // POV Right: left back forward
    operator
        .povRight()
        .and(climb::isInCalibrationMode)
        .whileTrue(climb.calibrationLeftBackForward());

    // Y: right front forward
    operator.y().and(climb::isInCalibrationMode).whileTrue(climb.calibrationRightFrontForward());

    // A: right front reverse
    operator.a().and(climb::isInCalibrationMode).whileTrue(climb.calibrationRightFrontReverse());

    // X/B: Right back motor (cal mode only)
    operator.x().and(climb::isInCalibrationMode).whileTrue(climb.calibrationRightBackForward());

    operator.b().and(climb::isInCalibrationMode).whileTrue(climb.calibrationRightBackReverse());

    // LT/RT: Hardstop servo stow/release (available in all modes)
    operator.leftTrigger(0.3).onTrue(climb.stowHardstopServosCommand());
    operator.rightTrigger(0.3).onTrue(climb.releaseHardstopServosCommand());

    // Left/Right joystick buttons: Angle servo stow/release (available in all modes)
    operator.leftStick().onTrue(climb.stowAngleServosCommand());
    operator.rightStick().onTrue(climb.releaseAngleServosCommand());

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
   * Returns the starting pose for the currently selected auto mode. Used by Robot.java for
   * pre-seeding odometry during disabled and SIM hard-reset.
   *
   * <ul>
   *   <li>"Hardcoded Auto" → delegates to the hardcoded auto's {@code START_POSE}
   *   <li>"Dashboard Auto" → {@code dashboardAutoManager.getStartingPose()}
   *   <li>Otherwise → {@code null} (no pre-seeding for Do Nothing, Sweep, Music, etc.)
   * </ul>
   */
  public Pose2d getAutoStartingPose() {
    String selected = autoChooser.getSendableChooser().getSelected();
    if ("Hardcoded Auto".equals(selected)) {
      return getHardcodedAutoStartingPose();
    } else if ("Dashboard Auto".equals(selected)) {
      return dashboardAutoManager.getStartingPose();
    }
    return null;
  }

  /**
   * Returns the starting pose for the currently selected hardcoded auto. Reads from the hardcoded
   * auto chooser and maps each selection to its constant START_POSE. As new hardcoded autos are
   * added, extend this switch.
   */
  private Pose2d getHardcodedAutoStartingPose() {
    String selection = hardcodedAutoChooser.get();
    // Currently only one hardcoded auto — extend with additional cases as needed
    if ("Outpost".equals(selection)) {
      return OutpostAuto.START_POSE.getPose();
    }
    return OutpostAuto.START_POSE.getPose(); // Default fallback
  }

  /**
   * Resolve the hardcoded auto chooser selection to a fresh command. Called inside a {@code
   * Commands.defer()} so a new command graph is built each auto run, avoiding the "already
   * composed" error.
   */
  private Command getHardcodedAutoCommand() {
    String selection = hardcodedAutoChooser.get();
    // Currently only one hardcoded auto — extend with additional cases as needed
    if ("Outpost".equals(selection)) {
      return outpostAuto.buildCommand();
    }
    return outpostAuto.buildCommand(); // Default fallback
  }

  /**
   * Called during disabled periodic to update dashboard auto manager and handle PathPlanner warmup.
   */
  public void runAutoWarmup() {
    dashboardAutoManager.update();
  }
}
