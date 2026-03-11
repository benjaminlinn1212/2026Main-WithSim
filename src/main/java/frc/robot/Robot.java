// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.dashboard.FieldConstants;
import frc.robot.subsystems.shooter.ShooterIOSim;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Main robot class. Extends LoggedRobot for AdvantageKit logging. Handles mode transitions,
 * 254-style pose pre-seeding during disabled, and sim initialization.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  /**
   * Tracks whether the robot has been enabled at least once. Used for fallback heading reset in
   * teleopInit() if auto was skipped.
   */
  private boolean hasBeenEnabled = false;

  /** Disabled periodic iteration counter for throttled pose pre-seeding. */
  private int disabledPeriodicCount = 0;

  private static final int DISABLED_PERIODIC_CHECK_INTERVAL = 50; // every ~1 second (50 * 20ms)

  /**
   * Last pre-seeded starting pose. Used to detect auto selection changes independently of
   * dashboardAutoManager.didSettingsChange().
   */
  private Pose2d lastPreSeededPose = null;

  /** FPGA timestamp at auto start, for computing auto countdown timer. */
  private double autoStartTimestamp = 0.0;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Optionally log to a .wpilog file on disk
        if (Constants.ENABLE_FILE_LOGGING) {
          if (new java.io.File("/U").exists()) {
            Logger.addDataReceiver(new WPILOGWriter());
          } else {
            System.out.println(
                "[Robot] WARNING: No USB drive found at /U. Logging to /home/lvuser/logs instead.");
            Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
          }
        }
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer
    robotContainer = new RobotContainer();

    // Warm up PathPlanner to avoid first-call JIT delay
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Global auto countdown timer — works for every auto routine
    if (DriverStation.isAutonomous() && autoStartTimestamp > 0.0) {
      double elapsed = Timer.getFPGATimestamp() - autoStartTimestamp;
      double remaining = Math.max(0.0, FieldConstants.AUTO_DURATION - elapsed);
      Logger.recordOutput("Auto/TimeRemaining", remaining);
    }
  }

  @Override
  public void disabledInit() {
    // Cancel any running commands when disabled
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
      autonomousCommand = null;
    }

    // Throttle Limelight frame processing while disabled to reduce thermal load
    LimelightHelpers.SetThrottle(Constants.Vision.FRONT_LIMELIGHT_NAME, 150);
    LimelightHelpers.SetThrottle(Constants.Vision.TURRET_LIMELIGHT_NAME, 150);

    // Reset tracking so the first disabledPeriodic check immediately pre-seeds
    disabledPeriodicCount = 0;
    lastPreSeededPose = null;
  }

  /**
   * Periodically check if auto settings changed and pre-seed odometry with the starting pose
   * (alliance-corrected).
   */
  @Override
  public void disabledPeriodic() {
    robotContainer.runAutoWarmup();

    // 254-style: Only run the heavy check every N iterations (~1s)
    disabledPeriodicCount++;
    if (disabledPeriodicCount % DISABLED_PERIODIC_CHECK_INTERVAL == 0) {
      // Centralized starting pose that accounts for all auto modes
      Pose2d startingPose = robotContainer.getAutoStartingPose();

      // Publish "Near Auto Starting Pose" indicator
      if (startingPose != null) {
        boolean nearStartPose = robotContainer.odometryCloseToPose(startingPose);
        SmartDashboard.putBoolean("Near Auto Starting Pose", nearStartPose);
        Logger.recordOutput("Auto/NearAutoStartingPose", nearStartPose);
      }

      // Detect changes from either source:
      //  1. Dashboard auto settings changed (start pose dropdown, intake locations, etc.)
      //  2. Top-level auto chooser or hardcoded auto chooser selection changed
      boolean dashboardSettingsChanged =
          robotContainer.getDashboardAutoManager().didSettingsChange();
      boolean autoSelectionChanged =
          startingPose != null && !startingPose.equals(lastPreSeededPose);

      if (dashboardSettingsChanged || autoSelectionChanged) {
        if (startingPose != null) {
          lastPreSeededPose = startingPose;
          robotContainer.getDriveSubsystem().setPose(startingPose);
          System.out.println(
              "[Robot] 254-style: Pre-seeded odometry to auto start pose: " + startingPose);
          Logger.recordOutput("Auto/PreSeededPose", startingPose);
        }
      }
    }
  }

  /**
   * No hard pose reset -- pre-seeded during disabled. SIM hard-resets since no physical placement.
   */
  @Override
  public void autonomousInit() {
    hasBeenEnabled = true;

    // Remove Limelight throttle — full framerate for match
    LimelightHelpers.SetThrottle(Constants.Vision.FRONT_LIMELIGHT_NAME, 0);
    LimelightHelpers.SetThrottle(Constants.Vision.TURRET_LIMELIGHT_NAME, 0);

    // Capture FPGA timestamp for global auto countdown timer
    autoStartTimestamp = Timer.getFPGATimestamp();

    // SIM-only: Hard-reset pose to the selected auto's starting pose
    if (Constants.currentMode == Constants.Mode.SIM) {
      Pose2d startingPose = robotContainer.getAutoStartingPose();
      if (startingPose != null) {
        robotContainer.getDriveSubsystem().setPose(startingPose);
        System.out.println("[Robot] SIM: Reset pose to auto start pose: " + startingPose);
        Logger.recordOutput("Auto/SimResetPose", startingPose);
      }

      // Reset climb to STOWED so Mechanism2d re-initializes properly
      robotContainer.getClimbSubsystem().resetToStowed();

      // Reset the simulated field: clear all game pieces and re-place them at starting positions
      if (Constants.DriveConstants.USE_MAPLE_SIM) {
        SimulatedArena.getInstance().resetFieldForAuto();
        System.out.println("[Robot] SIM: Reset MapleSimField game pieces for auto");
      }

      // If scoring preload, give the simulated robot 8 FUEL so it has ammo to shoot
      ShooterIOSim.resetFuelCount();
      if (robotContainer.getDashboardAutoManager().getSettings().isScorePreload()) {
        for (int i = 0; i < 8; i++) {
          ShooterIOSim.notifyFuelReady();
        }
        System.out.println("[Robot] SIM: Preloaded 8 FUEL for score-preload auto");
      }
    }

    // Always get a fresh command from the chooser
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  /** If auto was skipped, reset heading to alliance wall orientation for field-relative driving. */
  @Override
  public void teleopInit() {
    // Cancel any running autonomous command
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    // Remove Limelight throttle — full framerate for match
    LimelightHelpers.SetThrottle(Constants.Vision.FRONT_LIMELIGHT_NAME, 0);
    LimelightHelpers.SetThrottle(Constants.Vision.TURRET_LIMELIGHT_NAME, 0);

    // 254-style: Fallback heading reset on first enable
    if (!hasBeenEnabled) {
      hasBeenEnabled = true;
      Pose2d currentPose = robotContainer.getDriveSubsystem().getPose();
      boolean isRed = FieldConstants.isRedAlliance();
      Pose2d resetPose =
          new Pose2d(
              currentPose.getTranslation(),
              isRed ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0));
      robotContainer.getDriveSubsystem().setPose(resetPose);
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    robotContainer.setSwerveStartingPose(
        new edu.wpi.first.math.geometry.Pose2d(
            0.5, 0.5, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)));

    // Place game pieces on the field for simulation
    if (Constants.DriveConstants.USE_MAPLE_SIM) {
      ((org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt)
              SimulatedArena.getInstance())
          .setEfficiencyMode(false);
      SimulatedArena.getInstance().placeGamePiecesOnField();
    }
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    if (Constants.DriveConstants.USE_MAPLE_SIM) {
      Logger.recordOutput(
          "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }
  }
}
