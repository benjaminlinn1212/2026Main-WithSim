// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.dashboard.FieldConstants;
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
   * 254-style: Track whether the robot has been enabled at least once this power cycle. Used for
   * fallback heading reset in teleopInit() — if the robot goes directly to teleop without ever
   * running auto, we reset heading to alliance wall orientation.
   */
  private boolean hasBeenEnabled = false;

  /**
   * 254-style disabled periodic iteration counter. We only run the pose pre-seeding logic every N
   * iterations to avoid overwhelming the CAN bus and dashboard with updates.
   */
  private int disabledPeriodicCount = 0;

  private static final int DISABLED_PERIODIC_CHECK_INTERVAL = 50; // every ~1 second (50 * 20ms)

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
        // Running on a real robot, log to a USB stick ("/U/logs") if available
        if (new java.io.File("/U").exists()) {
          Logger.addDataReceiver(new WPILOGWriter());
        } else {
          System.out.println(
              "[Robot] WARNING: No USB drive found at /U. Logging to /home/lvuser/logs instead.");
          Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
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
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    // Cancel any running commands when disabled
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
      autonomousCommand = null;
    }

    // Reset the periodic counter so the first disabledPeriodic immediately pre-seeds
    disabledPeriodicCount = 0;
  }

  /**
   * 254-style pose pre-seeding: periodically check if auto settings changed, and when they do,
   * pre-seed odometry with the selected auto's starting pose (alliance-corrected).
   */
  @Override
  public void disabledPeriodic() {
    robotContainer.runAutoWarmup();

    // 254-style: Only run the heavy check every N iterations (~1s)
    disabledPeriodicCount++;
    if (disabledPeriodicCount % DISABLED_PERIODIC_CHECK_INTERVAL == 0) {
      var dashboardAutoManager = robotContainer.getDashboardAutoManager();

      // Publish "Near Auto Starting Pose" indicator
      Pose2d startingPose = dashboardAutoManager.getStartingPose();
      if (startingPose != null) {
        boolean nearStartPose = robotContainer.odometryCloseToPose(startingPose);
        SmartDashboard.putBoolean("Near Auto Starting Pose", nearStartPose);
        Logger.recordOutput("Auto/NearAutoStartingPose", nearStartPose);
      }

      // Pre-seed odometry when auto settings change
      if (dashboardAutoManager.didSettingsChange()) {
        if (startingPose != null) {
          robotContainer.getDriveSubsystem().setPose(startingPose);
          System.out.println(
              "[Robot] 254-style: Pre-seeded odometry to auto start pose: " + startingPose);
          Logger.recordOutput("Auto/PreSeededPose", startingPose);
        }
      }
    }
  }

  /**
   * 254-style: No hard pose reset here. The pose was pre-seeded during disabled and refined by
   * vision. In SIM, hard-reset since there is no physical placement or vision.
   */
  @Override
  public void autonomousInit() {
    hasBeenEnabled = true;

    // SIM-only: Hard-reset pose to the selected auto's starting pose
    if (Constants.currentMode == Constants.Mode.SIM) {
      Pose2d startingPose = robotContainer.getDashboardAutoManager().getStartingPose();
      if (startingPose != null) {
        robotContainer.getDriveSubsystem().setPose(startingPose);
        System.out.println("[Robot] SIM: Reset pose to auto start pose: " + startingPose);
        Logger.recordOutput("Auto/SimResetPose", startingPose);
      }

      // Reset climb to STOWED so Mechanism2d re-initializes properly
      robotContainer.getClimbSubsystem().resetToStowed();
    }

    // Always get a fresh command from the chooser
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  /**
   * 254-style: If the robot has never been enabled (skipped auto), reset heading to alliance wall
   * orientation for correct field-relative driving.
   */
  @Override
  public void teleopInit() {
    // Cancel any running autonomous command
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

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
