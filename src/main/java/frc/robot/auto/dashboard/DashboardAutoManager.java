// Copyright (c) 2026 FRC Team 0 (Amped)
// Dashboard-driven autonomous system — Lifecycle manager

package frc.robot.auto.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * The top-level manager for the dashboard-driven autonomous system. This class:
 *
 * <ul>
 *   <li>Owns the {@link AutoSettings} (dashboard configuration)
 *   <li>Runs the {@link AutoPlanner} whenever settings change
 *   <li>Builds a fresh {@link Command} via {@link AutoCommandBuilder}
 *   <li>Publishes a plan preview to the dashboard / AdvantageKit logs
 *   <li>Provides the autonomous command to {@code Robot.java}
 * </ul>
 *
 * <p>Call {@link #update()} every disabled periodic cycle. When autonomous starts, call {@link
 * #getAutoCommand()} to get the latest generated command.
 *
 * <p>Inspired by 254's AutoModeSelector and 6328's auto management approach.
 */
public class DashboardAutoManager {

  private final AutoSettings settings;
  private final AutoCommandBuilder commandBuilder;
  private final DriveSwerveDrivetrain drive;

  // Cached plan and command
  private List<AutoAction> currentPlan = new ArrayList<>();
  private Command cachedCommand = null;

  // Plan visualization
  private Pose2d[] planPosePreview = new Pose2d[0];

  /**
   * Create a new DashboardAutoManager.
   *
   * @param drive The swerve drive subsystem
   * @param superstructure The superstructure
   */
  public DashboardAutoManager(DriveSwerveDrivetrain drive, Superstructure superstructure) {
    this.drive = drive;
    this.settings = new AutoSettings();
    this.commandBuilder = new AutoCommandBuilder(drive, superstructure);

    System.out.println(
        "[DashboardAutoManager] Initialized — configure settings in 'Auto Settings' tab");
  }

  /**
   * Call this every disabled periodic cycle. Reads dashboard settings, replans if anything changed,
   * and publishes the preview.
   */
  public void update() {
    boolean changed = settings.readFromDashboard();

    if (changed) {
      System.out.println("[DashboardAutoManager] Settings changed — replanning...");
      replan();
    }

    // Always log the current plan summary (useful for dashboard)
    logPlanPreview();
  }

  /**
   * Force a replan regardless of whether settings changed. Useful at autonomousInit() to ensure the
   * latest plan is used.
   */
  public void forceReplan() {
    settings.readFromDashboard();
    replan();
  }

  /** Run the planner and cache the result. */
  private void replan() {
    long startTime = System.nanoTime();

    currentPlan = AutoPlanner.plan(settings);
    cachedCommand = null; // Invalidate cached command

    // Build pose preview for dashboard visualization
    buildPosePreview();

    long elapsed = System.nanoTime() - startTime;
    Logger.recordOutput("DashboardAuto/PlanTimeUs", elapsed / 1000.0);
    System.out.println(
        "[DashboardAutoManager] Plan generated: "
            + currentPlan.size()
            + " actions in "
            + (elapsed / 1000)
            + "μs");
  }

  /**
   * Get the autonomous command. This should be called in autonomousInit(). Builds a fresh command
   * from the current plan.
   *
   * @return The auto command, or a no-op if no plan exists
   */
  public Command getAutoCommand() {
    // Always force a replan at auto start to ensure we have the freshest settings
    forceReplan();

    if (currentPlan.isEmpty()) {
      System.out.println("[DashboardAutoManager] WARNING: Empty plan — returning no-op");
      return Commands.print("[DashboardAuto] No plan generated — check Auto Settings tab!")
          .withName("DashboardAuto_NoPlan");
    }

    // Build a fresh command (uses Commands.defer internally for PathPlanner commands)
    // Pass settings so the command builder can check climb config at runtime
    final AutoSettings settingsSnapshot = settings;
    cachedCommand =
        Commands.defer(
                () -> commandBuilder.buildAutoCommand(currentPlan, settingsSnapshot), Set.of(drive))
            .withName("DashboardAuto");

    System.out.println(
        "[DashboardAutoManager] Auto command ready: " + currentPlan.size() + " actions");
    Logger.recordOutput("DashboardAuto/CommandReady", true);

    return cachedCommand;
  }

  // ===== Dashboard Preview =====

  /** Build an array of poses for AdvantageScope path visualization. */
  private void buildPosePreview() {
    List<Pose2d> poses = new ArrayList<>();
    for (AutoAction action : currentPlan) {
      Pose2d target = action.getTargetPose();
      if (target != null) {
        poses.add(target);
      }
    }
    planPosePreview = poses.toArray(new Pose2d[0]);
  }

  /** Publish the plan preview to AdvantageKit / NetworkTables. */
  private void logPlanPreview() {
    // Plan summary string
    String summary = AutoPlanner.summarizePlan(currentPlan);
    Logger.recordOutput("DashboardAuto/PlanSummary", summary);

    // Action count
    Logger.recordOutput("DashboardAuto/ActionCount", currentPlan.size());

    // Estimated total duration
    if (!currentPlan.isEmpty()) {
      double totalDuration =
          AutoPlanner.estimateTotalDuration(currentPlan, settings.getStartPose().getPose());
      Logger.recordOutput("DashboardAuto/EstimatedDuration", totalDuration);
      Logger.recordOutput("DashboardAuto/TimeMargin", FieldConstants.AUTO_DURATION - totalDuration);
    }

    // Pose array for AdvantageScope visualization
    if (planPosePreview.length > 0) {
      Logger.recordOutput("DashboardAuto/PlanPoses", planPosePreview);
    }

    // Individual action descriptions
    String[] descriptions = new String[currentPlan.size()];
    for (int i = 0; i < currentPlan.size(); i++) {
      descriptions[i] = (i + 1) + ". " + currentPlan.get(i).describe();
    }
    Logger.recordOutput("DashboardAuto/ActionList", descriptions);

    // Settings echo
    Logger.recordOutput("DashboardAuto/Settings", settings.toString());
  }

  // ===== Accessors =====

  /** Get the current plan (read-only). */
  public List<AutoAction> getCurrentPlan() {
    return currentPlan;
  }

  /** Get the settings object (for direct access if needed). */
  public AutoSettings getSettings() {
    return settings;
  }
}
