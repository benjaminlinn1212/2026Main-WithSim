// Copyright (c) 2026 FRC Team 0 (Amped)
// Dashboard-driven autonomous system — Configurable settings for REBUILT

package frc.robot.auto.dashboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.auto.dashboard.FieldConstants.ClimbLevel;
import frc.robot.auto.dashboard.FieldConstants.IntakeLocation;
import frc.robot.auto.dashboard.FieldConstants.Lane;
import frc.robot.auto.dashboard.FieldConstants.ScoringLocation;
import frc.robot.auto.dashboard.FieldConstants.StartPose;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * All dashboard-configurable autonomous settings for the 2026 REBUILT game. Before each match, the
 * drive team toggles these on Shuffleboard / AdvantageScope / SmartDashboard. The {@link
 * DashboardAutoManager} reads them and feeds them to the {@link AutoPlanner}.
 *
 * <p>REBUILT-specific settings:
 *
 * <ul>
 *   <li>Start pose — which DRIVER STATION side to start at
 *   <li>Shooting priority — preferred positions around the HUB to shoot FUEL
 *   <li>Preferred intake — OUTPOST (human player), DEPOT (floor bin), or NEUTRAL ZONE (ground)
 *   <li>Lane management — TRENCH/BUMP side vs center, partner coordination
 *   <li>Climb level — which TOWER RUNG to attempt (LEVEL 1/2/3, different point values)
 *   <li>Preload count — how many FUEL to preload (1-8 per game manual 6.3.4.C)
 *   <li>Shoot while driving — continuous scoring vs stop-and-shoot
 *   <li>Risk level — conservative/balanced/aggressive time budgets
 * </ul>
 *
 * <p>Inspired by 254's pre-match configuration and 6328's dashboard-driven approach.
 */
public class AutoSettings {

  // ===== Risk Level =====
  public enum RiskLevel {
    /** Conservative: stick to safe lanes, extra time margins, avoid NEUTRAL ZONE. */
    CONSERVATIVE,
    /** Balanced: normal pathing, reasonable margins. */
    BALANCED,
    /** Aggressive: tight time budgets, willing to cross into contested NEUTRAL ZONE. */
    AGGRESSIVE
  }

  // ===== Backing fields (read from dashboard each cycle) =====
  private StartPose startPose = StartPose.CENTER;
  private final List<ScoringLocation> scoringPriority = new ArrayList<>();
  private IntakeLocation preferredIntake = IntakeLocation.OUTPOST;
  private final Set<Lane> allowedLanes = EnumSet.allOf(Lane.class);
  private final Set<Lane> partnerLanes = EnumSet.noneOf(Lane.class);
  private boolean attemptClimb = false;
  private ClimbLevel climbLevel = ClimbLevel.LEVEL_1;
  private boolean shootWhileDriving = false;
  private RiskLevel riskLevel = RiskLevel.BALANCED;
  private boolean preloadFuel = true;
  private int preloadCount = 8; // FUEL preloaded (1-8 per game manual)
  private int maxCycles = 4; // max score+intake cycles to attempt

  // ===== Shuffleboard Widgets =====
  private final ShuffleboardTab tab;
  private GenericEntry startPoseEntry;
  private GenericEntry scoringPriorityEntry;
  private GenericEntry preferredIntakeEntry;
  private GenericEntry allowedLanesEntry;
  private GenericEntry partnerLanesEntry;
  private GenericEntry attemptClimbEntry;
  private GenericEntry climbLevelEntry;
  private GenericEntry shootWhileDrivingEntry;
  private GenericEntry riskLevelEntry;
  private GenericEntry preloadEntry;
  private GenericEntry preloadCountEntry;
  private GenericEntry maxCyclesEntry;

  // Fingerprint for change detection
  private String lastFingerprint = "";

  public AutoSettings() {
    tab = Shuffleboard.getTab("Auto Settings");
    initDashboard();
  }

  // ===== Dashboard Initialization =====

  private void initDashboard() {
    // Start Pose — which DRIVER STATION side
    startPoseEntry =
        tab.add("Start Pose", startPose.name()).withPosition(0, 0).withSize(2, 1).getEntry();

    // Shooting Priority — comma-separated HUB positions, e.g. "HUB_BACK_CENTER,HUB_UPPER_CLOSE"
    scoringPriorityEntry =
        tab.add("Shooting Priority", "HUB_BACK_CENTER,HUB_UPPER_CLOSE,HUB_LOWER_CLOSE")
            .withPosition(0, 1)
            .withSize(3, 1)
            .getEntry();

    // Preferred Intake — OUTPOST (human player), DEPOT (floor), or NEUTRAL_ZONE_*
    preferredIntakeEntry =
        tab.add("Preferred Intake", preferredIntake.name())
            .withPosition(0, 2)
            .withSize(2, 1)
            .getEntry();

    // Allowed Lanes — TRENCH/BUMP side selection
    allowedLanesEntry =
        tab.add("Allowed Lanes", "UPPER,CENTER,LOWER").withPosition(3, 0).withSize(2, 1).getEntry();

    // Partner Lanes — lanes our partner will use (we avoid these)
    partnerLanesEntry = tab.add("Partner Lanes", "").withPosition(3, 1).withSize(2, 1).getEntry();

    // Attempt Climb (TOWER)
    attemptClimbEntry =
        tab.add("Attempt TOWER Climb", attemptClimb)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(5, 0)
            .withSize(1, 1)
            .getEntry();

    // Climb Level — LEVEL_1 (10pts), LEVEL_2 (20pts), LEVEL_3 (30pts)
    climbLevelEntry =
        tab.add("Climb Level", climbLevel.name()).withPosition(5, 1).withSize(2, 1).getEntry();

    // Shoot While Driving
    shootWhileDrivingEntry =
        tab.add("Shoot While Driving", shootWhileDriving)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(5, 2)
            .withSize(1, 1)
            .getEntry();

    // Risk Level
    riskLevelEntry =
        tab.add("Risk Level", riskLevel.name()).withPosition(7, 0).withSize(2, 1).getEntry();

    // Has Preloaded FUEL
    preloadEntry =
        tab.add("Has Preload FUEL", preloadFuel)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(7, 1)
            .withSize(1, 1)
            .getEntry();

    // Preload Count (1-8 FUEL per game manual 6.3.4.C)
    preloadCountEntry =
        tab.add("Preload FUEL Count", preloadCount).withPosition(7, 2).withSize(1, 1).getEntry();

    // Max Cycles
    maxCyclesEntry = tab.add("Max Cycles", maxCycles).withPosition(8, 0).withSize(1, 1).getEntry();
  }

  // ===== Read from Dashboard =====

  /**
   * Poll all dashboard entries and update local fields. Returns true if any setting changed since
   * the last call.
   */
  public boolean readFromDashboard() {
    // Start Pose
    try {
      startPose =
          StartPose.valueOf(startPoseEntry.getString(startPose.name()).trim().toUpperCase());
    } catch (IllegalArgumentException e) {
      // keep previous
    }

    // Scoring Priority
    scoringPriority.clear();
    String priorityStr =
        scoringPriorityEntry
            .getString("HUB_BACK_CENTER,HUB_UPPER_CLOSE,HUB_LOWER_CLOSE")
            .trim()
            .toUpperCase();
    for (String token : priorityStr.split("[,\\s]+")) {
      if (token.isEmpty()) continue;
      try {
        scoringPriority.add(ScoringLocation.valueOf(token));
      } catch (IllegalArgumentException e) {
        // skip invalid
      }
    }

    // Preferred Intake
    try {
      preferredIntake =
          IntakeLocation.valueOf(
              preferredIntakeEntry.getString(preferredIntake.name()).trim().toUpperCase());
    } catch (IllegalArgumentException e) {
      // keep previous
    }

    // Allowed Lanes
    allowedLanes.clear();
    String lanesStr = allowedLanesEntry.getString("UPPER,CENTER,LOWER").trim().toUpperCase();
    for (String token : lanesStr.split("[,\\s]+")) {
      if (token.isEmpty()) continue;
      try {
        allowedLanes.add(Lane.valueOf(token));
      } catch (IllegalArgumentException e) {
        // skip
      }
    }
    if (allowedLanes.isEmpty()) {
      allowedLanes.addAll(EnumSet.allOf(Lane.class)); // fallback: all lanes
    }

    // Partner Lanes
    partnerLanes.clear();
    String partnerStr = partnerLanesEntry.getString("").trim().toUpperCase();
    for (String token : partnerStr.split("[,\\s]+")) {
      if (token.isEmpty()) continue;
      try {
        partnerLanes.add(Lane.valueOf(token));
      } catch (IllegalArgumentException e) {
        // skip
      }
    }

    // Booleans
    attemptClimb = attemptClimbEntry.getBoolean(attemptClimb);
    shootWhileDriving = shootWhileDrivingEntry.getBoolean(shootWhileDriving);
    preloadFuel = preloadEntry.getBoolean(preloadFuel);

    // Preload Count (1-8)
    preloadCount =
        Math.max(
            1,
            Math.min(
                FieldConstants.MAX_PRELOAD_FUEL, (int) preloadCountEntry.getInteger(preloadCount)));

    // Climb Level
    try {
      climbLevel =
          ClimbLevel.valueOf(climbLevelEntry.getString(climbLevel.name()).trim().toUpperCase());
    } catch (IllegalArgumentException e) {
      // keep previous
    }

    // Risk Level
    try {
      riskLevel =
          RiskLevel.valueOf(riskLevelEntry.getString(riskLevel.name()).trim().toUpperCase());
    } catch (IllegalArgumentException e) {
      // keep previous
    }

    // Max Cycles
    maxCycles = (int) maxCyclesEntry.getInteger(maxCycles);

    // Change detection
    String fingerprint = computeFingerprint();
    boolean changed = !fingerprint.equals(lastFingerprint);
    lastFingerprint = fingerprint;

    // Log to AdvantageKit
    logSettings();

    return changed;
  }

  // ===== Getters =====

  public StartPose getStartPose() {
    return startPose;
  }

  /** Ordered list of scoring locations, highest priority first. */
  public List<ScoringLocation> getScoringPriority() {
    return List.copyOf(scoringPriority);
  }

  public IntakeLocation getPreferredIntake() {
    return preferredIntake;
  }

  /** Lanes this robot is allowed to use. */
  public Set<Lane> getAllowedLanes() {
    return EnumSet.copyOf(allowedLanes);
  }

  /** Lanes our alliance partner will use — we should avoid these. */
  public Set<Lane> getPartnerLanes() {
    return partnerLanes.isEmpty() ? EnumSet.noneOf(Lane.class) : EnumSet.copyOf(partnerLanes);
  }

  /**
   * Effective lanes = allowed lanes minus partner lanes. These are the lanes the planner should
   * prefer.
   */
  public Set<Lane> getEffectiveLanes() {
    Set<Lane> effective = EnumSet.copyOf(allowedLanes);
    effective.removeAll(partnerLanes);
    if (effective.isEmpty()) {
      // If removing partner lanes leaves nothing, fall back to allowed lanes
      return EnumSet.copyOf(allowedLanes);
    }
    return effective;
  }

  public boolean shouldAttemptClimb() {
    return attemptClimb;
  }

  public ClimbLevel getClimbLevel() {
    return climbLevel;
  }

  public boolean isShootWhileDriving() {
    return shootWhileDriving;
  }

  public RiskLevel getRiskLevel() {
    return riskLevel;
  }

  public boolean hasPreload() {
    return preloadFuel;
  }

  /** Number of FUEL preloaded into the robot (1-8). */
  public int getPreloadCount() {
    return preloadCount;
  }

  public int getMaxCycles() {
    return maxCycles;
  }

  /**
   * Get the time margin multiplier based on risk level. Conservative adds a big buffer; aggressive
   * is tight.
   */
  public double getTimeMarginMultiplier() {
    return switch (riskLevel) {
      case CONSERVATIVE -> 1.4;
      case BALANCED -> 1.15;
      case AGGRESSIVE -> 1.0;
    };
  }

  // ===== Change Detection =====

  private String computeFingerprint() {
    return startPose.name()
        + "|"
        + scoringPriority
        + "|"
        + preferredIntake.name()
        + "|"
        + allowedLanes
        + "|"
        + partnerLanes
        + "|"
        + attemptClimb
        + "|"
        + climbLevel.name()
        + "|"
        + shootWhileDriving
        + "|"
        + riskLevel.name()
        + "|"
        + preloadFuel
        + "|"
        + preloadCount
        + "|"
        + maxCycles;
  }

  // ===== Logging =====

  private void logSettings() {
    Logger.recordOutput("AutoSettings/StartPose", startPose.name());
    Logger.recordOutput("AutoSettings/ShootingPriority", scoringPriority.toString());
    Logger.recordOutput("AutoSettings/PreferredIntake", preferredIntake.name());
    Logger.recordOutput("AutoSettings/AllowedLanes", allowedLanes.toString());
    Logger.recordOutput("AutoSettings/PartnerLanes", partnerLanes.toString());
    Logger.recordOutput("AutoSettings/EffectiveLanes", getEffectiveLanes().toString());
    Logger.recordOutput("AutoSettings/AttemptClimb", attemptClimb);
    Logger.recordOutput("AutoSettings/ClimbLevel", climbLevel.name());
    Logger.recordOutput("AutoSettings/ShootWhileDriving", shootWhileDriving);
    Logger.recordOutput("AutoSettings/RiskLevel", riskLevel.name());
    Logger.recordOutput("AutoSettings/HasPreloadFuel", preloadFuel);
    Logger.recordOutput("AutoSettings/PreloadCount", preloadCount);
    Logger.recordOutput("AutoSettings/MaxCycles", maxCycles);
  }

  @Override
  public String toString() {
    return "AutoSettings{"
        + "start="
        + startPose
        + ", shooting="
        + scoringPriority
        + ", intake="
        + preferredIntake
        + ", lanes="
        + allowedLanes
        + ", partner="
        + partnerLanes
        + ", climb="
        + attemptClimb
        + ", climbLevel="
        + climbLevel
        + ", shootMoving="
        + shootWhileDriving
        + ", risk="
        + riskLevel
        + ", preloadFuel="
        + preloadFuel
        + ", preloadCount="
        + preloadCount
        + ", maxCycles="
        + maxCycles
        + "}";
  }
}
