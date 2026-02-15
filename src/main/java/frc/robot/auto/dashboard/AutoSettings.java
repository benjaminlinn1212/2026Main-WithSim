// Copyright (c) 2026 FRC Team 0 (Amped)
// Dashboard-driven autonomous system — Configurable settings for REBUILT

package frc.robot.auto.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.dashboard.FieldConstants.ClimbLevel;
import frc.robot.auto.dashboard.FieldConstants.ClimbPose;
import frc.robot.auto.dashboard.FieldConstants.IntakeLocation;
import frc.robot.auto.dashboard.FieldConstants.Lane;
import frc.robot.auto.dashboard.FieldConstants.ScoringWaypoint;
import frc.robot.auto.dashboard.FieldConstants.StartPose;
import java.util.ArrayList;
import java.util.Comparator;
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
  private final List<ScoringWaypoint> scoringPriority = new ArrayList<>();
  private final List<IntakeLocation> intakePriority = new ArrayList<>();
  private final Set<Lane> allowedLanes = EnumSet.allOf(Lane.class);
  private final Set<Lane> partnerLanes = EnumSet.noneOf(Lane.class);
  private boolean attemptClimb = false;
  private ClimbLevel climbLevel = ClimbLevel.LEVEL_1;
  private ClimbPose climbPose = ClimbPose.DEPOT_SIDE;
  private boolean shootWhileDriving = false;
  private RiskLevel riskLevel = RiskLevel.BALANCED;
  private boolean preloadFuel = true;
  private int preloadCount = 8; // FUEL preloaded (1-8 per game manual)
  private int maxCycles = 4; // max score+intake cycles to attempt

  // ===== SmartDashboard Keys (under "Auto/" subtable for Elastic) =====
  private static final String PREFIX = "Auto/";

  // ===== SendableChoosers for dropdown settings in Elastic =====
  private final SendableChooser<StartPose> startPoseChooser = new SendableChooser<>();
  private final SendableChooser<ClimbLevel> climbLevelChooser = new SendableChooser<>();
  private final SendableChooser<ClimbPose> climbPoseChooser = new SendableChooser<>();
  private final SendableChooser<RiskLevel> riskLevelChooser = new SendableChooser<>();

  // Fingerprint for change detection
  private String lastFingerprint = "";

  public AutoSettings() {
    initDashboard();
  }

  // ===== Dashboard Initialization =====

  private void initDashboard() {
    // --- Dropdown: Start Pose ---
    for (StartPose sp : StartPose.values()) {
      if (sp == StartPose.CENTER) {
        startPoseChooser.setDefaultOption(sp.name(), sp);
      } else {
        startPoseChooser.addOption(sp.name(), sp);
      }
    }
    SmartDashboard.putData(PREFIX + "Start Pose", startPoseChooser);

    // --- Intake Priority ---
    // Each intake location gets a number: 1 = first priority, 2 = second, etc.
    // Set to 0 to disable that location entirely.
    // Example: DEPOT=1, OUTPOST=2 means go to DEPOT first, then OUTPOST.
    SmartDashboard.putNumber(PREFIX + "Intake/OUTPOST", 1);
    SmartDashboard.putNumber(PREFIX + "Intake/DEPOT", 2);
    SmartDashboard.putNumber(PREFIX + "Intake/NEUTRAL_ZONE_UPPER", 3);
    SmartDashboard.putNumber(PREFIX + "Intake/NEUTRAL_ZONE_LOWER", 4);

    // --- Dropdown: Climb Level ---
    for (ClimbLevel cl : ClimbLevel.values()) {
      if (cl == ClimbLevel.LEVEL_1) {
        climbLevelChooser.setDefaultOption(cl.name(), cl);
      } else {
        climbLevelChooser.addOption(cl.name(), cl);
      }
    }
    SmartDashboard.putData(PREFIX + "Climb Level", climbLevelChooser);

    // --- Dropdown: Climb Pose (which side of TOWER to approach) ---
    for (ClimbPose cp : ClimbPose.values()) {
      if (cp == ClimbPose.DEPOT_SIDE) {
        climbPoseChooser.setDefaultOption(cp.name(), cp);
      } else {
        climbPoseChooser.addOption(cp.name(), cp);
      }
    }
    SmartDashboard.putData(PREFIX + "Climb Pose", climbPoseChooser);

    // --- Dropdown: Risk Level ---
    for (RiskLevel rl : RiskLevel.values()) {
      if (rl == RiskLevel.BALANCED) {
        riskLevelChooser.setDefaultOption(rl.name(), rl);
      } else {
        riskLevelChooser.addOption(rl.name(), rl);
      }
    }
    SmartDashboard.putData(PREFIX + "Risk Level", riskLevelChooser);

    // --- Multi-value boolean toggles ---
    // Shooting Priority: one toggle per scoring waypoint (enabled = included in priority list)
    for (ScoringWaypoint sl : ScoringWaypoint.values()) {
      // Default: enable all 3 positions
      SmartDashboard.putBoolean(PREFIX + "Score/" + sl.name(), true);
    }

    // Allowed Lanes: one toggle per lane (default: all enabled)
    for (Lane lane : Lane.values()) {
      SmartDashboard.putBoolean(PREFIX + "Lane/" + lane.name(), true);
    }

    // Partner Lanes: one toggle per lane (default: all disabled)
    for (Lane lane : Lane.values()) {
      SmartDashboard.putBoolean(PREFIX + "Partner Lane/" + lane.name(), false);
    }

    // --- Boolean toggles ---
    SmartDashboard.putBoolean(PREFIX + "Attempt TOWER Climb", attemptClimb);
    SmartDashboard.putBoolean(PREFIX + "Shoot While Driving", shootWhileDriving);
    SmartDashboard.putBoolean(PREFIX + "Has Preload FUEL", preloadFuel);

    // --- Number fields ---
    SmartDashboard.putNumber(PREFIX + "Preload FUEL Count", preloadCount);
    SmartDashboard.putNumber(PREFIX + "Max Cycles", maxCycles);
  }

  // ===== Read from Dashboard =====

  /**
   * Poll all dashboard entries and update local fields. Returns true if any setting changed since
   * the last call.
   */
  public boolean readFromDashboard() {
    // Start Pose (dropdown)
    StartPose selectedStartPose = startPoseChooser.getSelected();
    if (selectedStartPose != null) {
      startPose = selectedStartPose;
    }

    // Scoring Priority (boolean toggles per location)
    scoringPriority.clear();
    for (ScoringWaypoint sl : ScoringWaypoint.values()) {
      if (SmartDashboard.getBoolean(PREFIX + "Score/" + sl.name(), false)) {
        scoringPriority.add(sl);
      }
    }

    // Intake Priority (numbered per location — lower number = higher priority, 0 = disabled)
    intakePriority.clear();
    for (IntakeLocation il : IntakeLocation.values()) {
      int priority = (int) SmartDashboard.getNumber(PREFIX + "Intake/" + il.name(), 0);
      if (priority > 0) {
        intakePriority.add(il);
      }
    }
    // Sort by the dashboard-assigned priority number (ascending: 1 first, 2 second, etc.)
    intakePriority.sort(
        Comparator.comparingInt(
            il -> (int) SmartDashboard.getNumber(PREFIX + "Intake/" + il.name(), 99)));

    // Allowed Lanes (boolean toggles per lane)
    allowedLanes.clear();
    for (Lane lane : Lane.values()) {
      if (SmartDashboard.getBoolean(PREFIX + "Lane/" + lane.name(), true)) {
        allowedLanes.add(lane);
      }
    }
    if (allowedLanes.isEmpty()) {
      allowedLanes.addAll(EnumSet.allOf(Lane.class)); // fallback: all lanes
    }

    // Partner Lanes (boolean toggles per lane)
    partnerLanes.clear();
    for (Lane lane : Lane.values()) {
      if (SmartDashboard.getBoolean(PREFIX + "Partner Lane/" + lane.name(), false)) {
        partnerLanes.add(lane);
      }
    }

    // Booleans
    attemptClimb = SmartDashboard.getBoolean(PREFIX + "Attempt TOWER Climb", attemptClimb);
    shootWhileDriving =
        SmartDashboard.getBoolean(PREFIX + "Shoot While Driving", shootWhileDriving);
    preloadFuel = SmartDashboard.getBoolean(PREFIX + "Has Preload FUEL", preloadFuel);

    // Preload Count (1-8)
    preloadCount =
        Math.max(
            1,
            Math.min(
                FieldConstants.MAX_PRELOAD_FUEL,
                (int) SmartDashboard.getNumber(PREFIX + "Preload FUEL Count", preloadCount)));

    // Climb Level (dropdown)
    ClimbLevel selectedClimb = climbLevelChooser.getSelected();
    if (selectedClimb != null) {
      climbLevel = selectedClimb;
    }

    // Climb Pose (dropdown)
    ClimbPose selectedClimbPose = climbPoseChooser.getSelected();
    if (selectedClimbPose != null) {
      climbPose = selectedClimbPose;
    }

    // Risk Level (dropdown)
    RiskLevel selectedRisk = riskLevelChooser.getSelected();
    if (selectedRisk != null) {
      riskLevel = selectedRisk;
    }

    // Max Cycles
    maxCycles = (int) SmartDashboard.getNumber(PREFIX + "Max Cycles", maxCycles);

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
  public List<ScoringWaypoint> getScoringPriority() {
    return List.copyOf(scoringPriority);
  }

  /** Ordered list of intake locations, highest priority first. */
  public List<IntakeLocation> getIntakePriority() {
    return List.copyOf(intakePriority);
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

  public ClimbPose getClimbPose() {
    return climbPose;
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
        + intakePriority
        + "|"
        + allowedLanes
        + "|"
        + partnerLanes
        + "|"
        + attemptClimb
        + "|"
        + climbLevel.name()
        + "|"
        + climbPose.name()
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
    Logger.recordOutput("AutoSettings/IntakePriority", intakePriority.toString());
    Logger.recordOutput("AutoSettings/AllowedLanes", allowedLanes.toString());
    Logger.recordOutput("AutoSettings/PartnerLanes", partnerLanes.toString());
    Logger.recordOutput("AutoSettings/EffectiveLanes", getEffectiveLanes().toString());
    Logger.recordOutput("AutoSettings/AttemptClimb", attemptClimb);
    Logger.recordOutput("AutoSettings/ClimbLevel", climbLevel.name());
    Logger.recordOutput("AutoSettings/ClimbPose", climbPose.name());
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
        + intakePriority
        + ", lanes="
        + allowedLanes
        + ", partner="
        + partnerLanes
        + ", climb="
        + attemptClimb
        + ", climbLevel="
        + climbLevel
        + ", climbPose="
        + climbPose
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
