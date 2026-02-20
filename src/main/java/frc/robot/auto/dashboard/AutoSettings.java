// Copyright (c) 2026 FRC Team 0 (Amped)
// Dashboard-driven autonomous system — Configurable settings for REBUILT

package frc.robot.auto.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.dashboard.FieldConstants.ClimbLevel;
import frc.robot.auto.dashboard.FieldConstants.ClimbPose;
import frc.robot.auto.dashboard.FieldConstants.IntakeLocation;
import frc.robot.auto.dashboard.FieldConstants.ScoringWaypoint;
import frc.robot.auto.dashboard.FieldConstants.StartPose;
import java.util.ArrayList;
import java.util.List;
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
 *   <li>Climb level — which TOWER RUNG to attempt (LEVEL 1/2/3, different point values)
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
  private boolean attemptClimb = false;
  private ClimbLevel climbLevel = ClimbLevel.LEVEL_1;
  private ClimbPose climbPose = ClimbPose.DEPOT_SIDE;
  private boolean shootWhileDriving = false;
  private RiskLevel riskLevel = RiskLevel.BALANCED;
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

    // --- Intake Sequence (string input) ---
    // Type a sequence of characters to define intake visit order:
    //   U = NEUTRAL_ZONE_UPPER, L = NEUTRAL_ZONE_LOWER, D = DEPOT, O = OUTPOST
    // Example: "ULDO" → Upper Neutral first, then Lower Neutral, Depot, Outpost
    // Duplicate or unknown characters are ignored.
    SmartDashboard.putString(PREFIX + "Intake Sequence", "ULDO");

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

    // --- Boolean toggles ---
    SmartDashboard.putBoolean(PREFIX + "Attempt TOWER Climb", attemptClimb);
    SmartDashboard.putBoolean(PREFIX + "Shoot While Driving", shootWhileDriving);

    // --- Number fields ---
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

    // Intake Sequence (string input — each character maps to an IntakeLocation)
    // Duplicates are allowed: "UUDO" means cycle 1→U, cycle 2→U, cycle 3→D, cycle 4→O
    intakePriority.clear();
    String intakeSequence =
        SmartDashboard.getString(PREFIX + "Intake Sequence", "ULDO").toUpperCase().trim();
    for (char c : intakeSequence.toCharArray()) {
      IntakeLocation loc = charToIntakeLocation(c);
      if (loc != null) {
        intakePriority.add(loc);
      }
    }

    // Booleans
    attemptClimb = SmartDashboard.getBoolean(PREFIX + "Attempt TOWER Climb", attemptClimb);
    shootWhileDriving =
        SmartDashboard.getBoolean(PREFIX + "Shoot While Driving", shootWhileDriving);

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

  /** Ordered list of intake locations in sequence order (1st visited first). */
  public List<IntakeLocation> getIntakePriority() {
    return List.copyOf(intakePriority);
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
        + maxCycles;
  }

  // ===== Logging =====

  private void logSettings() {
    Logger.recordOutput("AutoSettings/StartPose", startPose.name());
    Logger.recordOutput("AutoSettings/ShootingPriority", scoringPriority.toString());
    Logger.recordOutput(
        "AutoSettings/IntakeSequence",
        SmartDashboard.getString(PREFIX + "Intake Sequence", "ULDO"));
    Logger.recordOutput("AutoSettings/IntakePriority", intakePriority.toString());
    Logger.recordOutput("AutoSettings/AttemptClimb", attemptClimb);
    Logger.recordOutput("AutoSettings/ClimbLevel", climbLevel.name());
    Logger.recordOutput("AutoSettings/ClimbPose", climbPose.name());
    Logger.recordOutput("AutoSettings/ShootWhileDriving", shootWhileDriving);
    Logger.recordOutput("AutoSettings/RiskLevel", riskLevel.name());
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
        + ", maxCycles="
        + maxCycles
        + "}";
  }

  // ===== Intake Sequence Parsing =====

  /**
   * Map a single character to an {@link IntakeLocation}.
   *
   * <ul>
   *   <li>{@code U} → NEUTRAL_ZONE_UPPER
   *   <li>{@code L} → NEUTRAL_ZONE_LOWER
   *   <li>{@code D} → DEPOT
   *   <li>{@code O} → OUTPOST
   * </ul>
   *
   * @param c The character (case-insensitive, already uppercased by caller)
   * @return The matching IntakeLocation, or null if unknown
   */
  private static IntakeLocation charToIntakeLocation(char c) {
    return switch (c) {
      case 'U' -> IntakeLocation.NEUTRAL_ZONE_UPPER;
      case 'L' -> IntakeLocation.NEUTRAL_ZONE_LOWER;
      case 'D' -> IntakeLocation.DEPOT;
      case 'O' -> IntakeLocation.OUTPOST;
      default -> null;
    };
  }
}
