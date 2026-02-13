// Copyright (c) 2026 FRC Team 0 (Amped)
// Dashboard-driven autonomous system — REBUILT field geometry definitions

package frc.robot.auto.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Field constants for the 2026 REBUILT game field. Defines HUB shooting positions, FUEL intake
 * locations, traversal waypoints, zones, and lanes. All coordinates are in BLUE alliance origin
 * (0,0 = bottom-left of blue driver station wall). Red alliance poses are mirrored at runtime.
 *
 * <p>REBUILT field elements:
 *
 * <ul>
 *   <li>HUB — rectangular scoring target per alliance, centered 158.6in from ALLIANCE WALL
 *   <li>TOWER — climbing structure with 3 RUNGS (LOW 27in, MID 45in, HIGH 63in)
 *   <li>BUMP — 6.5in tall ramps on either side of HUB that robots drive over
 *   <li>TRENCH — 22.25in tall tunnels along field edges robots drive under
 *   <li>DEPOT — floor-level FUEL storage along ALLIANCE WALL (24 FUEL)
 *   <li>OUTPOST — human player station with CHUTE (25 FUEL) and CORRAL
 *   <li>NEUTRAL ZONE — center of field with ~360-408 scattered FUEL
 * </ul>
 *
 * <p>Inspired by 254's FieldLayout and 6328's field constants approach.
 */
public final class FieldConstants {

  private FieldConstants() {}

  // ===== Field Dimensions =====
  public static final double FIELD_LENGTH = 16.54; // meters (~651.2in)
  public static final double FIELD_WIDTH = 8.07; // meters (~317.7in)
  public static final Translation2d FIELD_CENTER =
      new Translation2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0);

  // ===== HUB 3D Positions =====
  // Used by ShooterSetpoint for distance-based aiming
  public static final Translation3d BLUE_HUB_TRANSLATION3D =
      new Translation3d(4.625689, 4.040981, 0);
  public static final Translation3d RED_HUB_POSE_TRANSLATION3D =
      new Translation3d(FIELD_LENGTH - 4.625689, 4.040981, 0);

  // ===== Zones =====
  // Zones define regions of the REBUILT field. The planner uses these for constraint-checking.
  public enum Zone {
    /** ALLIANCE ZONE — behind the ROBOT STARTING LINE, contains our TOWER and DEPOT. */
    ALLIANCE_ZONE,
    /** OUTPOST AREA — human player station with CHUTE and CORRAL. */
    OUTPOST_AREA,
    /** Area around our HUB where we shoot FUEL. Between ALLIANCE ZONE and NEUTRAL ZONE. */
    HUB_ZONE,
    /** NEUTRAL ZONE — center of field with scattered FUEL. Contested territory. */
    NEUTRAL_ZONE,
    /** Opponent's half — risky, penalties possible. */
    OPPONENT_SIDE
  }

  // ===== Lanes =====
  // Lanes are high-level traversal corridors defined by field geometry.
  // TRENCH side (low clearance, 22.25in) vs. BUMP side (ramp over) vs. CENTER (open).
  // Drivers/strategists pick which lanes are "ours" vs. partner's to avoid collisions.
  public enum Lane {
    /** Upper lane — TRENCH/BUMP side closer to scoring table wall (high Y). */
    UPPER,
    /** Center lane — through the middle of the field between HUBs. */
    CENTER,
    /** Lower lane — TRENCH/BUMP side away from scoring table (low Y). */
    LOWER
  }

  // ===== Scoring Waypoints =====
  public enum ScoringWaypoint {
    /** Upper-close — shooting from the upper side of the HUB, near the BUMP. */
    HUB_UPPER_CLOSE(new Translation2d(3.50, 5.80), Zone.HUB_ZONE, Lane.UPPER),
    /** Upper-far — shooting from farther back on the upper side. */
    HUB_UPPER_FAR(new Translation2d(2.80, 6.20), Zone.HUB_ZONE, Lane.UPPER),
    /** Front-center — shooting straight at the HUB from the NEUTRAL ZONE side. */
    HUB_FRONT_CENTER(new Translation2d(5.00, 4.03), Zone.NEUTRAL_ZONE, Lane.CENTER),
    /** Back-center — shooting from behind the HUB near the ALLIANCE ZONE. */
    HUB_BACK_CENTER(new Translation2d(2.50, 4.03), Zone.HUB_ZONE, Lane.CENTER),
    /** Lower-close — shooting from the lower side of the HUB, near the BUMP. */
    HUB_LOWER_CLOSE(new Translation2d(3.50, 2.27), Zone.HUB_ZONE, Lane.LOWER),
    /** Lower-far — shooting from farther back on the lower side. */
    HUB_LOWER_FAR(new Translation2d(2.80, 1.87), Zone.HUB_ZONE, Lane.LOWER);

    public final Translation2d bluePosition;
    public final Zone zone;
    public final Lane lane;

    ScoringWaypoint(Translation2d bluePosition, Zone zone, Lane lane) {
      this.bluePosition = bluePosition;
      this.zone = zone;
      this.lane = lane;
    }

    /** Get the alliance-corrected position. */
    public Translation2d getPosition() {
      return isRedAlliance() ? flipTranslation(bluePosition) : bluePosition;
    }

    /** Get as a Pose2d (heading = 0) for PathPlanner pathfinding. */
    public Pose2d toPose() {
      return new Pose2d(getPosition(), new Rotation2d());
    }
  }

  // ===== Intake Locations =====
  // Where the robot can collect FUEL. REBUILT has three FUEL sources:
  // 1. OUTPOST CHUTE — human player feeds FUEL (up to 25 stored)
  // 2. DEPOT — floor-level bin along ALLIANCE WALL (24 FUEL)
  // 3. NEUTRAL ZONE — scattered FUEL across the center of the field (~360-408)
  public enum IntakeLocation {
    /** OUTPOST — human player CHUTE delivers FUEL. Located at one end of the field. */
    OUTPOST(new Pose2d(0.495, 0.656, Rotation2d.fromDegrees(135)), Zone.OUTPOST_AREA, Lane.UPPER),
    /** DEPOT — floor-level FUEL bin along ALLIANCE WALL. */
    DEPOT(new Pose2d(0.665, 5.962, Rotation2d.fromDegrees(180)), Zone.ALLIANCE_ZONE, Lane.LOWER),
    
    /** NEUTRAL ZONE upper — pick up FUEL from the upper side of the neutral zone. */
    NEUTRAL_ZONE_UPPER(
        new Pose2d(7.084, 5.905, Rotation2d.fromDegrees(-90)),
        Zone.NEUTRAL_ZONE,
        Lane.UPPER),
    /** NEUTRAL ZONE lower — pick up FUEL from the lower side of the neutral zone. */
    NEUTRAL_ZONE_LOWER(
        new Pose2d(7.084, 2.165, Rotation2d.fromDegrees(90)),
        Zone.NEUTRAL_ZONE,
        Lane.LOWER);

    public final Pose2d bluePose;
    public final Zone zone;
    public final Lane lane;

    IntakeLocation(Pose2d bluePose, Zone zone, Lane lane) {
      this.bluePose = bluePose;
      this.zone = zone;
      this.lane = lane;
    }

    /** Get the alliance-corrected pose. */
    public Pose2d getPose() {
      return isRedAlliance() ? flipPose(bluePose) : bluePose;
    }
  }

  // ===== Traversal Waypoints =====
  public enum Waypoint {
    // Upper lane waypoints (TRENCH/BUMP side, high Y)
    UPPER_HUB_EXIT(new Pose2d(4.50, 6.50, Rotation2d.fromDegrees(0)), Lane.UPPER),
    UPPER_NEUTRAL(new Pose2d(FIELD_LENGTH / 2.0, 6.50, Rotation2d.fromDegrees(0)), Lane.UPPER),
    UPPER_OUTPOST_APPROACH(new Pose2d(2.00, 7.00, Rotation2d.fromDegrees(135)), Lane.UPPER),

    // Lower lane waypoints (TRENCH/BUMP side, low Y)
    LOWER_HUB_EXIT(new Pose2d(4.50, 1.57, Rotation2d.fromDegrees(0)), Lane.LOWER),
    LOWER_NEUTRAL(new Pose2d(FIELD_LENGTH / 2.0, 1.57, Rotation2d.fromDegrees(0)), Lane.LOWER),
    LOWER_DEPOT_APPROACH(new Pose2d(1.50, 2.00, Rotation2d.fromDegrees(180)), Lane.LOWER);

    public final Pose2d bluePose;
    public final Lane lane;

    Waypoint(Pose2d bluePose, Lane lane) {
      this.bluePose = bluePose;
      this.lane = lane;
    }

    /** Get the alliance-corrected pose. */
    public Pose2d getPose() {
      return isRedAlliance() ? flipPose(bluePose) : bluePose;
    }
  }

  // ===== Start Poses =====
  // Pre-defined starting positions behind the ROBOT STARTING LINE.
  // The ROBOT STARTING LINE is the ALLIANCE colored line at the edge of the ALLIANCE's
  // base in front of two BARRIERs and the ALLIANCE HUB.
  public enum StartPose {
    /** Starting pose near DRIVER STATION 1 (upper side, near OUTPOST). */
    UPPER(new Pose2d(0.75, 6.70, Rotation2d.fromDegrees(0))),
    /** Starting pose near DRIVER STATION 2 (center, near TOWER). */
    CENTER(new Pose2d(0.75, 4.03, Rotation2d.fromDegrees(0))),
    /** Starting pose near DRIVER STATION 3 (lower side, near DEPOT). */
    LOWER(new Pose2d(0.75, 1.30, Rotation2d.fromDegrees(0)));

    public final Pose2d bluePose;

    StartPose(Pose2d bluePose) {
      this.bluePose = bluePose;
    }

    /** Get the alliance-corrected pose. */
    public Pose2d getPose() {
      return isRedAlliance() ? flipPose(bluePose) : bluePose;
    }
  }

    // ===== Climb Poses =====
  public enum ClimbPose {
    /** Starting pose near DRIVER STATION 1 (upper side, near OUTPOST). */
    DEPOT_SIDE(new Pose2d(1.554, 3.993, Rotation2d.fromDegrees(180))),
    /** Starting pose near DRIVER STATION 2 (center, near TOWER). */
    OUTPOST_SIDE(new Pose2d(1.554, 3.548, Rotation2d.fromDegrees(180)));

    public final Pose2d bluePose;

    ClimbPose(Pose2d bluePose) {
      this.bluePose = bluePose;
    }

    /** Get the alliance-corrected pose. */
    public Pose2d getPose() {
      return isRedAlliance() ? flipPose(bluePose) : bluePose;
    }
  }

  // ===== Climb Level =====
  // The TOWER has 3 RUNGS at different heights. Higher RUNGS score more points.
  // LEVEL 1: robot off carpet/TOWER BASE (15 pts auto, 10 pts teleop)
  // LEVEL 2: BUMPERS above LOW RUNG (20 pts teleop)
  // LEVEL 3: BUMPERS above MID RUNG (30 pts teleop)
  // The TOWER is integrated into the ALLIANCE WALL between DS2 and DS3.
  public enum ClimbLevel {
    /** LEVEL 1 — off the carpet/TOWER BASE. LOW RUNG at 27in. (10 pts teleop, 15 pts auto) */
    LEVEL_1(10, 15, 3.0),
    /** LEVEL 2 — BUMPERS above LOW RUNG. MID RUNG at 45in. (20 pts teleop) */
    LEVEL_2(20, 0, 5.0),
    /** LEVEL 3 — BUMPERS above MID RUNG. HIGH RUNG at 63in. (30 pts teleop) */
    LEVEL_3(30, 0, 8.0);

    public final int teleopPoints;
    public final int autoPoints;
    public final double estimatedClimbDuration; // seconds

    ClimbLevel(int teleopPoints, int autoPoints, double estimatedClimbDuration) {
      this.teleopPoints = teleopPoints;
      this.autoPoints = autoPoints;
      this.estimatedClimbDuration = estimatedClimbDuration;
    }
  }

  // ===== FUEL Preload =====
  /** Maximum FUEL a robot can preload (per game manual 6.3.4.C). */
  public static final int MAX_PRELOAD_FUEL = 8;

  // ===== Estimated action durations (seconds) =====
  // Used by the planner for time budgeting.
  /** Time to dump/shoot a load of FUEL into the HUB. */
  public static final double SCORE_DURATION = 3.0;

  /** Time to intake FUEL at a location (ground sweep or OUTPOST/DEPOT pickup). */
  public static final double INTAKE_DURATION = 2.0;

  /** AUTO period duration — 20 seconds per REBUILT game manual. */
  public static final double AUTO_DURATION = 20.0;

  // Average drive speed for time estimates (meters/second)
  public static final double AVG_DRIVE_SPEED = 3.0;

  // ===== Ranking Point Thresholds =====
  /** ENERGIZED RP — score at least this many FUEL in HUB (Regional/District). */
  public static final int ENERGIZED_RP_THRESHOLD = 100;

  /** SUPERCHARGED RP — score at least this many FUEL in HUB (Regional/District). */
  public static final int SUPERCHARGED_RP_THRESHOLD = 360;

  /** TRAVERSAL RP — score at least this many TOWER points (Regional/District). */
  public static final int TRAVERSAL_RP_THRESHOLD = 50;

  // ===== Alliance Utilities =====

  /** Check if we are on the red alliance. */
  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }

  /**
   * Flip a pose from blue origin to red origin. Mirrors X across the field center, flips heading.
   */
  public static Pose2d flipPose(Pose2d bluePose) {
    return new Pose2d(
        FIELD_LENGTH - bluePose.getX(),
        bluePose.getY(),
        Rotation2d.fromDegrees(180).minus(bluePose.getRotation()));
  }

  /** Flip a translation from blue origin to red origin. Mirrors X across the field center. */
  public static Translation2d flipTranslation(Translation2d blueTranslation) {
    return new Translation2d(FIELD_LENGTH - blueTranslation.getX(), blueTranslation.getY());
  }

  /**
   * Estimate drive time between two poses (straight-line distance / avg speed). The planner uses
   * this for rough time budgeting — not for trajectory generation.
   */
  public static double estimateDriveTime(Pose2d from, Pose2d to) {
    double distance = from.getTranslation().getDistance(to.getTranslation());
    return distance / AVG_DRIVE_SPEED;
  }
}
