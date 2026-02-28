// Copyright (c) 2026 FRC Team 10922 (Amped)
// Dashboard-driven autonomous system — REBUILT field geometry definitions

package frc.robot.auto.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

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

  // ===== Trench Geometry =====
  // TRENCHES are 22.25in tall tunnels running along the upper and lower field edges.
  // Robots must drive straight (cardinal headings only: 0/90/180/270°) to pass through.
  // Each trench spans the full field length and is ~1.22m (48in) wide from the field wall inward.
  // The trench STRUCTURE (low ceiling) only exists in the X range [4.0, 5.2] (blue origin).
  // Outside that X range the robot is in open field and doesn't need heading constraints.
  // A buffer zone extends OUTSIDE the trench so the robot snaps heading BEFORE entering.
  public static final double TRENCH_HEIGHT_METERS = Units.inchesToMeters(22.25);
  public static final double TRENCH_WIDTH_METERS = Units.inchesToMeters(48.0);

  /** Trench X boundaries (blue origin). The low-ceiling tunnel only spans this X range. */
  public static final double TRENCH_MIN_X = 4.0; // meters

  public static final double TRENCH_MAX_X = 5.2; // meters

  /** Red trench X boundaries (blue origin, point-symmetric about field center). */
  public static final double TRENCH_RED_MIN_X = FIELD_LENGTH - TRENCH_MAX_X; // ~11.34m

  public static final double TRENCH_RED_MAX_X = FIELD_LENGTH - TRENCH_MIN_X; // ~12.54m

  /**
   * Buffer distance (meters) outside the trench boundaries. When the robot is within this distance
   * of a trench, its heading should already be snapped to a cardinal direction so it enters
   * straight.
   */
  public static final double TRENCH_APPROACH_BUFFER = 1.0; // meters

  /** Upper trench Y boundaries (high Y, near scoring table wall). */
  public static final double TRENCH_UPPER_MIN_Y = FIELD_WIDTH - TRENCH_WIDTH_METERS; // ~6.85m

  public static final double TRENCH_UPPER_MAX_Y = FIELD_WIDTH; // 8.07m

  /** Lower trench Y boundaries (low Y, away from scoring table). */
  public static final double TRENCH_LOWER_MIN_Y = 0.0;

  public static final double TRENCH_LOWER_MAX_Y = TRENCH_WIDTH_METERS; // ~1.22m

  /**
   * Check if a blue-origin position is near any trench on the field (inside OR within the approach
   * buffer). Both the blue-side trench (X 4.0–5.2) and the red-side trench (X 11.34–12.54,
   * point-symmetric) are checked, since both physical structures exist regardless of alliance.
   *
   * <p>With point symmetry the Y bands also flip: the blue upper trench corresponds to the red
   * lower trench and vice versa.
   */
  public static boolean isNearTrench(Translation2d bluePosition) {
    double x = bluePosition.getX();
    double y = bluePosition.getY();

    double buf = TRENCH_APPROACH_BUFFER;

    // --- Blue-side trench (X 4.0–5.2) ---
    boolean inBlueTrenchX = x >= (TRENCH_MIN_X - buf) && x <= (TRENCH_MAX_X + buf);
    if (inBlueTrenchX) {
      boolean nearUpper = y >= (TRENCH_UPPER_MIN_Y - buf) && y <= TRENCH_UPPER_MAX_Y;
      boolean nearLower = y >= TRENCH_LOWER_MIN_Y && y <= (TRENCH_LOWER_MAX_Y + buf);
      if (nearUpper || nearLower) return true;
    }

    // --- Red-side trench (X 11.34–12.54, point-symmetric) ---
    // Y bands are swapped: blue upper → red lower, blue lower → red upper
    boolean inRedTrenchX = x >= (TRENCH_RED_MIN_X - buf) && x <= (TRENCH_RED_MAX_X + buf);
    if (inRedTrenchX) {
      boolean nearUpper = y >= (TRENCH_UPPER_MIN_Y - buf) && y <= TRENCH_UPPER_MAX_Y;
      boolean nearLower = y >= TRENCH_LOWER_MIN_Y && y <= (TRENCH_LOWER_MAX_Y + buf);
      if (nearUpper || nearLower) return true;
    }

    return false;
  }

  /**
   * Check if an alliance-aware position is near either trench (flips red to blue first). Used by
   * the auto path generator to snap heading to cardinal BEFORE the robot enters the trench.
   */
  public static boolean isNearTrenchAlliance(Translation2d alliancePosition) {
    Translation2d blue = isRedAlliance() ? flipTranslation(alliancePosition) : alliancePosition;
    return isNearTrench(blue);
  }

  /**
   * Snap an angle to the nearest cardinal direction (0°, 90°, 180°, 270°). Used by the auto path
   * generator when the robot's target pose is near a TRENCH, so it arrives aligned to pass through
   * the 22.25in tunnel.
   *
   * @param currentHeading The heading to snap
   * @return The nearest cardinal Rotation2d (0, 90, 180, or -90 degrees)
   */
  public static Rotation2d snapToCardinal(Rotation2d currentHeading) {
    double degrees = currentHeading.getDegrees();
    // Normalize to [0, 360)
    degrees = ((degrees % 360) + 360) % 360;
    // Round to nearest 90°
    double snapped = Math.round(degrees / 90.0) * 90.0;
    // Wrap 360 → 0
    if (snapped >= 360.0) snapped = 0.0;
    return Rotation2d.fromDegrees(snapped);
  }

  /**
   * Snap an angle to the nearest horizontal direction (0° or 180°). Used when the intake is
   * deployed — the robot is wider along 90°/270° and must only travel through trenches at 0° or
   * 180°.
   *
   * @param currentHeading The heading to snap
   * @return The nearest horizontal Rotation2d (0 or 180 degrees)
   */
  public static Rotation2d snapToHorizontal(Rotation2d currentHeading) {
    double degrees = currentHeading.getDegrees();
    // Normalize to [0, 360)
    degrees = ((degrees % 360) + 360) % 360;
    // Round to nearest 180°
    double snapped = Math.round(degrees / 180.0) * 180.0;
    // Wrap 360 → 0
    if (snapped >= 360.0) snapped = 0.0;
    return Rotation2d.fromDegrees(snapped);
  }

  // ===== Teleop Trench Assist Helpers =====

  /** Center Y of the upper trench (midpoint of Y band). */
  public static final double TRENCH_UPPER_CENTER_Y =
      (TRENCH_UPPER_MIN_Y + TRENCH_UPPER_MAX_Y) / 2.0;

  /** Center Y of the lower trench (midpoint of Y band). */
  public static final double TRENCH_LOWER_CENTER_Y =
      (TRENCH_LOWER_MIN_Y + TRENCH_LOWER_MAX_Y) / 2.0;

  /**
   * Get the center Y of the nearest trench if the robot is near one, otherwise {@code Double.NaN}.
   *
   * @param bluePosition Robot position in blue-origin coordinates
   * @param buffer Approach buffer distance (meters) outside trench walls
   * @return The center Y of the nearest trench, or {@code Double.NaN} if not near any trench
   */
  public static double getNearestTrenchCenterY(Translation2d bluePosition, double buffer) {
    double x = bluePosition.getX();
    double y = bluePosition.getY();

    boolean inTrenchX =
        (x >= (TRENCH_MIN_X - buffer) && x <= (TRENCH_MAX_X + buffer))
            || (x >= (TRENCH_RED_MIN_X - buffer) && x <= (TRENCH_RED_MAX_X + buffer));

    if (!inTrenchX) return Double.NaN;

    // Check upper trench (with buffer)
    boolean nearUpper = y >= (TRENCH_UPPER_MIN_Y - buffer) && y <= TRENCH_UPPER_MAX_Y;
    // Check lower trench (with buffer)
    boolean nearLower = y >= TRENCH_LOWER_MIN_Y && y <= (TRENCH_LOWER_MAX_Y + buffer);

    if (nearUpper && nearLower) {
      // Shouldn't happen (trenches are on opposite sides of field), but pick closest
      double distUpper = Math.abs(y - TRENCH_UPPER_CENTER_Y);
      double distLower = Math.abs(y - TRENCH_LOWER_CENTER_Y);
      return distUpper < distLower ? TRENCH_UPPER_CENTER_Y : TRENCH_LOWER_CENTER_Y;
    } else if (nearUpper) {
      return TRENCH_UPPER_CENTER_Y;
    } else if (nearLower) {
      return TRENCH_LOWER_CENTER_Y;
    }
    return Double.NaN;
  }

  /**
   * Compute a blend factor [0, 1] representing how strongly the trench heading assist should
   * influence the velocity vector. Returns 0 if the robot is outside the approach buffer, and ramps
   * linearly up to {@code maxBlend} as the robot moves from the buffer edge to the trench interior.
   *
   * @param bluePosition The robot position in blue-origin coordinates
   * @param maxBlend The maximum blend factor
   * @param buffer Approach buffer distance (meters) outside trench walls
   * @return Blend factor in [0, maxBlend]. 0 = no assist.
   */
  public static double getTrenchBlendFactor(
      Translation2d bluePosition, double maxBlend, double buffer) {
    double x = bluePosition.getX();
    double y = bluePosition.getY();

    double bestBlend = 0.0;

    // --- Blue-side trenches (X 4.0–5.2) ---
    double blueXBlend = trenchXBlend(x, TRENCH_MIN_X, TRENCH_MAX_X, buffer);
    if (blueXBlend > 0.0) {
      bestBlend = Math.max(bestBlend, blueXBlend * trenchYBlend(y, buffer));
    }

    // --- Red-side trenches (X 11.34–12.54, point-symmetric) ---
    double redXBlend = trenchXBlend(x, TRENCH_RED_MIN_X, TRENCH_RED_MAX_X, buffer);
    if (redXBlend > 0.0) {
      bestBlend = Math.max(bestBlend, redXBlend * trenchYBlend(y, buffer));
    }

    return bestBlend * maxBlend;
  }

  /**
   * Internal: compute a 0–1 blend based on how deep into a trench Y-band the position is. Ramps
   * from 0 at the buffer edge to 1 at the trench wall.
   */
  private static double trenchYBlend(double y, double buf) {
    double blend = 0.0;

    // Upper trench: Y in [TRENCH_UPPER_MIN_Y - buf, FIELD_WIDTH]
    if (y >= (TRENCH_UPPER_MIN_Y - buf) && y <= TRENCH_UPPER_MAX_Y) {
      if (y >= TRENCH_UPPER_MIN_Y) {
        blend = Math.max(blend, 1.0); // inside trench
      } else {
        // In buffer zone: ramp from 0 at (MIN_Y - buf) to 1 at MIN_Y
        blend = Math.max(blend, (y - (TRENCH_UPPER_MIN_Y - buf)) / buf);
      }
    }

    // Lower trench: Y in [0, TRENCH_LOWER_MAX_Y + buf]
    if (y >= TRENCH_LOWER_MIN_Y && y <= (TRENCH_LOWER_MAX_Y + buf)) {
      if (y <= TRENCH_LOWER_MAX_Y) {
        blend = Math.max(blend, 1.0); // inside trench
      } else {
        // In buffer zone: ramp from 1 at MAX_Y to 0 at (MAX_Y + buf)
        blend = Math.max(blend, 1.0 - (y - TRENCH_LOWER_MAX_Y) / buf);
      }
    }

    return Math.max(0.0, Math.min(1.0, blend));
  }

  /**
   * Internal: compute a 0–1 multiplier based on how deep into a trench X-band the position is.
   * Returns 1.0 when fully inside the trench X range, ramps linearly to 0 at the buffer edge. This
   * ensures the assist fades smoothly as the robot exits the trench along the X axis.
   */
  private static double trenchXBlend(double x, double minX, double maxX, double buf) {
    if (x >= minX && x <= maxX) {
      return 1.0; // fully inside trench X range
    } else if (x >= (minX - buf) && x < minX) {
      // Approaching from the low-X side: ramp 0→1
      return (x - (minX - buf)) / buf;
    } else if (x > maxX && x <= (maxX + buf)) {
      // Exiting from the high-X side: ramp 1→0
      return 1.0 - (x - maxX) / buf;
    }
    return 0.0;
  }

  /**
   * Apply trench lateral centering to a teleop velocity vector. When the robot is near a trench and
   * moving roughly toward it, the velocity <b>direction</b> is deflected toward the trench's center
   * Y line, pulling the travel path toward the middle of the 48in corridor. Speed magnitude is
   * preserved exactly.
   *
   * <p>This method handles <b>only lateral centering</b>. Chassis orientation alignment (rotating
   * the robot's heading to a cardinal) is handled separately by {@link #getTrenchOrientationOmega}.
   *
   * @param bluePosition Robot position in blue-origin coordinates
   * @param vxFieldMps Field-relative X velocity (m/s)
   * @param vyFieldMps Field-relative Y velocity (m/s)
   * @param maxBlend Maximum blend factor (0–1)
   * @param minSpeedMps Minimum speed for assist activation (m/s)
   * @param maxHeadingErrorDeg Maximum angular error from cardinal for vel-direction gating
   *     (degrees)
   * @param centeringDegPerMeter Degrees of deflection per meter of lateral offset from center
   * @param maxCenteringDeg Maximum centering deflection angle (degrees)
   * @param buffer Approach buffer distance (meters) outside trench walls
   * @param robotHalfWidthM Half the robot's bumper width (meters), used for wall avoidance
   * @param wallRepulsionMpsPerMeter Wall repulsion gain (m/s of lateral push per meter of
   *     encroachment into the danger zone)
   * @param wallDangerZoneM Distance from trench wall (inward) at which repulsion begins (meters)
   * @return double[3] = {adjusted vx, adjusted vy, centering deflection degrees applied}
   */
  public static double[] applyTrenchAssist(
      Translation2d bluePosition,
      double vxFieldMps,
      double vyFieldMps,
      double maxBlend,
      double minSpeedMps,
      double maxHeadingErrorDeg,
      double centeringDegPerMeter,
      double maxCenteringDeg,
      double buffer,
      double robotHalfWidthM,
      double wallRepulsionMpsPerMeter,
      double wallDangerZoneM) {

    double speed = Math.sqrt(vxFieldMps * vxFieldMps + vyFieldMps * vyFieldMps);

    // Compute blend factor from proximity (used for centering & wall clamping)
    double blend = getTrenchBlendFactor(bluePosition, maxBlend, buffer);

    // === Wall avoidance (always active when near trench, even at low speed) ===
    // Prevents the robot's bumper edges from hitting the trench walls.
    // Works by clamping vy: if the bumper edge is inside the danger zone near a wall,
    // inject a lateral push away from the wall and suppress vy toward the wall.
    double wallVyAdjust = 0.0;
    if (blend > 1e-4) {
      wallVyAdjust =
          computeWallRepulsion(
              bluePosition,
              vyFieldMps,
              robotHalfWidthM,
              wallRepulsionMpsPerMeter,
              wallDangerZoneM,
              blend,
              buffer);
    }

    // Apply wall adjustment to vy
    double adjustedVy = vyFieldMps + wallVyAdjust;

    // Below speed threshold — only wall avoidance, no centering
    if (speed < minSpeedMps) {
      return new double[] {vxFieldMps, adjustedVy, 0.0};
    }

    if (blend < 1e-4) {
      return new double[] {vxFieldMps, vyFieldMps, 0.0};
    }

    // Check velocity heading gate — only activate centering when traveling roughly along
    // the X axis (0° or 180°), meaning the driver intends to go THROUGH the trench.
    // Driving along Y (90°/270°) means parallel to / across the trench — don't assist.
    Rotation2d velocityAngle = new Rotation2d(vxFieldMps, vyFieldMps);
    double velDeg = velocityAngle.getDegrees(); // −180 to +180
    double errorFrom0 = Math.abs(velDeg); // distance from 0°
    double errorFrom180 = 180.0 - Math.abs(velDeg); // distance from ±180°
    double errorFromXAxis = Math.min(errorFrom0, errorFrom180);
    if (errorFromXAxis > maxHeadingErrorDeg) {
      return new double[] {vxFieldMps, adjustedVy, 0.0};
    }

    // === Lateral centering toward trench mid-Y ===
    // Uses a direction-agnostic approach: directly compute a vy offset toward center,
    // then reconstruct the velocity with preserved speed magnitude.
    double centeringDeg = 0.0;
    double trenchCenterY = getNearestTrenchCenterY(bluePosition, buffer);
    if (!Double.isNaN(trenchCenterY)) {
      double yOffset = bluePosition.getY() - trenchCenterY; // positive = above center
      double rawCenteringDeg = yOffset * centeringDegPerMeter;
      centeringDeg =
          Math.copySign(Math.min(Math.abs(rawCenteringDeg), maxCenteringDeg), rawCenteringDeg);
      // Scale by blend factor so centering fades at the buffer edge
      centeringDeg *= blend;
    }

    // Convert centering to a direct vy correction (direction-agnostic).
    // Positive yOffset → positive centeringDeg → robot is above center → push vy negative (−Y).
    // This works regardless of whether the robot is traveling at 0° or 180°.
    double centeringFraction = Math.sin(Math.toRadians(centeringDeg));
    double lateralCorrectionMps = -centeringFraction * speed;

    // Apply centering to the already wall-adjusted vy
    double newVy = adjustedVy + lateralCorrectionMps;
    // Preserve speed magnitude: scale vx so total speed stays the same
    double newVx =
        Math.copySign(Math.sqrt(Math.max(0.0, speed * speed - newVy * newVy)), vxFieldMps);

    return new double[] {newVx, newVy, centeringDeg};
  }

  /**
   * Compute wall repulsion vy adjustment. When the robot's bumper edge is within the danger zone of
   * a trench wall, this pushes the robot laterally away from the wall and suppresses vy toward the
   * wall.
   *
   * <p>The trench walls are at the Y boundaries: upper trench walls at {@code TRENCH_UPPER_MIN_Y}
   * (inner / field-center side) and {@code TRENCH_UPPER_MAX_Y} (outer / field perimeter), lower
   * trench walls at {@code TRENCH_LOWER_MIN_Y} (outer / field perimeter) and {@code
   * TRENCH_LOWER_MAX_Y} (inner / field-center side).
   *
   * <p><b>Field-edge (outer) walls</b> are the field perimeter — they exist everywhere, so their
   * repulsion activates in the approach buffer zone (X range + buffer). <b>Field-center (inner)
   * walls</b> are the trench structure — they only exist within the trench X range, so their
   * repulsion is restricted to the trench X range only.
   */
  private static double computeWallRepulsion(
      Translation2d bluePosition,
      double vyFieldMps,
      double robotHalfWidthM,
      double repulsionGain,
      double dangerZoneM,
      double blend,
      double buffer) {

    double x = bluePosition.getX();
    double y = bluePosition.getY();
    double vyAdjust = 0.0;

    // Determine X-zone membership:
    // - "in trench X" = inside the actual trench structure X range (inner walls exist here)
    // - "in buffer X" = inside the approach buffer around the trench (outer/field walls still
    // apply)
    boolean inBlueTrenchX = x >= TRENCH_MIN_X && x <= TRENCH_MAX_X;
    boolean inBlueBufferX = x >= (TRENCH_MIN_X - buffer) && x <= (TRENCH_MAX_X + buffer);
    boolean inRedTrenchX = x >= TRENCH_RED_MIN_X && x <= TRENCH_RED_MAX_X;
    boolean inRedBufferX = x >= (TRENCH_RED_MIN_X - buffer) && x <= (TRENCH_RED_MAX_X + buffer);

    boolean inAnyTrenchX = inBlueTrenchX || inRedTrenchX;
    boolean inAnyBufferX = inBlueBufferX || inRedBufferX;

    // No repulsion at all if outside even the buffer zone
    if (!inAnyBufferX) {
      return 0.0;
    }

    // Upper trench:
    //   Inner wall (TRENCH_UPPER_MIN_Y, field-center side) — only in trench X range
    //   Outer wall (TRENCH_UPPER_MAX_Y, field perimeter) — in buffer X range (starts earlier)
    if (y >= (TRENCH_UPPER_MIN_Y - buffer) && y <= TRENCH_UPPER_MAX_Y) {
      // Inner wall — field-center side (trench structure, only exists in trench X range)
      if (inAnyTrenchX) {
        double distToInnerWall = y - robotHalfWidthM - TRENCH_UPPER_MIN_Y;
        if (distToInnerWall < dangerZoneM) {
          double encroachment = dangerZoneM - distToInnerWall;
          // Push toward +Y (away from inner wall, toward field edge)
          vyAdjust += encroachment * repulsionGain * blend;
        }
      }
      // Outer wall — field perimeter (exists everywhere, active in buffer X range)
      double distToOuterWall = TRENCH_UPPER_MAX_Y - (y + robotHalfWidthM);
      if (distToOuterWall < dangerZoneM) {
        double encroachment = dangerZoneM - distToOuterWall;
        // Push toward −Y (away from field edge)
        vyAdjust -= encroachment * repulsionGain * blend;
      }
    }

    // Lower trench:
    //   Inner wall (TRENCH_LOWER_MAX_Y, field-center side) — only in trench X range
    //   Outer wall (TRENCH_LOWER_MIN_Y = 0, field perimeter) — in buffer X range (starts earlier)
    if (y >= TRENCH_LOWER_MIN_Y && y <= (TRENCH_LOWER_MAX_Y + buffer)) {
      // Inner wall — field-center side (trench structure, only exists in trench X range)
      if (inAnyTrenchX) {
        double distToInnerWall = TRENCH_LOWER_MAX_Y - (y + robotHalfWidthM);
        if (distToInnerWall < dangerZoneM) {
          double encroachment = dangerZoneM - distToInnerWall;
          // Push toward −Y (away from inner wall, toward field edge)
          vyAdjust -= encroachment * repulsionGain * blend;
        }
      }
      // Outer wall — field perimeter (exists everywhere, active in buffer X range)
      double distToOuterWall = (y - robotHalfWidthM) - TRENCH_LOWER_MIN_Y;
      if (distToOuterWall < dangerZoneM) {
        double encroachment = dangerZoneM - distToOuterWall;
        // Push toward +Y (away from field edge)
        vyAdjust += encroachment * repulsionGain * blend;
      }
    }

    return vyAdjust;
  }

  /**
   * Compute an omega (rotational velocity) correction to align the robot's <b>chassis heading</b>
   * toward the nearest cardinal direction when near a trench. This is a P-controller on the heading
   * error, scaled by the proximity blend factor.
   *
   * <p>The correction is <b>added to</b> the driver's omega input, so the driver retains partial
   * rotational authority. The correction is capped at {@code maxOmega} to avoid violent spinning.
   *
   * <p>Returns 0 if the robot is outside the trench zone, moving too slowly, or traveling
   * perpendicular to the trench (velocity heading gate).
   *
   * @param bluePosition Robot position in blue-origin coordinates
   * @param robotHeading Current robot chassis heading (field-relative)
   * @param vxFieldMps Field-relative X velocity (m/s, for velocity heading gate)
   * @param vyFieldMps Field-relative Y velocity (m/s, for velocity heading gate)
   * @param maxBlend Maximum blend factor (0–1)
   * @param minSpeedMps Minimum speed for activation (m/s)
   * @param maxHeadingErrorDeg Velocity heading gate (degrees from cardinal)
   * @param orientationKp P-gain (rad/s per radian of chassis heading error)
   * @param maxOmega Maximum omega correction magnitude (rad/s)
   * @param buffer Approach buffer distance (meters) outside trench walls
   * @param intakeDeployed If true, snap heading to 0°/180° only instead of all four cardinals
   * @return Omega correction in rad/s to add to the driver's omega. Positive = CCW.
   */
  public static double getTrenchOrientationOmega(
      Translation2d bluePosition,
      Rotation2d robotHeading,
      double vxFieldMps,
      double vyFieldMps,
      double maxBlend,
      double minSpeedMps,
      double maxHeadingErrorDeg,
      double orientationKp,
      double maxOmega,
      double buffer,
      boolean intakeDeployed) {

    double speed = Math.sqrt(vxFieldMps * vxFieldMps + vyFieldMps * vyFieldMps);
    if (speed < minSpeedMps) {
      return 0.0;
    }

    double blend = getTrenchBlendFactor(bluePosition, maxBlend, buffer);
    if (blend < 1e-4) {
      return 0.0;
    }

    // Velocity heading gate — only activate when traveling roughly along X axis (0° or 180°).
    // Driving along Y (90°/270°) means the driver is not going through the trench.
    Rotation2d velocityAngle = new Rotation2d(vxFieldMps, vyFieldMps);
    double velDeg = velocityAngle.getDegrees();
    double errorFrom0 = Math.abs(velDeg);
    double errorFrom180 = 180.0 - Math.abs(velDeg);
    double errorFromXAxis = Math.min(errorFrom0, errorFrom180);
    if (errorFromXAxis > maxHeadingErrorDeg) {
      return 0.0;
    }

    // Snap the *robot chassis heading* to the nearest cardinal (or horizontal if intake deployed)
    Rotation2d targetHeading =
        intakeDeployed ? snapToHorizontal(robotHeading) : snapToCardinal(robotHeading);
    // Compute heading error (shortest path)
    double headingErrorRad = targetHeading.minus(robotHeading).getRadians();

    // P-controller on heading error, scaled by proximity blend
    double omega = headingErrorRad * orientationKp * blend;

    // Cap the correction
    omega = Math.copySign(Math.min(Math.abs(omega), maxOmega), omega);

    return omega;
  }

  // ===== HUB 3D Positions =====
  // Used by ShooterSetpoint for distance-based aiming
  // Point-symmetric: RED HUB is 180° rotated from BLUE HUB about field center
  public static final Translation3d BLUE_HUB_TRANSLATION3D =
      new Translation3d(4.625689, 4.040981, 0);
  public static final Translation3d RED_HUB_POSE_TRANSLATION3D =
      new Translation3d(FIELD_LENGTH - 4.625689, FIELD_WIDTH - 4.040981, 0);

  // ===== Neutral Zone Aim Targets =====
  // When in the neutral zone, the robot aims at one of two targets instead of the alliance wall.
  // If robot Y > FIELD_WIDTH/2 → aim at upper target; Y < FIELD_WIDTH/2 → aim at lower target.
  // Point-symmetric: RED targets are 180° rotated from BLUE targets about field center.
  public static final Translation3d BLUE_NEUTRAL_ZONE_UPPER_AIM =
      new Translation3d(2.8, FIELD_WIDTH * 0.75, 0);
  public static final Translation3d BLUE_NEUTRAL_ZONE_LOWER_AIM =
      new Translation3d(2.8, FIELD_WIDTH * 0.25, 0);
  public static final Translation3d RED_NEUTRAL_ZONE_UPPER_AIM =
      new Translation3d(FIELD_LENGTH - 2.8, FIELD_WIDTH * 0.75, 0);
  public static final Translation3d RED_NEUTRAL_ZONE_LOWER_AIM =
      new Translation3d(FIELD_LENGTH - 2.8, FIELD_WIDTH * 0.25, 0);

  // ===== Zones =====
  // Zones define rectangular regions of the REBUILT field (blue-alliance origin).
  // Boundaries are axis-aligned rectangles: (minX, minY) to (maxX, maxY) in meters.
  // The planner uses these for constraint-checking and spatial queries.
  //
  // NOTE: AutoTuning.AIMING_ZONE_MAX_X uses NEUTRAL_ZONE.minX (5.50m) as the aiming
  // threshold. The turret starts aiming when the robot leaves the neutral zone (enters HUB zone).
  // The hood stays stowed independently via trench detection until the robot clears the trench.
  //
  // Field layout (blue origin, X right, Y up):
  //   x=0.00        x=2.00    x=3.60    x=5.2  x=5.50       x=8.27         x=11.04       x=16.54
  //   |  OUTPOST    |         | ROBOT   |TRENCH| HUB_ZONE   | NEUTRAL      | OPPONENT    |
  //   |  (corner)   | ALLIANCE| STARTING|  MAX |             | ZONE         | SIDE        |
  //   |             | ZONE    | LINE    |      |             |              |             |
  //
  public enum Zone {
    /**
     * ALLIANCE ZONE — behind the ROBOT STARTING LINE (x=3.60m), contains our TOWER and DEPOT. Spans
     * full field width. Safe territory.
     */
    ALLIANCE_ZONE(0.0, 0.0, 3.60, FIELD_WIDTH),

    /**
     * OUTPOST AREA — human player station corner with CHUTE and CORRAL. Overlaps the lower-left
     * corner of the ALLIANCE ZONE. Checked first for more specific matching.
     */
    OUTPOST_AREA(0.0, 0.0, 2.00, 2.00),

    /**
     * HUB ZONE — area around our HUB where we shoot FUEL. Between the ROBOT STARTING LINE (x=3.60m)
     * and the edge of the NEUTRAL ZONE (x=5.50m). Contains all scoring waypoints.
     */
    HUB_ZONE(3.60, 0.0, 5.50, FIELD_WIDTH),

    /**
     * NEUTRAL ZONE — center of field with scattered FUEL (~360-408). Contested territory between
     * x=5.50m and x=11.04m (symmetric about field center x=8.27m).
     */
    NEUTRAL_ZONE(5.50, 0.0, 10.884, FIELD_WIDTH),

    /**
     * OPPONENT SIDE — opponent's half of the field beyond the NEUTRAL ZONE. Risky territory,
     * penalties possible for FUEL interference.
     */
    OPPONENT_SIDE(10.884, 0.0, FIELD_LENGTH, FIELD_WIDTH);

    /** Blue-origin bounding box corners (meters). */
    public final double minX, minY, maxX, maxY;

    Zone(double minX, double minY, double maxX, double maxY) {
      this.minX = minX;
      this.minY = minY;
      this.maxX = maxX;
      this.maxY = maxY;
    }

    /** Check if a blue-origin point is inside this zone's bounding box. */
    public boolean contains(Translation2d bluePoint) {
      return bluePoint.getX() >= minX
          && bluePoint.getX() <= maxX
          && bluePoint.getY() >= minY
          && bluePoint.getY() <= maxY;
    }

    /** Check if an alliance-aware point is inside this zone (flips red to blue first). */
    public boolean containsAlliance(Translation2d alliancePoint) {
      Translation2d blue = isRedAlliance() ? flipTranslation(alliancePoint) : alliancePoint;
      return contains(blue);
    }

    /**
     * Get the most specific zone for a blue-origin position. Checks OUTPOST_AREA first since it
     * overlaps ALLIANCE_ZONE, then checks remaining zones in order.
     */
    public static Zone getZone(Translation2d bluePosition) {
      // Check OUTPOST first — it overlaps ALLIANCE_ZONE
      if (OUTPOST_AREA.contains(bluePosition)) return OUTPOST_AREA;
      for (Zone z : values()) {
        if (z == OUTPOST_AREA) continue; // already checked
        if (z.contains(bluePosition)) return z;
      }
      return NEUTRAL_ZONE; // fallback
    }

    /** Get the zone for an alliance-aware position (flips red to blue first). */
    public static Zone getZoneAlliance(Translation2d alliancePosition) {
      Translation2d blue = isRedAlliance() ? flipTranslation(alliancePosition) : alliancePosition;
      return getZone(blue);
    }
  }

  // ===== Scoring Waypoints =====
  public enum ScoringWaypoint {
    /** Upper — shooting from the upper side. */
    HUB_UPPER(new Translation2d(3.0, 7.44), Zone.ALLIANCE_ZONE),
    /** Center — shooting straight at the HUB from the NEUTRAL ZONE side. */
    HUB_CENTER(new Translation2d(2.50, 4.03), Zone.ALLIANCE_ZONE),
    /** Lower — shooting from the lower side. */
    HUB_LOWER(new Translation2d(3.0, 0.652), Zone.ALLIANCE_ZONE);

    public final Translation2d bluePosition;
    public final Zone zone;

    ScoringWaypoint(Translation2d bluePosition, Zone zone) {
      this.bluePosition = bluePosition;
      this.zone = zone;
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
    /** OUTPOST — human player CHUTE delivers FUEL. Located at lower end of the field (low Y). */
    OUTPOST(new Pose2d(0.495, 0.656, Rotation2d.fromDegrees(0)), Zone.OUTPOST_AREA, false),
    /** DEPOT — floor-level FUEL bin along ALLIANCE WALL (high Y, upper side). */
    DEPOT(new Pose2d(0.665, 5.962, Rotation2d.fromDegrees(0)), Zone.ALLIANCE_ZONE, false),

    /** NEUTRAL ZONE upper — pick up FUEL from the upper side of the neutral zone. */
    NEUTRAL_ZONE_UPPER(
        new Pose2d(7.84, 5.405, Rotation2d.fromDegrees(90)), Zone.NEUTRAL_ZONE, true),
    /** NEUTRAL ZONE lower — pick up FUEL from the lower side of the neutral zone. */
    NEUTRAL_ZONE_LOWER(
        new Pose2d(7.84, 2.665, Rotation2d.fromDegrees(-90)), Zone.NEUTRAL_ZONE, true);

    public final Pose2d bluePose;
    public final Zone zone;

    /**
     * Whether this intake location can be visited multiple times. OUTPOST and DEPOT are one-shot
     * (depleted after a single intake visit). NEUTRAL ZONE locations have scattered FUEL that can
     * be collected repeatedly.
     */
    public final boolean reusable;

    IntakeLocation(Pose2d bluePose, Zone zone, boolean reusable) {
      this.bluePose = bluePose;
      this.zone = zone;
      this.reusable = reusable;
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
    UPPER(new Pose2d(3.6, 7.444, Rotation2d.fromDegrees(0))),
    /** Starting pose near DRIVER STATION 2 (center, near TOWER). */
    CENTER(new Pose2d(3.6, 4.03, Rotation2d.fromDegrees(0))),
    /** Starting pose near DRIVER STATION 3 (lower side, near DEPOT). */
    LOWER(new Pose2d(3.6, 0.652, Rotation2d.fromDegrees(0)));

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
    LEVEL_1(10, 15, 1.0),
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

  // ===== Estimated action durations (seconds) =====
  // Runtime time-check budgeting uses the constants in AutoTuning:
  //   - STOP_AND_SHOOT_DURATION (1.5s) — stop, aim, fire
  //   - SWD_SCORE_DURATION (2.0s)      — drive through hub zone while shooting
  //   - INTAKE_DWELL_ESTIMATE (1.0s)   — deceleration + FUEL pickup
  //
  // The planner uses these same constants (via AutoTuning) for consistent time budgets.

  /** AUTO period duration — 20 seconds per REBUILT game manual. */
  public static final double AUTO_DURATION = 20.0;

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
   * Flip a pose from blue origin to red origin. REBUILT has <b>point symmetry</b> (180° rotational
   * symmetry about the field center), so both X and Y are mirrored and the heading rotates 180°.
   */
  public static Pose2d flipPose(Pose2d bluePose) {
    return new Pose2d(
        FIELD_LENGTH - bluePose.getX(),
        FIELD_WIDTH - bluePose.getY(),
        bluePose.getRotation().plus(Rotation2d.fromDegrees(180)));
  }

  /**
   * Flip a translation from blue origin to red origin. Point-symmetric: both X and Y are mirrored
   * about the field center.
   */
  public static Translation2d flipTranslation(Translation2d blueTranslation) {
    return new Translation2d(
        FIELD_LENGTH - blueTranslation.getX(), FIELD_WIDTH - blueTranslation.getY());
  }

  /**
   * Estimate drive time between two poses using a trapezoidal motion profile. Accounts for
   * acceleration and deceleration phases instead of assuming a flat average speed.
   *
   * <p>For short drives where the robot never reaches max velocity (triangle profile):
   *
   * <pre>t = 2 × √(d / a)</pre>
   *
   * For longer drives where max velocity is reached (trapezoid profile):
   *
   * <pre>t = 2 × (v / a) + (d - v² / a) / v</pre>
   *
   * <p>The straight-line distance is inflated by {@link
   * Constants.AutoConstants#PATH_DISTANCE_DERATING} to account for AD* paths being longer than
   * straight-line (curves, obstacle avoidance). Since the trapezoidal profile handles accel/decel
   * physically, the derating only covers path curvature (~11% with the default 0.9 factor).
   *
   * @param from Start pose
   * @param to End pose
   * @return Estimated drive time in seconds
   */
  public static double estimateDriveTime(Pose2d from, Pose2d to) {
    double straightLine = from.getTranslation().getDistance(to.getTranslation());
    // Inflate for path curvature / obstacle avoidance
    double distance = straightLine / Constants.AutoConstants.PATH_DISTANCE_DERATING;
    double maxVel = Constants.AutoConstants.PATHFINDING_MAX_VELOCITY_MPS;
    double accel = Constants.AutoConstants.PATHFINDING_MAX_ACCELERATION_MPS2;

    // Distance needed to ramp from 0 to maxVel (and back down)
    double rampDistance = (maxVel * maxVel) / (2.0 * accel);

    if (distance <= 2.0 * rampDistance) {
      // Triangle profile: never reaches max velocity
      return 2.0 * Math.sqrt(distance / accel);
    } else {
      // Trapezoid profile: accel + cruise + decel
      double rampTime = maxVel / accel;
      double cruiseDistance = distance - 2.0 * rampDistance;
      double cruiseTime = cruiseDistance / maxVel;
      return 2.0 * rampTime + cruiseTime;
    }
  }
}
