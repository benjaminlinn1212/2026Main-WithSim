package frc.robot.subsystems.climb.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ClimbConstants;

/**
 * ═══════════════════════════════════════════════════════════════════════════ INVERSE KINEMATICS -
 * Cable-Driven 2-Link Climb Mechanism
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * <p>Converts end effector position (x, y) → motor rotations for all 4 motors.
 *
 * <p>MECHANISM GEOMETRY: W1 = (0, 0) - Winch 1 (front motor) W2 = (a, 0) - Winch 2 (back motor) E =
 * (xe, ye) - End effector target position J = (xj, yj) - Joint (elbow) position
 *
 * <p>Rigid links: |W2 - J| = b - Link 1 (shoulder link) |J - E| = c - Link 2 (forearm link)
 *
 * <p>Cable attachment offsets: P on link b: p meters from J toward W2 Q on link c: q meters from E
 * toward J
 *
 * <p>Cable lengths: l1 = |P - W1| - Front cable l2 = |Q - W2| - Back cable
 *
 * <p>MOTOR CALCULATIONS: 1. Cable length (meters) 2. Cable length → Drum rotations (divide by drum
 * circumference) 3. Drum rotations → Motor rotations (multiply by gear ratio)
 *
 * <p>Example: If motor has 10:1 gearbox, motor spins 10x for each drum rotation
 *
 * <p>COORDINATE SYSTEM: X = Forward (away from robot) Y = Up (vertical)
 */
public class ClimbIK {

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  // Result Classes
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  /** IK solution for one side (4 motors total, 2 per side) */
  public static class ClimbSideIKResult {
    public final double frontMotorRotations; // Cable 1
    public final double backMotorRotations; // Cable 2
    public final boolean isValid;

    // Debug info
    public final double jointX;
    public final double jointY;

    public ClimbSideIKResult(
        double frontMotorRotations,
        double backMotorRotations,
        boolean isValid,
        double jointX,
        double jointY) {
      this.frontMotorRotations = frontMotorRotations;
      this.backMotorRotations = backMotorRotations;
      this.isValid = isValid;
      this.jointX = jointX;
      this.jointY = jointY;
    }

    public static ClimbSideIKResult invalid() {
      return new ClimbSideIKResult(0.0, 0.0, false, 0.0, 0.0);
    }
  }

  /** Complete IK solution for both sides */
  public static class ClimbIKResult {
    public final ClimbSideIKResult leftSide;
    public final ClimbSideIKResult rightSide;

    public ClimbIKResult(ClimbSideIKResult leftSide, ClimbSideIKResult rightSide) {
      this.leftSide = leftSide;
      this.rightSide = rightSide;
    }

    public boolean isValid() {
      return leftSide.isValid && rightSide.isValid;
    }
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  // Core IK Solver
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  /**
   * Solve IK for one side of the climb mechanism.
   *
   * @param xe End effector X position (meters)
   * @param ye End effector Y position (meters)
   * @return IK solution with motor rotations
   */
  public static ClimbSideIKResult calculateIK(double xe, double ye) {
    // Mechanism parameters from constants
    final double a = ClimbConstants.WINCH_SEPARATION_METERS; // Distance between winches
    final double b = ClimbConstants.LINK_1_LENGTH_METERS; // Shoulder link
    final double c = ClimbConstants.LINK_2_LENGTH_METERS; // Forearm link
    final double p = ClimbConstants.CABLE_1_OFFSET_METERS; // Cable 1 attachment offset
    final double q = ClimbConstants.CABLE_2_OFFSET_METERS; // Cable 2 attachment offset
    final double eps = 1e-9;

    // Vector from W2=(a,0) to E=(xe,ye)
    final double dx = xe - a;
    final double dy = ye;
    final double r = Math.hypot(dx, dy);

    // ─── Reachability Check ───
    if (r < eps) return ClimbSideIKResult.invalid(); // Too close
    if (r > b + c + 1e-12) return ClimbSideIKResult.invalid(); // Too far
    if (r < Math.abs(b - c) - 1e-12) return ClimbSideIKResult.invalid(); // Inside dead zone

    // ─── Two-Circle Intersection (Find Joint J) ───
    // Circle 1: Center at W2=(a,0), radius b
    // Circle 2: Center at E=(xe,ye), radius c
    // We want the LEFT (CCW) intersection point

    final double d =
        (b * b - c * c + r * r) / (2.0 * r); // Distance along W2->E to intersection midpoint

    double h2 = b * b - d * d; // Perpendicular distance squared
    if (h2 < 0 && h2 > -1e-10) h2 = 0; // Clamp tiny FP errors
    if (h2 < 0) return ClimbSideIKResult.invalid();
    final double h = Math.sqrt(h2);

    // Unit vector from W2 to E
    final double ux = dx / r;
    final double uy = dy / r;

    // CCW perpendicular (left side)
    final double upx = -uy;
    final double upy = ux;

    // Joint J = W2 + d*u + h*u_perp
    final double xj = a + d * ux + h * upx;
    final double yj = 0 + d * uy + h * upy;

    // ─── Cable Attachment Points ───

    // Point P on link b: p meters from J toward W2
    final double t1 = p / b;
    final double px = xj + t1 * (a - xj);
    final double py = yj + t1 * (0 - yj);

    // Point Q on link c: q meters from E toward J
    final double t2 = q / c;
    final double qx = xe + t2 * (xj - xe);
    final double qy = ye + t2 * (yj - ye);

    // ─── Cable Lengths ───
    final double l1 = Math.hypot(px, py); // Cable 1: P to W1=(0,0)
    final double l2 = Math.hypot(qx - a, qy); // Cable 2: Q to W2=(a,0)

    // ─── Convert Cable Lengths to Motor Rotations ───
    // Step 1: Cable length to drum rotations
    final double drumCircumference = Math.PI * ClimbConstants.CABLE_DRUM_DIAMETER_METERS;
    final double drumRotations_front = l1 / drumCircumference;
    final double drumRotations_back = l2 / drumCircumference;

    // Step 2: Drum rotations to motor rotations (account for gear ratio)
    // Front motors: 100:1 reduction → motor rotations = drum rotations / gear_ratio
    // Back motors: 80:1 reduction → motor rotations = drum rotations / gear_ratio
    final double frontMotorRotations = drumRotations_front / ClimbConstants.FRONT_GEAR_RATIO;
    final double backMotorRotations = drumRotations_back / ClimbConstants.BACK_GEAR_RATIO;

    return new ClimbSideIKResult(
        frontMotorRotations,
        backMotorRotations,
        true, // Valid solution
        xj,
        yj);
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  // Wrapper Methods (for our workflow)
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  /**
   * Calculate IK from Translation2d (matches our path workflow).
   *
   * @param endEffectorPosition Target position (x, y) in meters
   * @return IK solution with motor rotations
   */
  public static ClimbSideIKResult calculateIK(Translation2d endEffectorPosition) {
    return calculateIK(endEffectorPosition.getX(), endEffectorPosition.getY());
  }

  /**
   * Calculate IK for both sides (symmetric - same target).
   *
   * @param leftTarget Left side target
   * @param rightTarget Right side target
   * @return Complete IK solution for both sides
   */
  public static ClimbIKResult calculateBothSides(
      Translation2d leftTarget, Translation2d rightTarget) {
    ClimbSideIKResult leftResult = calculateIK(leftTarget);
    ClimbSideIKResult rightResult = calculateIK(rightTarget);
    return new ClimbIKResult(leftResult, rightResult);
  }

  /**
   * Calculate IK for both sides using same target (symmetric climb).
   *
   * @param targetPosition Target position for both sides
   * @return Complete IK solution
   */
  public static ClimbIKResult calculateSymmetric(Translation2d targetPosition) {
    return calculateBothSides(targetPosition, targetPosition);
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  // Validation & Limits
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  /**
   * Check if a position is reachable (within workspace).
   *
   * @param position Position to check
   * @return true if reachable
   */
  public static boolean isPositionReachable(Translation2d position) {
    ClimbSideIKResult result = calculateIK(position);
    return result.isValid;
  }
}
