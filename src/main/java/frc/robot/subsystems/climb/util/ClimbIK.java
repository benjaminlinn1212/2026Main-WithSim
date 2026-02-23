package frc.robot.subsystems.climb.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ClimbConstants;

/**
 * =========================================================================== INVERSE KINEMATICS -
 * Cable-Driven 2-Link Climb Mechanism
 * ===========================================================================
 *
 * <p>Converts end effector position (x, y) → motor rotations for all 4 motors.
 *
 * <p>MECHANISM GEOMETRY (all coords relative to back winch at origin):
 *
 * <pre>
 *   W_back  = (0, 0)             — Back winch (back cable drum)
 *   W_front = (wfx, wfy)         — Front winch (front cable drum)
 *   S       = (sx, sy)           — Shoulder joint (fixed pivot, link 1 base)
 *   J       = (jx, jy)           — Elbow joint (between links)
 *   E       = (ex, ey)           — End effector
 * </pre>
 *
 * <p>Rigid links: |S → J| = L1 (shoulder to elbow), |J → E| = L2 (elbow to end effector)
 *
 * <p>Cable attachment points:
 *
 * <ul>
 *   <li>P on link 1: BACK_CABLE_ATTACH meters from J toward S → back cable = |P − W_back|
 *   <li>Q on link 2: FRONT_CABLE_ATTACH meters from E toward J → front cable = |Q − W_front|
 * </ul>
 *
 * <p>COORDINATE SYSTEM: X = Forward (away from robot), Y = Up (vertical)
 */
public class ClimbIK {

  /**
   * Initial cable lengths at the start/stow position, computed from START_POSITION using the
   * mechanism geometry. Motors are zeroed at this position, so all IK outputs are relative to these
   * lengths. No need to hardcode — derived automatically from START_POSITION_X/Y.
   */
  private static final CableLengths INITIAL_CABLE_LENGTHS =
      calculateCableLengthsInternal(
          ClimbConstants.START_POSITION_X_METERS, ClimbConstants.START_POSITION_Y_METERS);

  public static double getInitialFrontCableLength() {
    return INITIAL_CABLE_LENGTHS.frontLengthMeters;
  }

  public static double getInitialBackCableLength() {
    return INITIAL_CABLE_LENGTHS.backLengthMeters;
  }

  /** Cable length result (absolute lengths in meters) */
  public static class CableLengths {
    public final double frontLengthMeters;
    public final double backLengthMeters;
    public final double jointX;
    public final double jointY;

    public CableLengths(
        double frontLengthMeters, double backLengthMeters, double jointX, double jointY) {
      this.frontLengthMeters = frontLengthMeters;
      this.backLengthMeters = backLengthMeters;
      this.jointX = jointX;
      this.jointY = jointY;
    }
  }

  // ===========================================================================
  // Result Classes
  // ===========================================================================

  /** IK solution for one side (4 motors total, 2 per side) */
  public static class ClimbSideIKResult {
    public final double
        frontMotorRotations; // Cable 1 (mechanism/drum rotations, IO converts to motor)
    public final double
        backMotorRotations; // Cable 2 (mechanism/drum rotations, IO converts to motor)
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

  // ===========================================================================
  // Core IK Solver
  // ===========================================================================

  /**
   * Solve IK for one side of the climb mechanism.
   *
   * @param xe End effector X position (meters)
   * @param ye End effector Y position (meters)
   * @return IK solution with motor rotations
   */
  public static ClimbSideIKResult calculateIK(double ex, double ey) {
    // ─── Mechanism geometry from constants ───
    // Fixed points
    final double sx = ClimbConstants.SHOULDER_X_METERS; // Shoulder pivot
    final double sy = ClimbConstants.SHOULDER_Y_METERS;
    final double wfx = ClimbConstants.FRONT_WINCH_X_METERS; // Front winch
    final double wfy = ClimbConstants.FRONT_WINCH_Y_METERS;
    // W_back = (0, 0) — back winch at origin

    // Link lengths
    final double L1 = ClimbConstants.LINK_1_LENGTH_METERS; // Shoulder → Elbow
    final double L2 = ClimbConstants.LINK_2_LENGTH_METERS; // Elbow → End effector

    // Cable attachment offsets along the link (from moving end toward fixed end)
    final double backAttach = ClimbConstants.BACK_CABLE_ATTACH_ON_LINK1_METERS;
    final double frontAttach = ClimbConstants.FRONT_CABLE_ATTACH_ON_LINK2_METERS;

    final double eps = 1e-9;

    if (!isWithinWorkspace(ex, ey)) {
      return ClimbSideIKResult.invalid();
    }

    // ─── Two-Circle Intersection to find Elbow J ───
    // Circle 1: center = Shoulder S, radius = L1
    // Circle 2: center = End effector E, radius = L2
    final double dx = ex - sx;
    final double dy = ey - sy;
    final double dist = Math.hypot(dx, dy); // Distance S → E

    if (dist < eps) return ClimbSideIKResult.invalid();
    if (dist > L1 + L2 + 1e-12) return ClimbSideIKResult.invalid();
    if (dist < Math.abs(L1 - L2) - 1e-12) return ClimbSideIKResult.invalid();

    // d = distance from S along S→E to the intersection midpoint
    final double d = (L1 * L1 - L2 * L2 + dist * dist) / (2.0 * dist);

    double h2 = L1 * L1 - d * d;
    if (h2 < 0 && h2 > -1e-10) h2 = 0;
    if (h2 < 0) return ClimbSideIKResult.invalid();
    final double h = Math.sqrt(h2);

    // Unit vector from S to E
    final double ux = dx / dist;
    final double uy = dy / dist;

    // CCW perpendicular
    final double upx = -uy;
    final double upy = ux;

    // Joint J = S + d * u + h * u_perp (CCW / "left" solution)
    final double jx = sx + d * ux + h * upx;
    final double jy = sy + d * uy + h * upy;

    // ─── Cable Attachment Points ───

    // Point P on link 1: backAttach meters from J toward S
    final double t1 = backAttach / L1;
    final double px = jx + t1 * (sx - jx);
    final double py = jy + t1 * (sy - jy);

    // Point Q on link 2: frontAttach meters from E toward J
    final double t2 = frontAttach / L2;
    final double qx = ex + t2 * (jx - ex);
    final double qy = ey + t2 * (jy - ey);

    // ─── Cable Lengths ───
    final double backCableLen = Math.hypot(px, py); // P to W_back=(0,0)
    final double frontCableLen = Math.hypot(qx - wfx, qy - wfy); // Q to W_front

    // ─── Convert to Drum Rotations (relative to initial lengths) ───
    // Uses CableDrumModel to account for cable layer buildup on the drum
    final double frontDelta = frontCableLen - INITIAL_CABLE_LENGTHS.frontLengthMeters;
    final double backDelta = backCableLen - INITIAL_CABLE_LENGTHS.backLengthMeters;

    final double frontDrumRot = CableDrumModel.cableDeltaToRotations(frontDelta);
    final double backDrumRot = CableDrumModel.cableDeltaToRotations(backDelta);

    return new ClimbSideIKResult(
        frontDrumRot, // frontMotorRotations → front cable
        backDrumRot, // backMotorRotations  → back cable
        true,
        jx,
        jy);
  }

  // ===========================================================================
  // Wrapper Methods (for our workflow)
  // ===========================================================================

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

  // ===========================================================================
  // Validation & Limits
  // ===========================================================================

  /**
   * Check if a position is reachable (within workspace).
   *
   * @param position Position to check
   * @return true if reachable
   */
  public static boolean isPositionReachable(Translation2d position) {
    if (!isWithinWorkspace(position.getX(), position.getY())) {
      return false;
    }
    ClimbSideIKResult result = calculateIK(position);
    return result.isValid;
  }

  private static boolean isWithinWorkspace(double x, double y) {
    return x >= ClimbConstants.WORKSPACE_MIN_X_METERS
        && x <= ClimbConstants.WORKSPACE_MAX_X_METERS
        && y >= ClimbConstants.WORKSPACE_MIN_Y_METERS
        && y <= ClimbConstants.WORKSPACE_MAX_Y_METERS;
  }

  /**
   * Core cable length geometry calculation. Private so it can be used during static initialization
   * (before the class is fully loaded) and by the public API.
   *
   * <p>Returns front cable length and back cable length for a given end effector (ex, ey).
   */
  private static CableLengths calculateCableLengthsInternal(double ex, double ey) {
    final double sx = ClimbConstants.SHOULDER_X_METERS;
    final double sy = ClimbConstants.SHOULDER_Y_METERS;
    final double wfx = ClimbConstants.FRONT_WINCH_X_METERS;
    final double wfy = ClimbConstants.FRONT_WINCH_Y_METERS;
    final double L1 = ClimbConstants.LINK_1_LENGTH_METERS;
    final double L2 = ClimbConstants.LINK_2_LENGTH_METERS;
    final double backAttach = ClimbConstants.BACK_CABLE_ATTACH_ON_LINK1_METERS;
    final double frontAttach = ClimbConstants.FRONT_CABLE_ATTACH_ON_LINK2_METERS;

    // Two-circle intersection: S(radius L1) ∩ E(radius L2)
    final double dx = ex - sx;
    final double dy = ey - sy;
    final double dist = Math.hypot(dx, dy);
    if (dist <= 1e-9 || dist > L1 + L2 || dist < Math.abs(L1 - L2)) {
      return null;
    }

    final double d = (L1 * L1 - L2 * L2 + dist * dist) / (2.0 * dist);
    double h2 = L1 * L1 - d * d;
    if (h2 < 0 && h2 > -1e-10) h2 = 0;
    if (h2 < 0) return null;
    final double h = Math.sqrt(h2);

    final double ux = dx / dist;
    final double uy = dy / dist;
    final double upx = -uy;
    final double upy = ux;

    final double jx = sx + d * ux + h * upx;
    final double jy = sy + d * uy + h * upy;

    // Cable attachment points
    final double t1 = backAttach / L1;
    final double px = jx + t1 * (sx - jx);
    final double py = jy + t1 * (sy - jy);
    final double t2 = frontAttach / L2;
    final double qx = ex + t2 * (jx - ex);
    final double qy = ey + t2 * (jy - ey);

    // Cable lengths
    final double backCableLen = Math.hypot(px, py); // P to W_back=(0,0)
    final double frontCableLen = Math.hypot(qx - wfx, qy - wfy); // Q to W_front
    return new CableLengths(frontCableLen, backCableLen, jx, jy);
  }

  /** Calculate absolute cable lengths (meters) for a given end effector position. */
  public static CableLengths calculateCableLengths(double xe, double ye) {
    return calculateCableLengthsInternal(xe, ye);
  }

  /** Convert mechanism rotations to absolute cable lengths (meters). */
  public static CableLengths lengthsFromRotations(double frontRotations, double backRotations) {
    double l1 =
        CableDrumModel.rotationsToCableDelta(frontRotations)
            + INITIAL_CABLE_LENGTHS.frontLengthMeters;
    double l2 =
        CableDrumModel.rotationsToCableDelta(backRotations)
            + INITIAL_CABLE_LENGTHS.backLengthMeters;
    return new CableLengths(l1, l2, 0.0, 0.0);
  }

  /**
   * Forward kinematics using numeric solve (Newton) to estimate end effector from cable lengths.
   */
  public static Translation2d estimateEndEffectorPosition(
      double frontRotations, double backRotations, Translation2d initialGuess) {
    CableLengths target = lengthsFromRotations(frontRotations, backRotations);
    double x = initialGuess.getX();
    double y = initialGuess.getY();

    for (int i = 0; i < 15; i++) {
      CableLengths current = calculateCableLengths(x, y);
      if (current == null) {
        return null;
      }

      double e1 = current.frontLengthMeters - target.frontLengthMeters;
      double e2 = current.backLengthMeters - target.backLengthMeters;
      double err = Math.hypot(e1, e2);
      if (err < ClimbConstants.IK_POSITION_TOLERANCE_METERS) {
        return new Translation2d(x, y);
      }

      double h = 1e-4;
      CableLengths fx = calculateCableLengths(x + h, y);
      CableLengths fy = calculateCableLengths(x, y + h);
      if (fx == null || fy == null) {
        return null;
      }

      double dL1dx = (fx.frontLengthMeters - current.frontLengthMeters) / h;
      double dL2dx = (fx.backLengthMeters - current.backLengthMeters) / h;
      double dL1dy = (fy.frontLengthMeters - current.frontLengthMeters) / h;
      double dL2dy = (fy.backLengthMeters - current.backLengthMeters) / h;

      double det = dL1dx * dL2dy - dL1dy * dL2dx;
      if (Math.abs(det) < 1e-9) {
        return null;
      }

      double dx = (-e1 * dL2dy + e2 * dL1dy) / det;
      double dy = (-dL1dx * e2 + dL2dx * e1) / det;

      x += dx;
      y += dy;
    }

    return null;
  }

  // ===========================================================================
  // Numerical Jacobian for Velocity Control
  // ===========================================================================

  /** Result containing motor velocities for velocity control */
  public static class ClimbSideVelocityResult {
    public final double frontMotorVelocity; // Front motor velocity (mechanism rot/s, IO converts)
    public final double backMotorVelocity; // Back motor velocity (mechanism rot/s, IO converts)
    public final boolean isValid;

    public ClimbSideVelocityResult(
        double frontMotorVelocity, double backMotorVelocity, boolean isValid) {
      this.frontMotorVelocity = frontMotorVelocity;
      this.backMotorVelocity = backMotorVelocity;
      this.isValid = isValid;
    }

    public static ClimbSideVelocityResult invalid() {
      return new ClimbSideVelocityResult(0.0, 0.0, false);
    }
  }

  /** Complete velocity solution for both sides */
  public static class ClimbVelocityResult {
    public final ClimbSideVelocityResult leftSide;
    public final ClimbSideVelocityResult rightSide;

    public ClimbVelocityResult(
        ClimbSideVelocityResult leftSide, ClimbSideVelocityResult rightSide) {
      this.leftSide = leftSide;
      this.rightSide = rightSide;
    }

    public boolean isValid() {
      return leftSide.isValid && rightSide.isValid;
    }
  }

  /**
   * Calculate motor velocities from end effector velocity using numerical Jacobian.
   *
   * <p>Uses finite difference method: J[i,j] = ∂(motor_i) / ∂(position_j)
   *
   * <p>Then: motor_velocity = J * end_effector_velocity
   *
   * @param position Current end effector position (m)
   * @param velocity Desired end effector velocity (m/s)
   * @return Motor velocities (rot/s)
   */
  public static ClimbSideVelocityResult calculateVelocityIK(
      Translation2d position, Translation2d velocity) {
    // Finite difference step size (1mm in each direction)
    final double h = 0.001;

    // Get current motor positions
    ClimbSideIKResult current = calculateIK(position);
    if (!current.isValid) {
      return ClimbSideVelocityResult.invalid();
    }

    // Calculate partial derivatives using central differences
    ClimbSideIKResult posXPlus = calculateIK(position.getX() + h, position.getY());
    ClimbSideIKResult posXMinus = calculateIK(position.getX() - h, position.getY());
    if (!posXPlus.isValid || !posXMinus.isValid) {
      return ClimbSideVelocityResult.invalid();
    }
    double dFront_dx = (posXPlus.frontMotorRotations - posXMinus.frontMotorRotations) / (2.0 * h);
    double dBack_dx = (posXPlus.backMotorRotations - posXMinus.backMotorRotations) / (2.0 * h);

    ClimbSideIKResult posYPlus = calculateIK(position.getX(), position.getY() + h);
    ClimbSideIKResult posYMinus = calculateIK(position.getX(), position.getY() - h);
    if (!posYPlus.isValid || !posYMinus.isValid) {
      return ClimbSideVelocityResult.invalid();
    }
    double dFront_dy = (posYPlus.frontMotorRotations - posYMinus.frontMotorRotations) / (2.0 * h);
    double dBack_dy = (posYPlus.backMotorRotations - posYMinus.backMotorRotations) / (2.0 * h);

    // Apply chain rule: dθ/dt = (∂θ/∂x)(dx/dt) + (∂θ/∂y)(dy/dt)
    double frontMotorVel = dFront_dx * velocity.getX() + dFront_dy * velocity.getY();
    double backMotorVel = dBack_dx * velocity.getX() + dBack_dy * velocity.getY();

    return new ClimbSideVelocityResult(frontMotorVel, backMotorVel, true);
  }

  /**
   * Calculate motor velocities for both sides from end effector velocities.
   *
   * @param leftPosition Left side current position (m)
   * @param rightPosition Right side current position (m)
   * @param leftVelocity Left side desired velocity (m/s)
   * @param rightVelocity Right side desired velocity (m/s)
   * @return Motor velocities for all 4 motors (rot/s)
   */
  public static ClimbVelocityResult calculateVelocityIKBothSides(
      Translation2d leftPosition,
      Translation2d rightPosition,
      Translation2d leftVelocity,
      Translation2d rightVelocity) {
    ClimbSideVelocityResult leftResult = calculateVelocityIK(leftPosition, leftVelocity);
    ClimbSideVelocityResult rightResult = calculateVelocityIK(rightPosition, rightVelocity);
    return new ClimbVelocityResult(leftResult, rightResult);
  }

  /**
   * Calculate gravity compensation feedforward for each motor using J^T * F_gravity.
   *
   * <p>When pulling (robot hanging), gravity acts downward on the end effector as F = (0, -weight).
   * The required motor feedforward voltages are proportional to J^T * F_gravity. This ensures each
   * motor contributes the correct amount of torque based on the current mechanism geometry.
   *
   * @param position Current end effector position (m)
   * @param gravityVoltage Voltage scale factor representing gravitational load (V)
   * @return Motor feedforward voltages [frontFF, backFF], or [0,0] if invalid
   */
  public static double[] calculateGravityFeedforward(
      Translation2d position, double gravityVoltage) {
    final double h = 0.001;

    ClimbSideIKResult posYPlus = calculateIK(position.getX(), position.getY() + h);
    ClimbSideIKResult posYMinus = calculateIK(position.getX(), position.getY() - h);
    if (!posYPlus.isValid || !posYMinus.isValid) {
      return new double[] {0.0, 0.0};
    }

    // J^T * F_gravity = J^T * (0, -weight)
    // Only the Y column of the Jacobian matters since Fx=0
    double dFront_dy = (posYPlus.frontMotorRotations - posYMinus.frontMotorRotations) / (2.0 * h);
    double dBack_dy = (posYPlus.backMotorRotations - posYMinus.backMotorRotations) / (2.0 * h);

    // Gravity pulls down (-Y), cables must resist (+Y), so feedforward opposes gravity
    // tau = J^T * F -> voltage ~ -dtheta/dy * (-weight) = dtheta/dy * weight
    double frontFF = dFront_dy * gravityVoltage;
    double backFF = dBack_dy * gravityVoltage;

    return new double[] {frontFF, backFF};
  }
}
