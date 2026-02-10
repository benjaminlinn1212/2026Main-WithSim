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
  public static ClimbSideIKResult calculateIK(double xe, double ye) {
    // Mechanism parameters from constants
    final double a = ClimbConstants.WINCH_SEPARATION_METERS; // Distance between winches
    final double b = ClimbConstants.LINK_1_LENGTH_METERS; // Shoulder link
    final double c = ClimbConstants.LINK_2_LENGTH_METERS; // Forearm link
    final double p = ClimbConstants.CABLE_1_OFFSET_METERS; // Cable 1 attachment offset
    final double q = ClimbConstants.CABLE_2_OFFSET_METERS; // Cable 2 attachment offset
    final double eps = 1e-9;

    if (!isWithinWorkspace(xe, ye)) {
      return ClimbSideIKResult.invalid();
    }

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

    // Joint angle limits disabled for testing reachability

    // ─── Convert Cable Lengths to Motor Rotations ───
    // Step 1: Cable length to drum rotations
    // rotations = length / circumference
    final double l1Delta = l1 - ClimbConstants.FRONT_CABLE_INITIAL_LENGTH_METERS;
    final double l2Delta = l2 - ClimbConstants.BACK_CABLE_INITIAL_LENGTH_METERS;

    final double drumRotations_front = l1Delta / ClimbConstants.CABLE_DRUM_CIRCUMFERENCE_METERS;
    final double drumRotations_back = l2Delta / ClimbConstants.CABLE_DRUM_CIRCUMFERENCE_METERS;

    // Step 2: Drum rotations = mechanism rotations (we return mechanism rotations)
    // Negative rotations are allowed (cable shorter than initial length)
    // The IO layer (ClimbIOTalonFX) handles conversion to motor rotations
    // since SensorToMechanismRatio is now 1.0 per CTRE recommendation
    final double frontMotorRotations = drumRotations_front;
    final double backMotorRotations = drumRotations_back;

    return new ClimbSideIKResult(
        frontMotorRotations,
        backMotorRotations,
        true, // Valid solution
        xj,
        yj);
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

  /** Calculate absolute cable lengths (meters) for a given end effector position. */
  public static CableLengths calculateCableLengths(double xe, double ye) {
    final double a = ClimbConstants.WINCH_SEPARATION_METERS;
    final double b = ClimbConstants.LINK_1_LENGTH_METERS;
    final double c = ClimbConstants.LINK_2_LENGTH_METERS;
    final double p = ClimbConstants.CABLE_1_OFFSET_METERS;
    final double q = ClimbConstants.CABLE_2_OFFSET_METERS;

    final double dx = xe - a;
    final double dy = ye;
    final double r = Math.hypot(dx, dy);
    if (r <= 1e-9 || r > b + c || r < Math.abs(b - c)) {
      return null;
    }

    final double d = (b * b - c * c + r * r) / (2.0 * r);
    double h2 = b * b - d * d;
    if (h2 < 0 && h2 > -1e-10) h2 = 0;
    if (h2 < 0) return null;
    final double h = Math.sqrt(h2);

    final double ux = dx / r;
    final double uy = dy / r;
    final double upx = -uy;
    final double upy = ux;

    final double xj = a + d * ux + h * upx;
    final double yj = d * uy + h * upy;

    final double t1 = p / b;
    final double px = xj + t1 * (a - xj);
    final double py = yj + t1 * (0 - yj);
    final double t2 = q / c;
    final double qx = xe + t2 * (xj - xe);
    final double qy = ye + t2 * (yj - ye);

    final double l1 = Math.hypot(px, py);
    final double l2 = Math.hypot(qx - a, qy);
    return new CableLengths(l1, l2, xj, yj);
  }

  /** Convert mechanism rotations to absolute cable lengths (meters). */
  public static CableLengths lengthsFromRotations(double frontRotations, double backRotations) {
    double l1 =
        frontRotations * ClimbConstants.CABLE_DRUM_CIRCUMFERENCE_METERS
            + ClimbConstants.FRONT_CABLE_INITIAL_LENGTH_METERS;
    double l2 =
        backRotations * ClimbConstants.CABLE_DRUM_CIRCUMFERENCE_METERS
            + ClimbConstants.BACK_CABLE_INITIAL_LENGTH_METERS;
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
