package frc.robot.util;

import org.littletonrobotics.junction.Logger;

/**
 * Gravity feedforward calculator for a rack-and-pinion actuated planar rocker linkage.
 *
 * <p>Mechanism topology: motor, pinion, rack (linear), short coupler link, V-link elbow, pivoted
 * V-link (carries intake). Single-DOF prismatic-to-revolute transmission with nonlinear kinematics
 * and configuration-dependent mechanical advantage.
 *
 * <h3>What this computes:</h3>
 *
 * <ol>
 *   <li>Given rack position s (meters), solve rocker/V-link angle alpha (rad) from the
 *       constant-length coupler constraint via Newton's method.
 *   <li>Compute d(alpha)/ds from the differentiated constraint (virtual-work Jacobian).
 *   <li>Compute required rack force to hold gravity: Fg = dU/ds = (dU/d_alpha)(d_alpha/ds).
 *   <li>Convert to pinion torque: T = rp * Fg.
 *   <li>Convert to motor voltage via gear ratio and motor constants: V = (T / GR / Kt) * R.
 * </ol>
 *
 * <h3>Assumptions:</h3>
 *
 * <ul>
 *   <li>Planar mechanism, all joints in one plane.
 *   <li>Pure gravity compensation, ignores friction and inertia (add kS for friction).
 *   <li>Lumped mass model, single COM on the V-link.
 *   <li>Sign conventions must be verified on the real mechanism.
 * </ul>
 */
public final class IntakePivotFF {

  /** Mechanism configuration and physical parameters. All units: meters, radians, kg, SI. */
  public static final class Params {
    // --- Gravity ---
    /** Standard gravitational acceleration (m/s^2). */
    public double g = 9.80665;

    // --- Rack / Pinion ---
    /** Pinion pitch radius (meters). */
    public double pinionRadius_m;

    /**
     * Motor-to-mechanism gear ratio for the rack axis. This is the ratio from motor rotations to
     * pinion rotations. e.g. if motor spins 1 rotation and pinion spins 1 rotation, gearRatio =
     * 1.0. If there is a 10:1 reduction, gearRatio = 10.0.
     */
    public double rackGearRatio = 1.0;

    /**
     * Rack axis angle theta measured from +x axis (radians). theta = 0 means rack moves along +x;
     * theta = pi/2 means rack moves along +y (vertical).
     */
    public double rackTheta_rad;

    /** Rack attachment point A0 position (meters) when rack displacement s = 0. */
    public double A0x_m;

    public double A0y_m;

    // --- V-link pivot + elbow geometry ---
    /** O: fixed pivot of the V-link (meters). */
    public double Ox_m;

    public double Oy_m;

    /** R: distance from pivot O to elbow point E (meters). */
    public double elbowRadius_m;

    /** L: short coupler link length between rack attachment A and elbow E (meters). */
    public double couplerLength_m;

    // --- Mass model ---
    /**
     * Total moving mass to compensate (kg). This is the lumped mass of everything that hangs from
     * the V-link pivot (intake assembly, roller, etc.).
     */
    public double mass_kg;

    /**
     * Distance from pivot O to the lumped center of mass (meters). If the COM lies on the same
     * radial ray as the elbow, comAngleOffset_rad = 0.
     */
    public double comRadius_m;

    /**
     * COM angle offset delta (radians) relative to the alpha direction. The COM y-coordinate is:
     * y_COM = Oy + comRadius * sin(alpha + delta).
     */
    public double comAngleOffset_rad;

    // --- Motor conversion (optional, for voltage FF) ---
    /**
     * Motor torque constant Kt (Nm per Amp). Examples: Kraken X60 ~ 0.0194, NEO ~ 0.025, Falcon 500
     * ~ 0.0182.
     */
    public double motorKt_NmPerA = 0.025;

    /** Motor winding resistance (Ohms). Examples: NEO ~ 0.114, Kraken X60 ~ 0.025. */
    public double motorR_ohm = 0.114;

    /**
     * Efficiency fudge factor (0..1). Accounts for friction losses in gears/bearings. Start at 0.85
     * and tune. 1.0 = lossless.
     */
    public double efficiency = 1.0;
  }

  private final Params p;

  // Newton solver tuning
  private final int maxNewtonIters;
  private final double newtonTol;

  // Branch-tracking: store last alpha solution for continuity across cycles
  private double lastAlpha_rad = 0.0;
  private boolean hasLastAlpha = false;

  // Cached unit vector along rack axis (computed once)
  private final double rackUx;
  private final double rackUy;

  // Conversion factor: motor rotations to rack displacement (meters)
  // s = motorRotations * (2*pi / rackGearRatio) * pinionRadius
  private final double rotationsToMeters;

  /**
   * Constructs the FF calculator with default Newton solver settings (25 iterations, 1e-9
   * tolerance).
   */
  public IntakePivotFF(Params params) {
    this(params, 25, 1e-9);
  }

  /** Constructs the FF calculator with custom Newton solver settings. */
  public IntakePivotFF(Params params, int maxNewtonIters, double newtonTol) {
    this.p = params;
    this.maxNewtonIters = maxNewtonIters;
    this.newtonTol = newtonTol;

    // Pre-compute rack direction unit vector
    this.rackUx = Math.cos(p.rackTheta_rad);
    this.rackUy = Math.sin(p.rackTheta_rad);

    // Pre-compute rotations to meters conversion
    this.rotationsToMeters = (2.0 * Math.PI / p.rackGearRatio) * p.pinionRadius_m;
  }

  // ===================================================================
  //  Public API
  // ===================================================================

  /**
   * Compute gravity-compensating motor voltage given the current motor position in rotations.
   *
   * <p>This is the main method to call from the IO layer. Feed the result into
   * MotionMagicVoltage.withFeedForward().
   *
   * @param motorPositionRotations current motor encoder reading (rotations, same units as
   *     IntakePivotConstants setpoints)
   * @return voltage (V) to counteract gravity at this position
   */
  public double calculateVoltageFF(double motorPositionRotations) {
    double rackS_m = motorRotationsToRackS(motorPositionRotations);
    double volts = motorVoltageFF_V(rackS_m);

    // Log for tuning
    Logger.recordOutput("IntakePivotFF/rackS_m", rackS_m);
    Logger.recordOutput("IntakePivotFF/voltageFF", volts);
    Logger.recordOutput("IntakePivotFF/alpha_rad", lastAlpha_rad);

    return volts;
  }

  /**
   * Compute gravity-compensating pinion torque (Nm) given motor position in rotations.
   *
   * @param motorPositionRotations current motor encoder reading (rotations)
   * @return pinion torque (Nm) needed to hold gravity
   */
  public double calculatePinionTorqueFF(double motorPositionRotations) {
    double rackS_m = motorRotationsToRackS(motorPositionRotations);
    return pinionTorqueFF_Nm(rackS_m);
  }

  // ===================================================================
  //  Core kinematics (all in SI: meters, radians)
  // ===================================================================

  /**
   * Convert motor rotations to rack linear displacement (meters).
   *
   * <p>s = motorRotations * (2*pi / gearRatio) * pinionRadius
   */
  public double motorRotationsToRackS(double motorRotations) {
    return motorRotations * rotationsToMeters;
  }

  /**
   * Compute rack force along rack axis (N) needed to hold gravity at rack position s.
   *
   * <p>F = dU/ds = (dU/d_alpha)(d_alpha/ds)
   */
  public double rackForceFF_N(double rackS_m) {
    // 1) Solve alpha(s) from coupler constraint
    double alpha = solveAlphaFromS(rackS_m);

    // 2) Compute d_alpha/ds from differentiated constraint
    double dalpha_ds = dAlpha_dS(rackS_m, alpha);

    // 3) Gravitational torque about pivot O:
    //    U(alpha) = m*g*y_COM = m*g*(Oy + r*sin(alpha + delta))
    //    dU/d_alpha = m*g*r*cos(alpha + delta)
    double dU_dalpha = p.mass_kg * p.g * p.comRadius_m * Math.cos(alpha + p.comAngleOffset_rad);

    // 4) Rack force via chain rule: F = (dU/d_alpha)(d_alpha/ds)
    double Fg = dU_dalpha * dalpha_ds;

    // Apply efficiency (slight over-drive to overcome friction)
    if (p.efficiency > 0.0 && p.efficiency < 1.0) {
      Fg /= p.efficiency;
    }

    return Fg;
  }

  /**
   * Compute pinion torque (Nm) to hold gravity at rack position s.
   *
   * <p>T_pinion = r_pinion * F_rack
   */
  public double pinionTorqueFF_Nm(double rackS_m) {
    return p.pinionRadius_m * rackForceFF_N(rackS_m);
  }

  /**
   * Compute motor voltage (V) to statically hold gravity at rack position s.
   *
   * <p>T_motor = T_pinion / gearRatio, I = T_motor / Kt, V = I * R
   *
   * <p>This is a static DC motor approximation. With FOC/TorqueCurrentFOC, output I directly.
   */
  public double motorVoltageFF_V(double rackS_m) {
    double Tpinion = pinionTorqueFF_Nm(rackS_m);
    double Tmotor = Tpinion / p.rackGearRatio;
    double currentA = Tmotor / p.motorKt_NmPerA;
    return currentA * p.motorR_ohm;
  }

  // ===================================================================
  //  Constraint solver internals
  // ===================================================================

  /**
   * Solve for alpha given rack displacement s using Newton's method.
   *
   * <p>Constraint: f(alpha) = ||A(s) - E(alpha)||^2 - L^2 = 0
   *
   * <p>Uses branch-tracking (lastAlpha) for continuity. If the mechanism jumps discontinuously,
   * call {@link #resetAlphaGuess(double)} first.
   */
  private double solveAlphaFromS(double s) {
    // Rack attachment point: A(s) = A0 + s * u_hat
    double Ax = p.A0x_m + s * rackUx;
    double Ay = p.A0y_m + s * rackUy;

    double L2 = p.couplerLength_m * p.couplerLength_m;

    // Initial guess: use last solution for continuity
    double alpha = hasLastAlpha ? lastAlpha_rad : estimateInitialAlpha(Ax, Ay);

    for (int i = 0; i < maxNewtonIters; i++) {
      // Elbow point: E(alpha) = O + R * [cos(alpha), sin(alpha)]
      double cosA = Math.cos(alpha);
      double sinA = Math.sin(alpha);
      double Ex = p.Ox_m + p.elbowRadius_m * cosA;
      double Ey = p.Oy_m + p.elbowRadius_m * sinA;

      // Vector v = A - E
      double vx = Ax - Ex;
      double vy = Ay - Ey;

      // f(alpha) = v.v - L^2
      double f = (vx * vx + vy * vy) - L2;

      if (Math.abs(f) < newtonTol) {
        break;
      }

      // df/d_alpha = 2 * v . dv/d_alpha
      // dv/d_alpha = -dE/d_alpha = -R*[-sin(a), cos(a)] = R*[sin(a), -cos(a)]
      double dvx = p.elbowRadius_m * sinA;
      double dvy = -p.elbowRadius_m * cosA;
      double df = 2.0 * (vx * dvx + vy * dvy);

      // Guard against singularities (toggle / dead-center positions)
      if (Math.abs(df) < 1e-12) {
        break;
      }

      alpha -= f / df;
    }

    lastAlpha_rad = alpha;
    hasLastAlpha = true;
    return alpha;
  }

  /**
   * Compute d_alpha/ds from the differentiated constraint.
   *
   * <p>Differentiating v.v = L^2 w.r.t. s: 2*v . (dA/ds - dE/d_alpha * d_alpha/ds) = 0
   *
   * <p>Solving: d_alpha/ds = (v . u_hat) / (v . dE/d_alpha)
   */
  private double dAlpha_dS(double s, double alpha) {
    // A(s)
    double Ax = p.A0x_m + s * rackUx;
    double Ay = p.A0y_m + s * rackUy;

    // E(alpha)
    double cosA = Math.cos(alpha);
    double sinA = Math.sin(alpha);
    double Ex = p.Ox_m + p.elbowRadius_m * cosA;
    double Ey = p.Oy_m + p.elbowRadius_m * sinA;

    // v = A - E
    double vx = Ax - Ex;
    double vy = Ay - Ey;

    // Numerator: v . u_hat (rack direction)
    double num = vx * rackUx + vy * rackUy;

    // Denominator: v . dE/d_alpha, where dE/d_alpha = R*[-sin(a), cos(a)]
    double dEx = -p.elbowRadius_m * sinA;
    double dEy = p.elbowRadius_m * cosA;
    double den = vx * dEx + vy * dEy;

    // Near-zero denominator = singular config (mechanical advantage is infinite)
    if (Math.abs(den) < 1e-12) {
      return 0.0;
    }

    return num / den;
  }

  /**
   * Estimate a reasonable initial alpha when no previous solution exists. Uses atan2 from pivot O
   * to rack attachment A as a starting point.
   */
  private double estimateInitialAlpha(double Ax, double Ay) {
    return Math.atan2(Ay - p.Oy_m, Ax - p.Ox_m);
  }

  // ===================================================================
  //  State management
  // ===================================================================

  /** Reset the internal alpha guess. Call this if the mechanism jumps discontinuously. */
  public void resetAlphaGuess(double alpha_rad) {
    this.lastAlpha_rad = alpha_rad;
    this.hasLastAlpha = true;
  }

  /** Clear state so next solve starts from a geometric estimate. */
  public void clearState() {
    this.hasLastAlpha = false;
    this.lastAlpha_rad = 0.0;
  }

  /** Get the last solved alpha (radians). Useful for logging/debugging. */
  public double getLastAlpha() {
    return lastAlpha_rad;
  }

  // ===================================================================
  //  Mechanism2d geometry query
  // ===================================================================

  /** Snapshot of all linkage joint positions for a given configuration. */
  public static final class LinkageState {
    /** Rack attachment point A (meters). */
    public final double Ax, Ay;

    /** Elbow point E on the V-link (meters). */
    public final double Ex, Ey;

    /** Fixed pivot O (meters). */
    public final double Ox, Oy;

    /** Center of mass location (meters). */
    public final double COMx, COMy;

    /** Solved V-link angle alpha (radians). */
    public final double alpha_rad;

    /** Rack displacement s (meters). */
    public final double rackS_m;

    public LinkageState(
        double Ax,
        double Ay,
        double Ex,
        double Ey,
        double Ox,
        double Oy,
        double COMx,
        double COMy,
        double alpha_rad,
        double rackS_m) {
      this.Ax = Ax;
      this.Ay = Ay;
      this.Ex = Ex;
      this.Ey = Ey;
      this.Ox = Ox;
      this.Oy = Oy;
      this.COMx = COMx;
      this.COMy = COMy;
      this.alpha_rad = alpha_rad;
      this.rackS_m = rackS_m;
    }
  }

  /**
   * Compute all joint positions for the current motor position. Use this for Mechanism2d
   * visualization.
   *
   * @param motorPositionRotations current motor encoder reading (rotations)
   * @return a LinkageState with all joint coordinates in meters
   */
  public LinkageState computeLinkageState(double motorPositionRotations) {
    double s = motorRotationsToRackS(motorPositionRotations);
    double alpha = solveAlphaFromS(s);

    double Ax = p.A0x_m + s * rackUx;
    double Ay = p.A0y_m + s * rackUy;

    double cosA = Math.cos(alpha);
    double sinA = Math.sin(alpha);
    double Ex = p.Ox_m + p.elbowRadius_m * cosA;
    double Ey = p.Oy_m + p.elbowRadius_m * sinA;

    double cosAD = Math.cos(alpha + p.comAngleOffset_rad);
    double sinAD = Math.sin(alpha + p.comAngleOffset_rad);
    double COMx = p.Ox_m + p.comRadius_m * cosAD;
    double COMy = p.Oy_m + p.comRadius_m * sinAD;

    return new LinkageState(Ax, Ay, Ex, Ey, p.Ox_m, p.Oy_m, COMx, COMy, alpha, s);
  }

  /** Get the Params this FF was constructed with. Useful for Mechanism2d sizing. */
  public Params getParams() {
    return p;
  }
}
