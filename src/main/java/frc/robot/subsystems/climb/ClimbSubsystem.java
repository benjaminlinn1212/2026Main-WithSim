package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.util.ClimbIK;
import frc.robot.subsystems.climb.util.ClimbIK.ClimbIKResult;
import frc.robot.subsystems.climb.util.ClimbPathPlanner;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * Climb subsystem: ClimbState waypoints ClimbIK math TalonFX motors
 *
 * <p>CONTROL STRATEGY (Hybrid Position + Velocity): - VELOCITY CONTROL: Used during path following
 * for smooth dynamic motion - Numerical Jacobian converts end effector velocity → motor velocity -
 * Similar to drivetrain: chassis speeds → wheel speeds - Enables accurate trajectory tracking with
 * feedforward
 *
 * <p>- POSITION CONTROL: Used for holding static positions - Motion Magic to final positions after
 * path completes - Used for MANUAL and EMERGENCY_STOP states - Provides stable position hold at
 * target locations
 */
public class ClimbSubsystem extends SubsystemBase {

  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private ClimbState currentState = ClimbState.STOWED;
  private Translation2d leftTargetPosition = ClimbState.STOWED.getTargetPosition();
  private Translation2d rightTargetPosition = ClimbState.STOWED.getTargetPosition();

  // ─── Calibration mode ───
  private boolean calibrationMode = false;

  // ─── Mechanism2d visualization ───
  // Canvas origin is the back winch (0,0). Canvas sized to contain workspace.
  // In AdvantageScope 3D, the robot origin maps to canvas (CANVAS_W/2, 0).
  // To place climb (0,0) at drivetrain-relative (x=-0.1, z=0.05):
  //   CANVAS_OFFSET_X = CANVAS_W/2 + (-0.1) = 0.6
  //   CANVAS_OFFSET_Y = 0.05
  private static final double CANVAS_W = 1.4;
  private static final double CANVAS_H = 1.4;
  private static final double CANVAS_OFFSET_X = CANVAS_W / 2.0 + (-0.1); // climb (0,0) at x=-0.1m
  private static final double CANVAS_OFFSET_Y = 0.05; // climb (0,0) at z=0.05m

  private final Mechanism2d mechanism;
  // Target: shoulder→link1→link2 (blue)
  private final MechanismLigament2d targetLink1;
  private final MechanismLigament2d targetLink2;
  // Measured: shoulder→link1→link2 (green)
  private final MechanismLigament2d measuredLink1;
  private final MechanismLigament2d measuredLink2;
  // Back cable (yellow, from backWinch to attachment P)
  private final MechanismLigament2d targetBackCable;
  // Front cable (orange, from frontWinch to attachment Q)
  private final MechanismLigament2d targetFrontCable;

  public ClimbSubsystem(ClimbIO io) {
    this.io = io;

    final double sx = ClimbConstants.SHOULDER_X_METERS;
    final double sy = ClimbConstants.SHOULDER_Y_METERS;
    final double wfx = ClimbConstants.FRONT_WINCH_X_METERS;
    final double wfy = ClimbConstants.FRONT_WINCH_Y_METERS;

    mechanism = new Mechanism2d(CANVAS_W, CANVAS_H);

    // ── Fixed-point markers ──
    // Back winch at origin
    MechanismRoot2d backWinchRoot =
        mechanism.getRoot("BackWinch", 0 + CANVAS_OFFSET_X, 0 + CANVAS_OFFSET_Y);
    backWinchRoot.append(
        new MechanismLigament2d("BackWinchDot", 0.02, 0, 8, new Color8Bit(Color.kWhite)));

    // Front winch
    MechanismRoot2d frontWinchRoot =
        mechanism.getRoot("FrontWinch", wfx + CANVAS_OFFSET_X, wfy + CANVAS_OFFSET_Y);
    frontWinchRoot.append(
        new MechanismLigament2d("FrontWinchDot", 0.02, 0, 8, new Color8Bit(Color.kWhite)));

    // ── Target arm (blue) — rooted at shoulder ──
    MechanismRoot2d targetShoulderRoot =
        mechanism.getRoot("TargetShoulder", sx + CANVAS_OFFSET_X, sy + CANVAS_OFFSET_Y);
    targetLink1 =
        targetShoulderRoot.append(
            new MechanismLigament2d(
                "TargetLink1",
                ClimbConstants.LINK_1_LENGTH_METERS,
                90,
                5,
                new Color8Bit(Color.kDodgerBlue)));
    targetLink2 =
        targetLink1.append(
            new MechanismLigament2d(
                "TargetLink2",
                ClimbConstants.LINK_2_LENGTH_METERS,
                0,
                4,
                new Color8Bit(Color.kCornflowerBlue)));

    // ── Measured arm (green) — rooted at same shoulder ──
    MechanismRoot2d measuredShoulderRoot =
        mechanism.getRoot("MeasuredShoulder", sx + CANVAS_OFFSET_X, sy + CANVAS_OFFSET_Y);
    measuredLink1 =
        measuredShoulderRoot.append(
            new MechanismLigament2d(
                "MeasuredLink1",
                ClimbConstants.LINK_1_LENGTH_METERS,
                90,
                3,
                new Color8Bit(Color.kLimeGreen)));
    measuredLink2 =
        measuredLink1.append(
            new MechanismLigament2d(
                "MeasuredLink2",
                ClimbConstants.LINK_2_LENGTH_METERS,
                0,
                2,
                new Color8Bit(Color.kLime)));

    // ── Cable lines (from winch roots) ──
    targetBackCable =
        backWinchRoot.append(
            new MechanismLigament2d("BackCable", 0.3, 90, 1, new Color8Bit(Color.kYellow)));
    targetFrontCable =
        frontWinchRoot.append(
            new MechanismLigament2d("FrontCable", 0.1, 90, 1, new Color8Bit(Color.kOrange)));

    // ── Initialize Mechanism2d to the STOWED pose so it renders correctly from frame 0 ──
    initializeMechanism2dToStowed(sx, sy, wfx, wfy);

    // Publish once — SmartDashboard auto-updates from mutated ligaments each cycle
    SmartDashboard.putData("Climb/Mechanism2d", mechanism);

    // Initialize calibration mode state so triggers work from first cycle
    Logger.recordOutput("Climb/CalibrationMode", false);
    Logger.recordOutput("Climb/CurrentState", ClimbState.STOWED.getName());
  }

  /**
   * Set all Mechanism2d ligament angles/lengths to match the initial STOWED position so the
   * visualization is correct before the first periodic() call.
   */
  private void initializeMechanism2dToStowed(double sx, double sy, double wfx, double wfy) {
    final double L1 = ClimbConstants.LINK_1_LENGTH_METERS;
    final double L2 = ClimbConstants.LINK_2_LENGTH_METERS;
    final double backAttach = ClimbConstants.BACK_CABLE_ATTACH_ON_LINK1_METERS;
    final double frontAttach = ClimbConstants.FRONT_CABLE_ATTACH_ON_LINK2_METERS;

    Translation2d stowedPos = ClimbState.STOWED.getTargetPosition();
    ClimbIK.ClimbSideIKResult ik = ClimbIK.calculateIK(stowedPos);
    if (!ik.isValid) {
      return; // keep defaults if IK fails (shouldn't happen for STOWED)
    }

    double jx = ik.jointX;
    double jy = ik.jointY;
    double ex = stowedPos.getX();
    double ey = stowedPos.getY();

    // Link angles
    double link1Angle = Math.toDegrees(Math.atan2(jy - sy, jx - sx));
    double link2AbsAngle = Math.toDegrees(Math.atan2(ey - jy, ex - jx));

    targetLink1.setAngle(link1Angle);
    targetLink2.setAngle(link2AbsAngle - link1Angle);
    measuredLink1.setAngle(link1Angle);
    measuredLink2.setAngle(link2AbsAngle - link1Angle);

    // Back cable: from backWinch(0,0) to point P on link 1
    double t1 = backAttach / L1;
    double px = jx + t1 * (sx - jx);
    double py = jy + t1 * (sy - jy);
    targetBackCable.setLength(Math.hypot(px, py));
    targetBackCable.setAngle(Math.toDegrees(Math.atan2(py, px)));

    // Front cable: from frontWinch to point Q on link 2
    double t2 = frontAttach / L2;
    double qx = ex + t2 * (jx - ex);
    double qy = ey + t2 * (jy - ey);
    targetFrontCable.setLength(Math.hypot(qx - wfx, qy - wfy));
    targetFrontCable.setAngle(Math.toDegrees(Math.atan2(qy - wfy, qx - wfx)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    // State is logged by commands (setState, runPath)
    Logger.recordOutput("Climb/CalibrationMode", calibrationMode);
    Logger.recordOutput("Climb/LeftTargetPosition", leftTargetPosition);
    Logger.recordOutput("Climb/RightTargetPosition", rightTargetPosition);

    // Estimate and log actual end effector positions from motor encoders (FK)
    Translation2d measuredLeft =
        ClimbIK.estimateEndEffectorPosition(
            inputs.leftFrontPositionRotations,
            inputs.leftBackPositionRotations,
            leftTargetPosition);
    Translation2d measuredRight =
        ClimbIK.estimateEndEffectorPosition(
            inputs.rightFrontPositionRotations,
            inputs.rightBackPositionRotations,
            rightTargetPosition);
    if (measuredLeft != null) {
      Logger.recordOutput("Climb/LeftMeasuredPosition", measuredLeft);
    }
    if (measuredRight != null) {
      Logger.recordOutput("Climb/RightMeasuredPosition", measuredRight);
    }

    // Log motor rotations directly for debugging
    Logger.recordOutput("Climb/LeftFrontRotations", inputs.leftFrontPositionRotations);
    Logger.recordOutput("Climb/LeftBackRotations", inputs.leftBackPositionRotations);
    Logger.recordOutput("Climb/RightFrontRotations", inputs.rightFrontPositionRotations);
    Logger.recordOutput("Climb/RightBackRotations", inputs.rightBackPositionRotations);

    // ── Update Mechanism2d ──
    final double sx = ClimbConstants.SHOULDER_X_METERS;
    final double sy = ClimbConstants.SHOULDER_Y_METERS;
    final double wfx = ClimbConstants.FRONT_WINCH_X_METERS;
    final double wfy = ClimbConstants.FRONT_WINCH_Y_METERS;
    final double L1 = ClimbConstants.LINK_1_LENGTH_METERS;
    final double L2 = ClimbConstants.LINK_2_LENGTH_METERS;
    final double backAttach = ClimbConstants.BACK_CABLE_ATTACH_ON_LINK1_METERS;
    final double frontAttach = ClimbConstants.FRONT_CABLE_ATTACH_ON_LINK2_METERS;

    // --- Target arm (use left target for visualization — both sides symmetric) ---
    ClimbIK.ClimbSideIKResult targetIK = ClimbIK.calculateIK(leftTargetPosition);
    if (targetIK.isValid) {
      double jx = targetIK.jointX;
      double jy = targetIK.jointY;
      double ex = leftTargetPosition.getX();
      double ey = leftTargetPosition.getY();

      // Link 1 angle: shoulder → elbow (absolute, degrees from +X axis)
      double link1Angle = Math.toDegrees(Math.atan2(jy - sy, jx - sx));
      targetLink1.setAngle(link1Angle);

      // Link 2 angle: elbow → EE, relative to link 1 direction
      double link2AbsAngle = Math.toDegrees(Math.atan2(ey - jy, ex - jx));
      targetLink2.setAngle(link2AbsAngle - link1Angle);

      // Back cable: from backWinch(0,0) to point P on link 1
      double t1 = backAttach / L1;
      double px = jx + t1 * (sx - jx);
      double py = jy + t1 * (sy - jy);
      targetBackCable.setLength(Math.hypot(px, py));
      targetBackCable.setAngle(Math.toDegrees(Math.atan2(py, px)));

      // Front cable: from frontWinch to point Q on link 2
      double t2 = frontAttach / L2;
      double qx = ex + t2 * (jx - ex);
      double qy = ey + t2 * (jy - ey);
      targetFrontCable.setLength(Math.hypot(qx - wfx, qy - wfy));
      targetFrontCable.setAngle(Math.toDegrees(Math.atan2(qy - wfy, qx - wfx)));
    }

    // --- Measured arm (from FK estimate) ---
    if (measuredLeft != null) {
      ClimbIK.ClimbSideIKResult measIK = ClimbIK.calculateIK(measuredLeft);
      if (measIK.isValid) {
        double jx = measIK.jointX;
        double jy = measIK.jointY;
        double ex = measuredLeft.getX();
        double ey = measuredLeft.getY();

        double link1Angle = Math.toDegrees(Math.atan2(jy - sy, jx - sx));
        measuredLink1.setAngle(link1Angle);

        double link2AbsAngle = Math.toDegrees(Math.atan2(ey - jy, ex - jx));
        measuredLink2.setAngle(link2AbsAngle - link1Angle);
      }
    }

    // Mechanism2d is published via SmartDashboard.putData in constructor — auto-updates
  }

  // =============================================================================
  // STATE MANAGEMENT
  // =============================================================================

  /**
   * Reset the climb to STOWED — resets both the subsystem state and the IO-layer motor positions.
   * In SIM this teleports the simulated motors back to STOWED cable lengths; on real hardware this
   * is a no-op at the IO layer (the real encoders already reflect reality).
   */
  public void resetToStowed() {
    io.resetToStowed();
    setState(ClimbState.STOWED);
  }

  public void setState(ClimbState state) {
    this.currentState = state;
    this.leftTargetPosition = state.getTargetPosition();
    this.rightTargetPosition = state.getTargetPosition();
    moveToTargetPositions();
    Logger.recordOutput("Climb/CurrentState", state.getName());
  }

  public ClimbState getState() {
    return currentState;
  }

  /** Advance to the next climb state with path following. */
  public Command nextState() {
    return runPath(
        () -> {
          ClimbState next = currentState.getNextState();
          if (next == null || !next.hasPrePlannedPath()) {
            if (next != null) setState(next);
            return null;
          }
          currentState = next;
          Logger.recordOutput("Climb/CurrentState", next.getName());
          // Replace first waypoint with current position so path starts where the arm
          // actually is, avoiding jumps from tracking error or mid-path interruptions.
          List<Translation2d> waypoints = new java.util.ArrayList<>(next.getPrePlannedWaypoints());
          waypoints.set(0, leftTargetPosition);
          return new PathParams(waypoints, next.isPulling());
        },
        "ClimbNextState");
  }

  /** Go back to the previous climb state using a path from current position. */
  public Command previousState() {
    return runPath(
        () -> {
          ClimbState prev = currentState.getPreviousState();
          if (prev == null) {
            return null;
          }
          // Build a 2-point path from where the arm actually is to the previous state's
          // target position. Using the full reversed waypoint list is wrong when the
          // forward path was interrupted mid-way — the reversed path would start from
          // the end of the original path, not from the arm's current position.
          Translation2d start = leftTargetPosition;
          Translation2d end = prev.getTargetPosition();
          boolean isPulling = !currentState.isPulling();
          currentState = prev;
          Logger.recordOutput("Climb/CurrentState", prev.getName());
          return new PathParams(List.of(start, end), isPulling);
        },
        "ClimbPreviousState");
  }

  /** Transition to a specific state. Safe to chain in Commands.sequence(). */
  public Command setStateCommand(ClimbState state) {
    return runPath(
        () -> {
          currentState = state;
          Logger.recordOutput("Climb/CurrentState", state.getName());
          if (!state.hasPrePlannedPath()) {
            setState(state);
            return null;
          }
          // Replace first waypoint with current position so path starts where the arm
          // actually is, avoiding jumps from tracking error or mid-path interruptions.
          List<Translation2d> waypoints = new java.util.ArrayList<>(state.getPrePlannedWaypoints());
          waypoints.set(0, leftTargetPosition);
          return new PathParams(waypoints, state.isPulling());
        },
        "ClimbSetState_" + state.name());
  }

  /**
   * Auto-only L1 climb command. Sequences EXTEND_L1_AUTO then RETRACT_L1_AUTO. This is separate
   * from the teleop climb state cycle.
   */
  public Command autoClimbL1() {
    return Commands.sequence(
            setStateCommand(ClimbState.EXTEND_L1_AUTO), setStateCommand(ClimbState.RETRACT_L1_AUTO))
        .withName("AutoClimbL1");
  }

  /**
   * Release from auto L1 climb by reversing RETRACT_L1_AUTO. After auto ends with the robot latched
   * at RETRACT_L1_AUTO, this reverses the retract path back to the EXTEND_L1_AUTO position (arm
   * extended, released from bar). Use {@link #stowFromCurrentState()} (POV Down) afterward to stow
   * the arm back to STOWED.
   *
   * <p>Only takes effect when the current state is RETRACT_L1_AUTO. If the robot is in a different
   * state, this is a no-op.
   */
  public Command releaseFromAutoL1() {
    return runPath(
            () -> {
              if (currentState != ClimbState.RETRACT_L1_AUTO) return null;
              Translation2d start = leftTargetPosition;
              Translation2d end = ClimbState.EXTEND_L1_AUTO.getTargetPosition();
              // Reversing a pull = not pulling (going back up)
              boolean isPulling = !ClimbState.RETRACT_L1_AUTO.isPulling();
              currentState = ClimbState.EXTEND_L1_AUTO;
              Logger.recordOutput("Climb/CurrentState", currentState.getName());
              return new PathParams(List.of(start, end), isPulling);
            },
            "ReleaseAutoL1_ReverseRetract")
        .withName("ReleaseFromAutoL1");
  }

  /**
   * Stow the climb by following a single path directly from the current position to STOWED. Also
   * stows any servos that may have been released. If already at STOWED, this is a no-op.
   */
  public Command stowFromCurrentState() {
    return Commands.sequence(
            // Stow servos sequentially — hardstop first, wait, then angle, wait
            runOnce(this::stowHardstopServos),
            Commands.waitSeconds(ClimbConstants.HardstopServo.TRAVEL_TIME_SEC),
            runOnce(this::stowAngleServos),
            Commands.waitSeconds(ClimbConstants.AngleServo.TRAVEL_TIME_SEC),
            // Follow a path directly from current position to STOWED
            runPath(
                () -> {
                  if (currentState == ClimbState.STOWED) return null;
                  Translation2d start = leftTargetPosition;
                  Translation2d end = ClimbState.STOWED.getTargetPosition();
                  currentState = ClimbState.STOWED;
                  Logger.recordOutput("Climb/CurrentState", currentState.getName());
                  return new PathParams(List.of(start, end), false);
                },
                "StowDirect"))
        .withName("StowFromCurrentState");
  }

  // -- Helper: reusable path-following command --

  private record PathParams(List<Translation2d> waypoints, boolean isPulling) {}

  /**
   * Creates a command that calls the supplier on initialize to get path params, then follows the
   * path. If supplier returns null, finishes immediately (no path to run).
   */
  private Command runPath(java.util.function.Supplier<PathParams> paramSupplier, String name) {
    return new Command() {
      private ClimbPathPlanner.PathExecutor executor;
      private PathParams params;

      {
        addRequirements(ClimbSubsystem.this);
        setName(name);
      }

      @Override
      public void initialize() {
        executor = null;
        params = paramSupplier.get();
        if (params == null || params.waypoints().size() < 2) return;

        try {
          var path =
              ClimbPathPlanner.createMultiBezierPath(params.waypoints(), 0.15, params.isPulling());
          if (!ClimbPathPlanner.isPathValid(path)) return;
          executor = new ClimbPathPlanner.PathExecutor(path, path);
          executor.start();
        } catch (Exception e) {
          System.out.println("[Climb] Path generation failed: " + e.getMessage());
        }
      }

      @Override
      public void execute() {
        if (executor != null) {
          Translation2d[] targets = executor.getCurrentTargets();
          Translation2d[] velocities = executor.getCurrentVelocities();
          setTargetVelocitiesInternal(
              targets[0], targets[1], velocities[0], velocities[1], params.isPulling());
        }
      }

      @Override
      public boolean isFinished() {
        return executor == null || executor.isFinished();
      }

      @Override
      public void end(boolean interrupted) {
        if (executor != null) executor.stop();
        if (!interrupted && params != null && !params.waypoints().isEmpty()) {
          Translation2d finalPos = params.waypoints().get(params.waypoints().size() - 1);
          setTargetPositionsInternal(finalPos, finalPos);
        } else {
          moveToTargetPositions();
        }
      }
    };
  }

  // ===========================================================================
  // POSITION CONTROL (Motion Magic for static positions)
  // ===========================================================================

  private void moveToTargetPositions() {
    ClimbIKResult ikResult = ClimbIK.calculateBothSides(leftTargetPosition, rightTargetPosition);
    if (ikResult.isValid()) {
      io.setLeftFrontPosition(ikResult.leftSide.frontMotorRotations);
      io.setLeftBackPosition(ikResult.leftSide.backMotorRotations);
      io.setRightFrontPosition(ikResult.rightSide.frontMotorRotations);
      io.setRightBackPosition(ikResult.rightSide.backMotorRotations);
      Logger.recordOutput("Climb/IK/Valid", true);

      // Log joint positions for mechanism visualization
      Logger.recordOutput(
          "Climb/IK/LeftJoint", new double[] {ikResult.leftSide.jointX, ikResult.leftSide.jointY});
      Logger.recordOutput(
          "Climb/IK/RightJoint",
          new double[] {ikResult.rightSide.jointX, ikResult.rightSide.jointY});
    } else {
      Logger.recordOutput("Climb/IK/Valid", false);
    }
  }

  // Internal method for path following (doesn't change state to MANUAL)
  private void setTargetPositionsInternal(Translation2d leftPosition, Translation2d rightPosition) {
    this.leftTargetPosition = leftPosition;
    this.rightTargetPosition = rightPosition;
    moveToTargetPositions();
  }

  // ===========================================================================
  // VELOCITY CONTROL (Jacobian-based for path following)
  // ===========================================================================

  /**
   * Set motor velocities using numerical Jacobian transformation.
   *
   * <p>Flow: (x, y, xdot, ydot) → IK → (l1, l2) → Jacobian → (l1dot, l2dot) → motors
   *
   * @param leftPosition Current left end effector position (m)
   * @param rightPosition Current right end effector position (m)
   * @param leftVelocity Desired left end effector velocity (m/s)
   * @param rightVelocity Desired right end effector velocity (m/s)
   * @param isPulling Whether this is a pulling motion (adds extra feedforward)
   */
  private void setTargetVelocitiesInternal(
      Translation2d leftPosition,
      Translation2d rightPosition,
      Translation2d leftVelocity,
      Translation2d rightVelocity,
      boolean isPulling) {
    this.leftTargetPosition = leftPosition;
    this.rightTargetPosition = rightPosition;

    Translation2d measuredLeftPosition =
        ClimbIK.estimateEndEffectorPosition(
            inputs.leftFrontPositionRotations,
            inputs.leftBackPositionRotations,
            leftTargetPosition);
    Translation2d measuredRightPosition =
        ClimbIK.estimateEndEffectorPosition(
            inputs.rightFrontPositionRotations,
            inputs.rightBackPositionRotations,
            rightTargetPosition);

    if (measuredLeftPosition == null) {
      measuredLeftPosition = leftTargetPosition;
    }
    if (measuredRightPosition == null) {
      measuredRightPosition = rightTargetPosition;
    }

    // Use numerical Jacobian to calculate motor velocities from end effector velocities
    ClimbIK.ClimbVelocityResult velocityResult =
        ClimbIK.calculateVelocityIKBothSides(
            measuredLeftPosition, measuredRightPosition, leftVelocity, rightVelocity);

    if (!velocityResult.isValid()) {
      Logger.recordOutput("Climb/VelocityIK/Valid", false);
      // Fallback to position control if velocity IK fails
      moveToTargetPositions();
      return;
    }

    Logger.recordOutput("Climb/VelocityIK/Valid", true);

    double leftFrontVel =
        MathUtil.clamp(
            velocityResult.leftSide.frontMotorVelocity,
            -frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY,
            frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY);
    double leftBackVel =
        MathUtil.clamp(
            velocityResult.leftSide.backMotorVelocity,
            -frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY,
            frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY);
    double rightFrontVel =
        MathUtil.clamp(
            velocityResult.rightSide.frontMotorVelocity,
            -frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY,
            frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY);
    double rightBackVel =
        MathUtil.clamp(
            velocityResult.rightSide.backMotorVelocity,
            -frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY,
            frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY);

    // Apply gravity compensation feedforward using Jacobian transpose if pulling
    // J^T * F_gravity gives geometrically-correct torque distribution across motors
    double leftFrontFF = 0.0;
    double leftBackFF = 0.0;
    double rightFrontFF = 0.0;
    double rightBackFF = 0.0;
    if (isPulling) {
      double[] leftGravFF =
          ClimbIK.calculateGravityFeedforward(
              measuredLeftPosition, frc.robot.Constants.ClimbConstants.VELOCITY_KG_PULLING);
      double[] rightGravFF =
          ClimbIK.calculateGravityFeedforward(
              measuredRightPosition, frc.robot.Constants.ClimbConstants.VELOCITY_KG_PULLING);
      leftFrontFF = leftGravFF[0];
      leftBackFF = leftGravFF[1];
      rightFrontFF = rightGravFF[0];
      rightBackFF = rightGravFF[1];
    }

    // Send velocity commands to all 4 motors
    io.setLeftFrontVelocity(leftFrontVel, leftFrontFF);
    io.setLeftBackVelocity(leftBackVel, leftBackFF);
    io.setRightFrontVelocity(rightFrontVel, rightFrontFF);
    io.setRightBackVelocity(rightBackVel, rightBackFF);

    // Log velocities for debugging
    Logger.recordOutput("Climb/VelocityIK/LeftFrontVel", leftFrontVel);
    Logger.recordOutput("Climb/VelocityIK/LeftBackVel", leftBackVel);
    Logger.recordOutput("Climb/VelocityIK/RightFrontVel", rightFrontVel);
    Logger.recordOutput("Climb/VelocityIK/RightBackVel", rightBackVel);

    // Log end effector velocities
    Logger.recordOutput("Climb/VelocityIK/LeftVelX", leftVelocity.getX());
    Logger.recordOutput("Climb/VelocityIK/LeftVelY", leftVelocity.getY());
    Logger.recordOutput("Climb/VelocityIK/RightVelX", rightVelocity.getX());
    Logger.recordOutput("Climb/VelocityIK/RightVelY", rightVelocity.getY());
    Logger.recordOutput("Climb/VelocityIK/LeftMeasured", measuredLeftPosition);
    Logger.recordOutput("Climb/VelocityIK/RightMeasured", measuredRightPosition);

    // Still calculate IK for position logging
    ClimbIKResult ikResult = ClimbIK.calculateBothSides(leftPosition, rightPosition);
    if (ikResult.isValid()) {
      Logger.recordOutput("Climb/IK/Valid", true);
      Logger.recordOutput(
          "Climb/IK/LeftJoint", new double[] {ikResult.leftSide.jointX, ikResult.leftSide.jointY});
      Logger.recordOutput(
          "Climb/IK/RightJoint",
          new double[] {ikResult.rightSide.jointX, ikResult.rightSide.jointY});
    }
  }

  // ===========================================================================
  // GETTERS
  // ===========================================================================

  public Translation2d getLeftTargetPosition() {
    return leftTargetPosition;
  }

  public Translation2d getRightTargetPosition() {
    return rightTargetPosition;
  }

  // ===========================================================================
  // SECONDARY HOOK SERVOS
  // ===========================================================================

  // ─── Raw per-servo setters (0.0–1.0 logical position, inversion applied in IO) ───

  /** Set the right secondary hook angle servo position (0.0–1.0). */
  public void setRightSecondaryHookAngle(double position) {
    io.setRightSecondaryHookAnglePosition(position);
  }

  /** Set the right secondary hook hardstop servo position (0.0–1.0). */
  public void setRightSecondaryHookHardstop(double position) {
    io.setRightSecondaryHookHardstopPosition(position);
  }

  /** Set the left secondary hook angle servo position (0.0–1.0). */
  public void setLeftSecondaryHookAngle(double position) {
    io.setLeftSecondaryHookAnglePosition(position);
  }

  /** Set the left secondary hook hardstop servo position (0.0–1.0). */
  public void setLeftSecondaryHookHardstop(double position) {
    io.setLeftSecondaryHookHardstopPosition(position);
  }

  // ─── Stow (retract hooks) ───

  /** Stow all 4 secondary hook servos to their starting position. */
  public void stowAllServos() {
    io.setLeftSecondaryHookAnglePosition(ClimbConstants.AngleServo.STOWED_POSITION);
    io.setRightSecondaryHookAnglePosition(ClimbConstants.AngleServo.STOWED_POSITION);
    io.setLeftSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.STOWED_POSITION);
    io.setRightSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.STOWED_POSITION);
  }

  /** Stow the left side secondary hook servos (angle + hardstop). */
  public void stowLeftServos() {
    io.setLeftSecondaryHookAnglePosition(ClimbConstants.AngleServo.STOWED_POSITION);
    io.setLeftSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.STOWED_POSITION);
  }

  /** Stow the right side secondary hook servos (angle + hardstop). */
  public void stowRightServos() {
    io.setRightSecondaryHookAnglePosition(ClimbConstants.AngleServo.STOWED_POSITION);
    io.setRightSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.STOWED_POSITION);
  }

  // ─── Release (deploy hooks) ───

  /** Release all 4 secondary hook servos to their deployed position. */
  public void releaseAllServos() {
    io.setLeftSecondaryHookAnglePosition(ClimbConstants.AngleServo.RELEASED_POSITION);
    io.setRightSecondaryHookAnglePosition(ClimbConstants.AngleServo.RELEASED_POSITION);
    io.setLeftSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.RELEASED_POSITION);
    io.setRightSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.RELEASED_POSITION);
  }

  /** Release the left side secondary hook servos (angle + hardstop). */
  public void releaseLeftServos() {
    io.setLeftSecondaryHookAnglePosition(ClimbConstants.AngleServo.RELEASED_POSITION);
    io.setLeftSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.RELEASED_POSITION);
  }

  /** Release the right side secondary hook servos (angle + hardstop). */
  public void releaseRightServos() {
    io.setRightSecondaryHookAnglePosition(ClimbConstants.AngleServo.RELEASED_POSITION);
    io.setRightSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.RELEASED_POSITION);
  }

  // ─── Individual angle / hardstop setters ───

  /** Release both angle servos only (no hardstop change). */
  public void releaseAngleServos() {
    io.setLeftSecondaryHookAnglePosition(ClimbConstants.AngleServo.RELEASED_POSITION);
    io.setRightSecondaryHookAnglePosition(ClimbConstants.AngleServo.RELEASED_POSITION);
  }

  /** Release both hardstop servos only (no angle change). */
  public void releaseHardstopServos() {
    io.setLeftSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.RELEASED_POSITION);
    io.setRightSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.RELEASED_POSITION);
  }

  /** Stow both angle servos only (no hardstop change). */
  public void stowAngleServos() {
    io.setLeftSecondaryHookAnglePosition(ClimbConstants.AngleServo.STOWED_POSITION);
    io.setRightSecondaryHookAnglePosition(ClimbConstants.AngleServo.STOWED_POSITION);
  }

  /** Stow both hardstop servos only (no angle change). */
  public void stowHardstopServos() {
    io.setLeftSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.STOWED_POSITION);
    io.setRightSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.STOWED_POSITION);
  }

  // ─── Command-returning versions for teleop/auto ───

  /** Command: stow all secondary hook servos. Fire-and-forget (runOnce). */
  public Command stowAllServosCommand() {
    return runOnce(this::stowAllServos).withName("ClimbStowAllServos");
  }

  /** Command: release all secondary hook servos. Fire-and-forget (runOnce). */
  public Command releaseAllServosCommand() {
    return runOnce(this::releaseAllServos).withName("ClimbReleaseAllServos");
  }

  /** Command: stow left secondary hook servos. Fire-and-forget (runOnce). */
  public Command stowLeftServosCommand() {
    return runOnce(this::stowLeftServos).withName("ClimbStowLeftServos");
  }

  /** Command: stow right secondary hook servos. Fire-and-forget (runOnce). */
  public Command stowRightServosCommand() {
    return runOnce(this::stowRightServos).withName("ClimbStowRightServos");
  }

  /** Command: release left secondary hook servos. Fire-and-forget (runOnce). */
  public Command releaseLeftServosCommand() {
    return runOnce(this::releaseLeftServos).withName("ClimbReleaseLeftServos");
  }

  /** Command: release right secondary hook servos. Fire-and-forget (runOnce). */
  public Command releaseRightServosCommand() {
    return runOnce(this::releaseRightServos).withName("ClimbReleaseRightServos");
  }

  /** Command: stow left angle servo only (no hardstop change). Fire-and-forget (runOnce). */
  public Command stowLeftAngleServoCommand() {
    return runOnce(
            () -> io.setLeftSecondaryHookAnglePosition(ClimbConstants.AngleServo.STOWED_POSITION))
        .withName("ClimbStowLeftAngleServo");
  }

  /** Command: release left angle servo only (no hardstop change). Fire-and-forget (runOnce). */
  public Command releaseLeftAngleServoCommand() {
    return runOnce(
            () -> io.setLeftSecondaryHookAnglePosition(ClimbConstants.AngleServo.RELEASED_POSITION))
        .withName("ClimbReleaseLeftAngleServo");
  }

  /** Command: stow left hardstop servo only (no angle change). Fire-and-forget (runOnce). */
  public Command stowLeftHardstopServoCommand() {
    return runOnce(
            () ->
                io.setLeftSecondaryHookHardstopPosition(
                    ClimbConstants.HardstopServo.STOWED_POSITION))
        .withName("ClimbStowLeftHardstopServo");
  }

  /** Command: release left hardstop servo only (no angle change). Fire-and-forget (runOnce). */
  public Command releaseLeftHardstopServoCommand() {
    return runOnce(
            () ->
                io.setLeftSecondaryHookHardstopPosition(
                    ClimbConstants.HardstopServo.RELEASED_POSITION))
        .withName("ClimbReleaseLeftHardstopServo");
  }

  // ─── Servo sequence building blocks (command + wait for travel) ───

  /** Release angle servos and wait for them to reach the released position. */
  private Command releaseAngleServosAndWait() {
    return Commands.sequence(
        runOnce(this::releaseAngleServos),
        Commands.waitSeconds(ClimbConstants.AngleServo.TRAVEL_TIME_SEC));
  }

  /** Release hardstop servos (fire-and-forget, no wait). */
  private Command releaseHardstopServosInstant() {
    return runOnce(this::releaseHardstopServos);
  }

  /** Stow hardstop servos and wait, then stow angle servos. */
  private Command stowHardstopThenAngle() {
    return Commands.sequence(
        runOnce(this::stowHardstopServos),
        Commands.waitSeconds(ClimbConstants.HardstopServo.TRAVEL_TIME_SEC),
        runOnce(this::stowAngleServos));
  }

  // ===========================================================================
  // TELEOP CLIMB STEP (11 presses)
  // ===========================================================================

  /**
   * Advance climb by one operator-triggered step. Each POV-Right press runs exactly one action:
   *
   * <pre>
   *  Press  1 (STOWED)             → Extend L1 (path)
   *  Press  2 (EXTEND_L1)          → Retract L1 (path)
   *  Press  3 (RETRACT_L1)         → Release angle servos
   *  Press  4 (RELEASE_ANGLE_L1)   → Release hardstop servos
   *  Press  5 (RELEASE_HARDSTOP_L1)→ Extend L2 (path)
   *  Press  6 (EXTEND_L2)          → Retract L2 (path)
   *  Press  7 (RETRACT_L2)         → Stow hardstop then angle servos
   *  Press  8 (STOW_SERVOS_L2)     → Release angle servos
   *  Press  9 (RELEASE_ANGLE_L2)   → Release hardstop servos
   *  Press 10 (RELEASE_HARDSTOP_L2)→ Extend L3 (path)
   *  Press 11 (EXTEND_L3)          → Retract L3 (path)
   * </pre>
   */
  public Command nextClimbStep() {
    return Commands.defer(
        () -> {
          switch (currentState) {
            case STOWED:
              return setStateCommand(ClimbState.EXTEND_L1);

            case EXTEND_L1:
              return setStateCommand(ClimbState.RETRACT_L1);

            case RETRACT_L1:
              return Commands.sequence(
                  releaseAngleServosAndWait(),
                  Commands.runOnce(
                      () -> {
                        currentState = ClimbState.RELEASE_ANGLE_L1;
                        Logger.recordOutput("Climb/CurrentState", currentState.getName());
                      }));

            case RELEASE_ANGLE_L1:
              return Commands.sequence(
                  releaseHardstopServosInstant(),
                  Commands.runOnce(
                      () -> {
                        currentState = ClimbState.RELEASE_HARDSTOP_L1;
                        Logger.recordOutput("Climb/CurrentState", currentState.getName());
                      }));

            case RELEASE_HARDSTOP_L1:
              return setStateCommand(ClimbState.EXTEND_L2);

            case EXTEND_L2:
              return setStateCommand(ClimbState.RETRACT_L2);

            case RETRACT_L2:
              return Commands.sequence(
                  stowHardstopThenAngle(),
                  Commands.runOnce(
                      () -> {
                        currentState = ClimbState.STOW_SERVOS_L2;
                        Logger.recordOutput("Climb/CurrentState", currentState.getName());
                      }));

            case STOW_SERVOS_L2:
              return Commands.sequence(
                  releaseAngleServosAndWait(),
                  Commands.runOnce(
                      () -> {
                        currentState = ClimbState.RELEASE_ANGLE_L2;
                        Logger.recordOutput("Climb/CurrentState", currentState.getName());
                      }));

            case RELEASE_ANGLE_L2:
              return Commands.sequence(
                  releaseHardstopServosInstant(),
                  Commands.runOnce(
                      () -> {
                        currentState = ClimbState.RELEASE_HARDSTOP_L2;
                        Logger.recordOutput("Climb/CurrentState", currentState.getName());
                      }));

            case RELEASE_HARDSTOP_L2:
              return setStateCommand(ClimbState.EXTEND_L3);

            case EXTEND_L3:
              return setStateCommand(ClimbState.RETRACT_L3);

            default:
              return Commands.none();
          }
        },
        Set.of(this));
  }

  /**
   * Go back one operator-triggered step. Each POV-Left press undoes exactly one action, reversing
   * servo changes or running the reversed path as appropriate.
   */
  public Command previousClimbStep() {
    return Commands.defer(
        () -> {
          switch (currentState) {
            case EXTEND_L1:
              // Reverse the extend path back to STOWED
              return previousState();

            case RETRACT_L1:
              // Reverse the retract path back to EXTEND_L1
              return previousState();

            case RELEASE_ANGLE_L1:
              // Stow angle servos back to RETRACT_L1
              return Commands.sequence(
                  runOnce(this::stowAngleServos),
                  Commands.runOnce(
                      () -> {
                        currentState = ClimbState.RETRACT_L1;
                        Logger.recordOutput("Climb/CurrentState", currentState.getName());
                      }));

            case RELEASE_HARDSTOP_L1:
              // Stow hardstop servos back to RELEASE_ANGLE_L1
              return Commands.sequence(
                  runOnce(this::stowHardstopServos),
                  Commands.runOnce(
                      () -> {
                        currentState = ClimbState.RELEASE_ANGLE_L1;
                        Logger.recordOutput("Climb/CurrentState", currentState.getName());
                      }));

            case EXTEND_L2:
              // Reverse the extend path back to RELEASE_HARDSTOP_L1
              return Commands.sequence(
                  previousState(),
                  Commands.runOnce(
                      () -> {
                        currentState = ClimbState.RELEASE_HARDSTOP_L1;
                        Logger.recordOutput("Climb/CurrentState", currentState.getName());
                      }));

            case RETRACT_L2:
              // Reverse the retract path back to EXTEND_L2
              return previousState();

            case STOW_SERVOS_L2:
              // Release servos again back to RETRACT_L2
              return Commands.sequence(
                  releaseAngleServosAndWait(),
                  releaseHardstopServosInstant(),
                  Commands.runOnce(
                      () -> {
                        currentState = ClimbState.RETRACT_L2;
                        Logger.recordOutput("Climb/CurrentState", currentState.getName());
                      }));

            case RELEASE_ANGLE_L2:
              // Stow angle servos back to STOW_SERVOS_L2
              return Commands.sequence(
                  runOnce(this::stowAngleServos),
                  Commands.runOnce(
                      () -> {
                        currentState = ClimbState.STOW_SERVOS_L2;
                        Logger.recordOutput("Climb/CurrentState", currentState.getName());
                      }));

            case RELEASE_HARDSTOP_L2:
              // Stow hardstop servos back to RELEASE_ANGLE_L2
              return Commands.sequence(
                  runOnce(this::stowHardstopServos),
                  Commands.runOnce(
                      () -> {
                        currentState = ClimbState.RELEASE_ANGLE_L2;
                        Logger.recordOutput("Climb/CurrentState", currentState.getName());
                      }));

            case EXTEND_L3:
              // Reverse the extend path back to RELEASE_HARDSTOP_L2
              return Commands.sequence(
                  previousState(),
                  Commands.runOnce(
                      () -> {
                        currentState = ClimbState.RELEASE_HARDSTOP_L2;
                        Logger.recordOutput("Climb/CurrentState", currentState.getName());
                      }));

            case RETRACT_L3:
              // Reverse the retract path back to EXTEND_L3
              return previousState();

            default:
              return Commands.none();
          }
        },
        Set.of(this));
  }

  // ===========================================================================
  // EMERGENCY
  // ===========================================================================

  // ===========================================================================
  // TEST / AD-HOC PATH COMMANDS
  // ===========================================================================

  /**
   * Create a command that follows a straight-line path from the current target position to the
   * given position. Useful for testing climb motions from the controller.
   *
   * @param target The target end-effector position (meters)
   * @param duration Time to complete the path (seconds)
   * @param pulling Whether this is a pulling motion (adds gravity feedforward)
   */
  public Command runPathToPosition(Translation2d target, boolean pulling) {
    return runPath(
        () -> {
          Translation2d start = leftTargetPosition; // symmetric, use left as reference
          if (start.getDistance(target) < 0.005) {
            // Already at target, just hold position
            setTargetPositionsInternal(target, target);
            return null;
          }
          currentState = ClimbState.MANUAL;
          Logger.recordOutput("Climb/CurrentState", "MANUAL");
          return new PathParams(List.of(start, target), pulling);
        },
        "ClimbPathTo_" + String.format("%.2f_%.2f", target.getX(), target.getY()));
  }

  /** Immediately stop all motors and enter emergency state. */
  public void stopMotors() {
    io.stop();
    currentState = ClimbState.EMERGENCY_STOP;
  }

  /** Command to emergency-stop the climb subsystem. */
  public Command emergencyStop() {
    Command cmd = runOnce(this::stopMotors);
    cmd.setName("ClimbEmergencyStop");
    return cmd;
  }

  // ─── Direct voltage control for testing ───

  /** Set the left front motor voltage directly. */
  public void setLeftFrontVoltage(double volts) {
    io.setLeftFrontVoltage(volts);
  }

  /** Set the left back motor voltage directly. */
  public void setLeftBackVoltage(double volts) {
    io.setLeftBackVoltage(volts);
  }

  /** Set the right front motor voltage directly. */
  public void setRightFrontVoltage(double volts) {
    io.setRightFrontVoltage(volts);
  }

  /** Set the right back motor voltage directly. */
  public void setRightBackVoltage(double volts) {
    io.setRightBackVoltage(volts);
  }

  // ===========================================================================
  // CALIBRATION MODE
  // ===========================================================================

  /** Whether the climb is currently in calibration mode. */
  public boolean isInCalibrationMode() {
    return calibrationMode;
  }

  /**
   * Enter calibration mode. Stops all motors and allows individual motor voltage control. Normal
   * climb state commands are blocked while in calibration mode.
   */
  public Command enterCalibrationMode() {
    return runOnce(
            () -> {
              calibrationMode = true;
              io.stop();
              Logger.recordOutput("Climb/CalibrationMode", true);
              System.out.println("[Climb] Entered calibration mode");
            })
        .withName("ClimbEnterCalibration");
  }

  /**
   * Exit calibration mode. Stops all motors, recalibrates encoder positions to match the initial
   * STOWED end-effector pose, then returns to STOWED state with position hold.
   */
  public Command exitCalibrationMode() {
    return runOnce(
            () -> {
              io.stop();
              io.recalibrateEncoders();
              calibrationMode = false;
              setState(ClimbState.STOWED);
              Logger.recordOutput("Climb/CalibrationMode", false);
              System.out.println("[Climb] Exited calibration mode — encoders recalibrated");
            })
        .withName("ClimbExitCalibration");
  }

  /**
   * Command that runs a specific motor at a voltage while held, and stops it when released. Only
   * active during calibration mode.
   *
   * @param motorName Display name for logging
   * @param voltageSetter Consumer to set the motor voltage
   * @param voltage The voltage to apply (positive or negative)
   */
  private Command calibrationMotorCommand(
      String motorName, java.util.function.DoubleConsumer voltageSetter, double voltage) {
    return Commands.startEnd(
            () -> {
              if (calibrationMode) {
                voltageSetter.accept(voltage);
                Logger.recordOutput("Climb/Calibration/" + motorName, voltage);
              }
            },
            () -> {
              voltageSetter.accept(0.0);
              Logger.recordOutput("Climb/Calibration/" + motorName, 0.0);
            },
            this)
        .withName("ClimbCal_" + motorName);
  }

  /** Left front motor forward (+3V). Only active in calibration mode. */
  public Command calibrationLeftFrontForward() {
    return calibrationMotorCommand("LeftFront", io::setLeftFrontVoltage, 3.0);
  }

  /** Left front motor reverse (-3V). Only active in calibration mode. */
  public Command calibrationLeftFrontReverse() {
    return calibrationMotorCommand("LeftFront", io::setLeftFrontVoltage, -3.0);
  }

  /** Left back motor forward (+3V). Only active in calibration mode. */
  public Command calibrationLeftBackForward() {
    return calibrationMotorCommand("LeftBack", io::setLeftBackVoltage, 3.0);
  }

  /** Left back motor reverse (-3V). Only active in calibration mode. */
  public Command calibrationLeftBackReverse() {
    return calibrationMotorCommand("LeftBack", io::setLeftBackVoltage, -3.0);
  }

  /** Right front motor forward (+3V). Only active in calibration mode. */
  public Command calibrationRightFrontForward() {
    return calibrationMotorCommand("RightFront", io::setRightFrontVoltage, 3.0);
  }

  /** Right front motor reverse (-3V). Only active in calibration mode. */
  public Command calibrationRightFrontReverse() {
    return calibrationMotorCommand("RightFront", io::setRightFrontVoltage, -3.0);
  }

  /** Right back motor forward (+3V). Only active in calibration mode. */
  public Command calibrationRightBackForward() {
    return calibrationMotorCommand("RightBack", io::setRightBackVoltage, 3.0);
  }

  /** Right back motor reverse (-3V). Only active in calibration mode. */
  public Command calibrationRightBackReverse() {
    return calibrationMotorCommand("RightBack", io::setRightBackVoltage, -3.0);
  }
}
