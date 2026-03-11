package frc.robot.subsystems.climb;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climb.util.ClimbIK;
import frc.robot.subsystems.climb.util.ClimbIK.ClimbIKResult;
import frc.robot.subsystems.climb.util.ClimbPathPlanner;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Climb subsystem: ClimbState waypoints -> ClimbIK -> TalonFX motors. Hybrid control: velocity
 * (Jacobian) during path following, position (MotionMagic) for holds.
 */
public class ClimbSubsystem extends SubsystemBase {

  /**
   * Dashboard-selectable climb level. L1 = auto L1 sequence (2 presses). L2L3 = full teleop
   * sequence through all levels.
   */
  public enum ClimbLevel {
    L1,
    L2L3
  }

  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private ClimbState currentState = ClimbState.STOWED;
  private Translation2d leftTargetPosition = ClimbState.STOWED.getTargetPosition();
  private Translation2d rightTargetPosition = ClimbState.STOWED.getTargetPosition();

  // ─── Last successful FK results (Newton solver initial guess) ───
  private Translation2d lastMeasuredLeft = ClimbState.STOWED.getTargetPosition();
  private Translation2d lastMeasuredRight = ClimbState.STOWED.getTargetPosition();

  // ─── Climb level (set from RobotContainer via supplier) ───
  private Supplier<ClimbLevel> climbLevelSupplier = () -> ClimbLevel.L2L3;

  // ─── Path execution tracking ───
  /** True while a climb path command is actively running (between initialize and end). */
  private boolean pathRunning = false;

  // ─── Calibration mode ───
  private boolean calibrationMode = false;
  // ─── Manual control mode (operator mushroom heads) ───
  private boolean manualMode = false;

  // ─── IMU Climb Assist (auto-level during retract) ───
  private DoubleSupplier rollDegreesSupplier = () -> 0.0;
  private boolean autoLevelEnabled = ClimbConstants.ImuAssist.ENABLED;
  private final PIDController autoLevelPID =
      new PIDController(
          ClimbConstants.ImuAssist.KP, ClimbConstants.ImuAssist.KI, ClimbConstants.ImuAssist.KD);

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

  // ─── Left side Mechanism2d ───
  private final Mechanism2d leftMechanism;
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

  // ─── Right side Mechanism2d ───
  private final Mechanism2d rightMechanism;
  private final MechanismLigament2d rightTargetLink1;
  private final MechanismLigament2d rightTargetLink2;
  private final MechanismLigament2d rightMeasuredLink1;
  private final MechanismLigament2d rightMeasuredLink2;
  private final MechanismLigament2d rightTargetBackCable;
  private final MechanismLigament2d rightTargetFrontCable;

  public ClimbSubsystem(ClimbIO io) {
    this.io = io;

    final double sx = ClimbConstants.SHOULDER_X_METERS;
    final double sy = ClimbConstants.SHOULDER_Y_METERS;
    final double wfx = ClimbConstants.FRONT_WINCH_X_METERS;
    final double wfy = ClimbConstants.FRONT_WINCH_Y_METERS;

    // ══════════════════════════════════════════════════════════════════════
    // LEFT side Mechanism2d
    // ══════════════════════════════════════════════════════════════════════
    leftMechanism = new Mechanism2d(CANVAS_W, CANVAS_H);

    // ── Fixed-point markers ──
    MechanismRoot2d backWinchRoot =
        leftMechanism.getRoot("BackWinch", 0 + CANVAS_OFFSET_X, 0 + CANVAS_OFFSET_Y);
    backWinchRoot.append(
        new MechanismLigament2d("BackWinchDot", 0.02, 0, 8, new Color8Bit(Color.kWhite)));

    MechanismRoot2d frontWinchRoot =
        leftMechanism.getRoot("FrontWinch", wfx + CANVAS_OFFSET_X, wfy + CANVAS_OFFSET_Y);
    frontWinchRoot.append(
        new MechanismLigament2d("FrontWinchDot", 0.02, 0, 8, new Color8Bit(Color.kWhite)));

    // ── Target arm (blue) — rooted at shoulder ──
    MechanismRoot2d targetShoulderRoot =
        leftMechanism.getRoot("TargetShoulder", sx + CANVAS_OFFSET_X, sy + CANVAS_OFFSET_Y);
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
        leftMechanism.getRoot("MeasuredShoulder", sx + CANVAS_OFFSET_X, sy + CANVAS_OFFSET_Y);
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

    // ── Cable lines ──
    targetBackCable =
        backWinchRoot.append(
            new MechanismLigament2d("BackCable", 0.3, 90, 1, new Color8Bit(Color.kYellow)));
    targetFrontCable =
        frontWinchRoot.append(
            new MechanismLigament2d("FrontCable", 0.1, 90, 1, new Color8Bit(Color.kOrange)));

    // ══════════════════════════════════════════════════════════════════════
    // RIGHT side Mechanism2d
    // ══════════════════════════════════════════════════════════════════════
    rightMechanism = new Mechanism2d(CANVAS_W, CANVAS_H);

    MechanismRoot2d rBackWinchRoot =
        rightMechanism.getRoot("BackWinch", 0 + CANVAS_OFFSET_X, 0 + CANVAS_OFFSET_Y);
    rBackWinchRoot.append(
        new MechanismLigament2d("BackWinchDot", 0.02, 0, 8, new Color8Bit(Color.kWhite)));

    MechanismRoot2d rFrontWinchRoot =
        rightMechanism.getRoot("FrontWinch", wfx + CANVAS_OFFSET_X, wfy + CANVAS_OFFSET_Y);
    rFrontWinchRoot.append(
        new MechanismLigament2d("FrontWinchDot", 0.02, 0, 8, new Color8Bit(Color.kWhite)));

    MechanismRoot2d rTargetShoulderRoot =
        rightMechanism.getRoot("TargetShoulder", sx + CANVAS_OFFSET_X, sy + CANVAS_OFFSET_Y);
    rightTargetLink1 =
        rTargetShoulderRoot.append(
            new MechanismLigament2d(
                "TargetLink1",
                ClimbConstants.LINK_1_LENGTH_METERS,
                90,
                5,
                new Color8Bit(Color.kRed)));
    rightTargetLink2 =
        rightTargetLink1.append(
            new MechanismLigament2d(
                "TargetLink2",
                ClimbConstants.LINK_2_LENGTH_METERS,
                0,
                4,
                new Color8Bit(Color.kOrangeRed)));

    MechanismRoot2d rMeasuredShoulderRoot =
        rightMechanism.getRoot("MeasuredShoulder", sx + CANVAS_OFFSET_X, sy + CANVAS_OFFSET_Y);
    rightMeasuredLink1 =
        rMeasuredShoulderRoot.append(
            new MechanismLigament2d(
                "MeasuredLink1",
                ClimbConstants.LINK_1_LENGTH_METERS,
                90,
                3,
                new Color8Bit(Color.kMagenta)));
    rightMeasuredLink2 =
        rightMeasuredLink1.append(
            new MechanismLigament2d(
                "MeasuredLink2",
                ClimbConstants.LINK_2_LENGTH_METERS,
                0,
                2,
                new Color8Bit(Color.kHotPink)));

    rightTargetBackCable =
        rBackWinchRoot.append(
            new MechanismLigament2d("BackCable", 0.3, 90, 1, new Color8Bit(Color.kYellow)));
    rightTargetFrontCable =
        rFrontWinchRoot.append(
            new MechanismLigament2d("FrontCable", 0.1, 90, 1, new Color8Bit(Color.kOrange)));

    // ── Initialize both Mechanism2d to the STOWED pose ──
    initializeMechanism2dToStowed();

    // Publish both — separate widgets in AdvantageScope
    SmartDashboard.putData("Climb/LeftMechanism2d", leftMechanism);
    SmartDashboard.putData("Climb/RightMechanism2d", rightMechanism);

    // Initialize calibration mode state so triggers work from first cycle
    Logger.recordOutput("Climb/CalibrationMode", false);
    Logger.recordOutput("Climb/CurrentState", ClimbState.STOWED.getName());
  }

  /**
   * Set all Mechanism2d ligament angles/lengths to match the STOWED position so the visualization
   * is correct. Called from the constructor and from {@link #resetToStowed()} to re-initialize the
   * visualization when auto restarts.
   */
  private void initializeMechanism2dToStowed() {
    final double sx = ClimbConstants.SHOULDER_X_METERS;
    final double sy = ClimbConstants.SHOULDER_Y_METERS;
    final double wfx = ClimbConstants.FRONT_WINCH_X_METERS;
    final double wfy = ClimbConstants.FRONT_WINCH_Y_METERS;
    final double L1 = ClimbConstants.LINK_1_LENGTH_METERS;
    final double L2 = ClimbConstants.LINK_2_LENGTH_METERS;
    final double backAttach = ClimbConstants.BACK_CABLE_ATTACH_ON_LINK1_METERS;
    final double frontAttach = ClimbConstants.FRONT_CABLE_ATTACH_ON_LINK2_METERS;

    Translation2d stowedPos = ClimbState.STOWED.getTargetPosition();
    ClimbIK.ClimbSideIKResult ik = ClimbIK.calculateIK(stowedPos, false);
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

    // Left side
    targetLink1.setAngle(link1Angle);
    targetLink2.setAngle(link2AbsAngle - link1Angle);
    measuredLink1.setAngle(link1Angle);
    measuredLink2.setAngle(link2AbsAngle - link1Angle);

    // Right side (same stowed pose)
    rightTargetLink1.setAngle(link1Angle);
    rightTargetLink2.setAngle(link2AbsAngle - link1Angle);
    rightMeasuredLink1.setAngle(link1Angle);
    rightMeasuredLink2.setAngle(link2AbsAngle - link1Angle);

    // Back cable: from backWinch(0,0) to point P on link 1
    double t1 = backAttach / L1;
    double px = jx + t1 * (sx - jx);
    double py = jy + t1 * (sy - jy);
    targetBackCable.setLength(Math.hypot(px, py));
    targetBackCable.setAngle(Math.toDegrees(Math.atan2(py, px)));
    rightTargetBackCable.setLength(Math.hypot(px, py));
    rightTargetBackCable.setAngle(Math.toDegrees(Math.atan2(py, px)));

    // Front cable: from frontWinch to point Q on link 2
    double t2 = frontAttach / L2;
    double qx = ex + t2 * (jx - ex);
    double qy = ey + t2 * (jy - ey);
    targetFrontCable.setLength(Math.hypot(qx - wfx, qy - wfy));
    targetFrontCable.setAngle(Math.toDegrees(Math.atan2(qy - wfy, qx - wfx)));
    rightTargetFrontCable.setLength(Math.hypot(qx - wfx, qy - wfy));
    rightTargetFrontCable.setAngle(Math.toDegrees(Math.atan2(qy - wfy, qx - wfx)));
  }

  /**
   * Set the supplier for the climb level chooser. Called from RobotContainer after constructing the
   * SendableChooser.
   */
  public void setClimbLevelSupplier(Supplier<ClimbLevel> supplier) {
    this.climbLevelSupplier = supplier;
  }

  /** Get the current dashboard-selected climb level. */
  public ClimbLevel getClimbLevel() {
    return climbLevelSupplier.get();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    // State is logged by commands (setState, runPath)
    Logger.recordOutput("Climb/CalibrationMode", calibrationMode);
    Logger.recordOutput("Climb/ClimbLevel", getClimbLevel().name());

    // Skip FK/IK visualization when stowed, no path is actively running, AND motors have
    // actually settled near the stowed position. Without the pathRunning check,
    // previousState() → STOWED would freeze the visualization mid-motion because it sets
    // currentState = STOWED before the return path starts. Without the settling check,
    // the measured arm would freeze when MotionMagic is still driving motors to STOWED
    // after a path ends (path velocity control undershoots, MotionMagic fills the gap).
    boolean motorsSettled =
        lastMeasuredLeft.getDistance(ClimbState.STOWED.getTargetPosition())
                < ClimbConstants.IK_POSITION_TOLERANCE_METERS
            && lastMeasuredRight.getDistance(ClimbState.STOWED.getTargetPosition())
                < ClimbConstants.IK_POSITION_TOLERANCE_METERS;
    boolean skipVisualization =
        currentState == ClimbState.STOWED
            && !pathRunning
            && motorsSettled
            && !calibrationMode
            && !manualMode;

    if (!skipVisualization) {
      Logger.recordOutput("Climb/LeftTargetPosition", leftTargetPosition);
      Logger.recordOutput("Climb/RightTargetPosition", rightTargetPosition);
      Logger.recordOutput("Climb/AutoLevel/Enabled", autoLevelEnabled);

      // Estimate and log actual end effector positions from motor encoders (FK)
      // Use last successful FK result as initial guess — it's closer to the true position
      // than the target, especially when motors lag behind during velocity-controlled paths.
      Translation2d measuredLeft =
          ClimbIK.estimateEndEffectorPosition(
              inputs.leftFrontPositionRotations,
              inputs.leftBackPositionRotations,
              lastMeasuredLeft);
      Translation2d measuredRight =
          ClimbIK.estimateEndEffectorPosition(
              inputs.rightFrontPositionRotations,
              inputs.rightBackPositionRotations,
              lastMeasuredRight);
      if (measuredLeft != null) {
        lastMeasuredLeft = measuredLeft;
        Logger.recordOutput("Climb/LeftMeasuredPosition", measuredLeft);
      }
      if (measuredRight != null) {
        lastMeasuredRight = measuredRight;
        Logger.recordOutput("Climb/RightMeasuredPosition", measuredRight);
      }

      // Log motor rotations directly
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

      // --- LEFT Target arm (blue) ---
      ClimbIK.ClimbSideIKResult leftTargetIK = ClimbIK.calculateIK(leftTargetPosition, false);
      if (leftTargetIK.isValid) {
        double jx = leftTargetIK.jointX;
        double jy = leftTargetIK.jointY;
        double ex = leftTargetPosition.getX();
        double ey = leftTargetPosition.getY();

        double link1Angle = Math.toDegrees(Math.atan2(jy - sy, jx - sx));
        targetLink1.setAngle(link1Angle);
        double link2AbsAngle = Math.toDegrees(Math.atan2(ey - jy, ex - jx));
        targetLink2.setAngle(link2AbsAngle - link1Angle);

        double t1 = backAttach / L1;
        double px = jx + t1 * (sx - jx);
        double py = jy + t1 * (sy - jy);
        targetBackCable.setLength(Math.hypot(px, py));
        targetBackCable.setAngle(Math.toDegrees(Math.atan2(py, px)));

        double t2 = frontAttach / L2;
        double qx = ex + t2 * (jx - ex);
        double qy = ey + t2 * (jy - ey);
        targetFrontCable.setLength(Math.hypot(qx - wfx, qy - wfy));
        targetFrontCable.setAngle(Math.toDegrees(Math.atan2(qy - wfy, qx - wfx)));
      }

      // --- LEFT Measured arm (green) ---
      if (measuredLeft != null) {
        ClimbIK.ClimbSideIKResult measIK = ClimbIK.calculateIK(measuredLeft, false);
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

      // --- RIGHT Target arm (red) ---
      ClimbIK.ClimbSideIKResult rightTargetIK = ClimbIK.calculateIK(rightTargetPosition, false);
      if (rightTargetIK.isValid) {
        double jx = rightTargetIK.jointX;
        double jy = rightTargetIK.jointY;
        double ex = rightTargetPosition.getX();
        double ey = rightTargetPosition.getY();

        double link1Angle = Math.toDegrees(Math.atan2(jy - sy, jx - sx));
        rightTargetLink1.setAngle(link1Angle);
        double link2AbsAngle = Math.toDegrees(Math.atan2(ey - jy, ex - jx));
        rightTargetLink2.setAngle(link2AbsAngle - link1Angle);

        double t1 = backAttach / L1;
        double px = jx + t1 * (sx - jx);
        double py = jy + t1 * (sy - jy);
        rightTargetBackCable.setLength(Math.hypot(px, py));
        rightTargetBackCable.setAngle(Math.toDegrees(Math.atan2(py, px)));

        double t2 = frontAttach / L2;
        double qx = ex + t2 * (jx - ex);
        double qy = ey + t2 * (jy - ey);
        rightTargetFrontCable.setLength(Math.hypot(qx - wfx, qy - wfy));
        rightTargetFrontCable.setAngle(Math.toDegrees(Math.atan2(qy - wfy, qx - wfx)));
      }

      // --- RIGHT Measured arm (magenta) ---
      if (measuredRight != null) {
        ClimbIK.ClimbSideIKResult measIK = ClimbIK.calculateIK(measuredRight, false);
        if (measIK.isValid) {
          double jx = measIK.jointX;
          double jy = measIK.jointY;
          double ex = measuredRight.getX();
          double ey = measuredRight.getY();

          double link1Angle = Math.toDegrees(Math.atan2(jy - sy, jx - sx));
          rightMeasuredLink1.setAngle(link1Angle);
          double link2AbsAngle = Math.toDegrees(Math.atan2(ey - jy, ex - jx));
          rightMeasuredLink2.setAngle(link2AbsAngle - link1Angle);
        }
      }
    } // end if (!skipVisualization)

    // Mechanism2d is published via SmartDashboard.putData in constructor — auto-updates
  }

  // =============================================================================
  // STATE MANAGEMENT
  // =============================================================================

  /** Reset climb to STOWED — resets state, IO motor positions, FK guesses, and Mechanism2d. */
  public void resetToStowed() {
    io.resetToStowed();
    setState(ClimbState.STOWED);
    // Reset FK initial guesses so the Newton solver starts from the stowed position
    lastMeasuredLeft = ClimbState.STOWED.getTargetPosition();
    lastMeasuredRight = ClimbState.STOWED.getTargetPosition();
    // Re-initialize Mechanism2d ligaments to stowed angles so the visualization resets
    // when auto restarts (periodic() skips visualization updates while STOWED).
    initializeMechanism2dToStowed();
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
          // Reverse transitions invert isPulling — except when stowing, which is always
          // arm-weight-only (not pulling) regardless of the state we came from.
          boolean isPulling = prev == ClimbState.STOWED ? false : !currentState.isPulling();
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
   * Build deferred teleop auto-climb: idle -> pathfind to standoff -> extend arms -> PID approach
   * -> stop. L1 retracts immediately; L2L3 waits for driver POV.
   */
  public Command buildTeleopAutoClimb(
      DriveSwerveDrivetrain drive,
      Superstructure superstructure,
      Supplier<Pose2d> climbTargetSupplier) {
    return Commands.defer(
        () -> {
          // Read dashboard selections at command-creation time (inside defer)
          Pose2d climbTarget = climbTargetSupplier.get();
          boolean isL1 = getClimbLevel() == ClimbLevel.L1;
          ClimbState extendState = isL1 ? ClimbState.EXTEND_L1_AUTO : ClimbState.EXTEND_L1;

          // Standoff pose: offset away from the wall so pathfinding ends short of the tower
          double approachDist = AutoConstants.CLIMB_APPROACH_DISTANCE_M;
          Translation2d standoffOffset =
              new Translation2d(approachDist, climbTarget.getRotation().plus(Rotation2d.k180deg));
          Pose2d standoffPose =
              new Pose2d(
                  climbTarget.getTranslation().plus(standoffOffset), climbTarget.getRotation());

          // PID controllers for the final straight-line approach
          double maxVelocity = AutoConstants.CLIMB_APPROACH_MAX_VELOCITY_MPS;
          double driveTolerance = AutoConstants.CLIMB_APPROACH_TOLERANCE_M;
          double thetaTolerance = AutoConstants.CLIMB_APPROACH_THETA_TOLERANCE_RAD;

          // Drive-to-point command using DriveSwerveDrivetrain's 2910-style state
          Command driveToPose =
              Commands.sequence(
                  Commands.runOnce(
                      () -> drive.setDesiredPoseForDriveToPoint(climbTarget, maxVelocity), drive),
                  Commands.waitUntil(
                      () -> drive.isAtDriveToPointSetpoint(driveTolerance, thetaTolerance)));

          // Assemble the full sequence
          Command sequence =
              Commands.sequence(
                  // Phase 0: Idle superstructure so intake stows (arm clearance)
                  superstructure.idle(),
                  // Phase 1: Pathfind to standoff while extending climb arms in parallel
                  Commands.parallel(
                      Commands.defer(
                          () ->
                              AutoBuilder.pathfindToPose(
                                  standoffPose, drive.getPathConstraints(), 0.0),
                          Set.of(drive)),
                      Commands.sequence(
                          Commands.waitUntil(superstructure::isIntakeStowed),
                          setStateCommand(extendState))),
                  // Phase 2: PID straight-line approach into the tower
                  driveToPose,
                  Commands.runOnce(() -> drive.stop(), drive));

          // Phase 3: If L1, retract immediately. L2L3 → command ends, driver continues via POV
          // Right.
          if (isL1) {
            sequence = sequence.andThen(setStateCommand(ClimbState.RETRACT_L1_AUTO));
          }

          return sequence.withName("TeleopAutoClimb_" + (isL1 ? "L1" : "L2L3"));
        },
        Set.of(drive, superstructure, this));
  }

  /**
   * Release from auto L1 by reversing RETRACT_L1_AUTO back to EXTEND_L1_AUTO. No-op if not in
   * RETRACT_L1_AUTO state.
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

  /**
   * Follow a direct path from the current position to STOWED without touching any servos. Used by
   * the L1 operator sequence where servos are never involved.
   */
  public Command stowPathOnly() {
    return runPath(
            () -> {
              if (currentState == ClimbState.STOWED) return null;
              Translation2d start = leftTargetPosition;
              Translation2d end = ClimbState.STOWED.getTargetPosition();
              currentState = ClimbState.STOWED;
              Logger.recordOutput("Climb/CurrentState", currentState.getName());
              return new PathParams(List.of(start, end), false);
            },
            "StowPathOnly")
        .withName("StowPathOnly");
  }

  // -- Helper: reusable path-following command --

  private record PathParams(List<Translation2d> waypoints, boolean isPulling) {}

  /**
   * Creates a command that calls the supplier on initialize to get path params, then follows the
   * path. If supplier returns null, finishes immediately (no path to run).
   */
  private Command runPath(Supplier<PathParams> paramSupplier, String name) {
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
        pathRunning = true;
        params = paramSupplier.get();
        System.out.println(
            "[Climb] runPath '"
                + name
                + "' initialize: params="
                + (params == null
                    ? "null"
                    : "waypoints="
                        + params.waypoints().size()
                        + " first="
                        + params.waypoints().get(0)
                        + " last="
                        + params.waypoints().get(params.waypoints().size() - 1)));
        if (params == null || params.waypoints().size() < 2) {
          System.out.println("[Climb] runPath '" + name + "': skipped (null or <2 waypoints)");
          return;
        }

        try {
          // Always decelerate to zero at the final waypoint. Previously this passed
          // params.isPulling() as maintainEndVelocity, which caused pulling (retract)
          // paths to arrive at full speed — the motors couldn't stop in time, producing
          // overshoot. Gravity/spring feedforward handles the load
          // during the path; there's no need to maintain velocity at the endpoint.
          var path = ClimbPathPlanner.createMultiBezierPath(params.waypoints(), 0.15, false);
          boolean valid = ClimbPathPlanner.isPathValid(path);
          System.out.println(
              "[Climb] runPath '"
                  + name
                  + "': path created, duration="
                  + path.getTotalDuration()
                  + "s, waypointCount="
                  + path.getWaypointCount()
                  + ", valid="
                  + valid);
          if (!valid) {
            // Find which waypoint fails
            for (int i = 0; i < path.getWaypoints().size(); i++) {
              var pt = path.getWaypoints().get(i);
              if (!ClimbIK.isPositionReachable(pt)) {
                System.out.println("[Climb]   UNREACHABLE waypoint " + i + ": " + pt);
              }
            }
            return;
          }
          executor = new ClimbPathPlanner.PathExecutor(path, path);
          executor.start();
          System.out.println("[Climb] runPath '" + name + "': executor started");
        } catch (Exception e) {
          System.out.println("[Climb] Path generation failed: " + e.getMessage());
          e.printStackTrace();
        }
      }

      @Override
      public void execute() {
        if (executor != null) {
          Translation2d[] targets = executor.getCurrentTargets();
          Translation2d[] velocities = executor.getCurrentVelocities();

          // ── IMU auto-level: adjust Y velocities differentially ──
          double velCorrection = getAutoLevelVelocityCorrection();
          if (velCorrection != 0.0 && params.isPulling()) {
            double maxVel = ClimbConstants.PATH_MAX_VELOCITY_MPS;
            double[] adjustedY =
                applyClampedVelocityCorrection(velocities[0].getY(), velCorrection, maxVel);
            velocities[0] = new Translation2d(velocities[0].getX(), adjustedY[0]);
            velocities[1] = new Translation2d(velocities[1].getX(), adjustedY[1]);
            Logger.recordOutput("Climb/AutoLevel/VelCorrectionMPS", velCorrection);
            Logger.recordOutput("Climb/AutoLevel/LeftAdjustedVelY", adjustedY[0]);
            Logger.recordOutput("Climb/AutoLevel/RightAdjustedVelY", adjustedY[1]);
          }

          setTargetVelocitiesInternal(
              targets[0], targets[1], velocities[0], velocities[1], params.isPulling());
        }
      }

      @Override
      public boolean isFinished() {
        boolean finished = executor == null || executor.isFinished();
        if (finished) {
          System.out.println(
              "[Climb] runPath '"
                  + name
                  + "' isFinished=true, executor="
                  + (executor == null ? "null" : "elapsed=" + executor.getElapsedTime()));
        }
        return finished;
      }

      @Override
      public void end(boolean interrupted) {
        pathRunning = false;
        System.out.println("[Climb] runPath '" + name + "' end: interrupted=" + interrupted);
        if (executor != null) executor.stop();

        // Switch directly to MotionMagic position hold at the final waypoint.
        // Do NOT send zero-velocity commands before position hold — in the sim,
        // each setXxxVelocity() call integrates position by one DT step, and
        // the subsequent setXxxPosition() integrates again, causing a double-step
        // overshoot. MotionMagic's internal profile handles deceleration from any
        // residual velocity, so going straight to position hold is both simpler
        // and more accurate.
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

      // Log motor position setpoints (mechanism rotations) for comparing against measured
      Logger.recordOutput("Climb/IK/LeftFrontSetpoint", ikResult.leftSide.frontMotorRotations);
      Logger.recordOutput("Climb/IK/LeftBackSetpoint", ikResult.leftSide.backMotorRotations);
      Logger.recordOutput("Climb/IK/RightFrontSetpoint", ikResult.rightSide.frontMotorRotations);
      Logger.recordOutput("Climb/IK/RightBackSetpoint", ikResult.rightSide.backMotorRotations);

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

  /** Set motor velocities via numerical Jacobian: EE velocity -> motor velocity. */
  private void setTargetVelocitiesInternal(
      Translation2d leftPosition,
      Translation2d rightPosition,
      Translation2d leftVelocity,
      Translation2d rightVelocity,
      boolean isPulling) {
    this.leftTargetPosition = leftPosition;
    this.rightTargetPosition = rightPosition;

    // Use lastMeasuredLeft/Right as the Newton solver initial guess — same fix as periodic().
    // The previous measured position is much closer to the true position than the target,
    // especially during velocity-controlled paths where tracking lag creates a gap.
    Translation2d measuredLeftPosition =
        ClimbIK.estimateEndEffectorPosition(
            inputs.leftFrontPositionRotations, inputs.leftBackPositionRotations, lastMeasuredLeft);
    Translation2d measuredRightPosition =
        ClimbIK.estimateEndEffectorPosition(
            inputs.rightFrontPositionRotations,
            inputs.rightBackPositionRotations,
            lastMeasuredRight);

    if (measuredLeftPosition == null) {
      measuredLeftPosition = leftTargetPosition;
    } else {
      lastMeasuredLeft = measuredLeftPosition;
    }
    if (measuredRightPosition == null) {
      measuredRightPosition = rightTargetPosition;
    } else {
      lastMeasuredRight = measuredRightPosition;
    }

    // Position correction: add proportional feedback to the feedforward velocities.
    // This compensates for velocity tracking error that would otherwise accumulate as
    // position drift during the path. kP * (target - measured) → corrective velocity.
    // Extend and retract use separate gains since retract has natural damping from robot weight.
    double kP =
        isPulling
            ? ClimbConstants.RETRACT_PATH_POSITION_CORRECTION_KP
            : ClimbConstants.EXTEND_PATH_POSITION_CORRECTION_KP;
    Translation2d leftPosError = leftPosition.minus(measuredLeftPosition);
    Translation2d rightPosError = rightPosition.minus(measuredRightPosition);
    Translation2d correctedLeftVel =
        new Translation2d(
            leftVelocity.getX() + kP * leftPosError.getX(),
            leftVelocity.getY() + kP * leftPosError.getY());
    Translation2d correctedRightVel =
        new Translation2d(
            rightVelocity.getX() + kP * rightPosError.getX(),
            rightVelocity.getY() + kP * rightPosError.getY());

    // Use numerical Jacobian to calculate motor velocities from end effector velocities
    ClimbIK.ClimbVelocityResult velocityResult =
        ClimbIK.calculateVelocityIKBothSides(
            measuredLeftPosition, measuredRightPosition, correctedLeftVel, correctedRightVel);

    if (!velocityResult.isValid()) {
      Logger.recordOutput("Climb/VelocityIK/Valid", false);
      // Fallback to position control if velocity IK fails
      moveToTargetPositions();
      return;
    }

    Logger.recordOutput("Climb/VelocityIK/Valid", true);

    // Clamp each motor to its own max drum velocity (front and back have different gear ratios)
    double leftFrontVel =
        MathUtil.clamp(
            velocityResult.leftSide.frontMotorVelocity,
            -frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY,
            frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY);
    double leftBackVel =
        MathUtil.clamp(
            velocityResult.leftSide.backMotorVelocity,
            -frc.robot.Constants.ClimbConstants.BACK_CRUISE_VELOCITY,
            frc.robot.Constants.ClimbConstants.BACK_CRUISE_VELOCITY);
    double rightFrontVel =
        MathUtil.clamp(
            velocityResult.rightSide.frontMotorVelocity,
            -frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY,
            frc.robot.Constants.ClimbConstants.CRUISE_VELOCITY);
    double rightBackVel =
        MathUtil.clamp(
            velocityResult.rightSide.backMotorVelocity,
            -frc.robot.Constants.ClimbConstants.BACK_CRUISE_VELOCITY,
            frc.robot.Constants.ClimbConstants.BACK_CRUISE_VELOCITY);

    // ── Feedforward: gravity + spring (separate for front/back gear ratios) ──
    // EXTEND: arm-only weight + spring assists motion (subtract spring FF).
    // RETRACT: full robot weight + spring opposes motion (add spring FF).
    double frontGravityVolts =
        isPulling
            ? ClimbConstants.FRONT_RETRACT_GRAVITY_FF_VOLTS
            : ClimbConstants.FRONT_EXTEND_GRAVITY_FF_VOLTS;
    double frontTotalFFVolts =
        isPulling
            ? frontGravityVolts + ClimbConstants.FRONT_SPRING_FF_VOLTS
            : frontGravityVolts - ClimbConstants.FRONT_SPRING_FF_VOLTS;

    double backGravityVolts =
        isPulling
            ? ClimbConstants.BACK_RETRACT_GRAVITY_FF_VOLTS
            : ClimbConstants.BACK_EXTEND_GRAVITY_FF_VOLTS;
    double backTotalFFVolts =
        isPulling
            ? backGravityVolts + ClimbConstants.BACK_SPRING_FF_VOLTS
            : backGravityVolts - ClimbConstants.BACK_SPRING_FF_VOLTS;

    double[] leftFrontBackFF =
        ClimbIK.calculateGravityFeedforward(measuredLeftPosition, frontTotalFFVolts);
    double[] leftBackBackFF =
        ClimbIK.calculateGravityFeedforward(measuredLeftPosition, backTotalFFVolts);
    double[] rightFrontBackFF =
        ClimbIK.calculateGravityFeedforward(measuredRightPosition, frontTotalFFVolts);
    double[] rightBackBackFF =
        ClimbIK.calculateGravityFeedforward(measuredRightPosition, backTotalFFVolts);

    double leftFrontFF = leftFrontBackFF[0];
    double leftBackFF = leftBackBackFF[1];
    double rightFrontFF = rightFrontBackFF[0];
    double rightBackFF = rightBackBackFF[1];

    // Send velocity commands to all 4 motors
    io.setLeftFrontVelocity(leftFrontVel, leftFrontFF);
    io.setLeftBackVelocity(leftBackVel, leftBackFF);
    io.setRightFrontVelocity(rightFrontVel, rightFrontFF);
    io.setRightBackVelocity(rightBackVel, rightBackFF);

    // ── Telemetry ──
    Logger.recordOutput("Climb/VelocityIK/LeftFrontVel", leftFrontVel);
    Logger.recordOutput("Climb/VelocityIK/LeftBackVel", leftBackVel);
    Logger.recordOutput("Climb/VelocityIK/RightFrontVel", rightFrontVel);
    Logger.recordOutput("Climb/VelocityIK/RightBackVel", rightBackVel);

    Logger.recordOutput("Climb/VelocityIK/LeftVelX", leftVelocity.getX());
    Logger.recordOutput("Climb/VelocityIK/LeftVelY", leftVelocity.getY());
    Logger.recordOutput("Climb/VelocityIK/RightVelX", rightVelocity.getX());
    Logger.recordOutput("Climb/VelocityIK/RightVelY", rightVelocity.getY());
    Logger.recordOutput("Climb/VelocityIK/LeftMeasured", measuredLeftPosition);
    Logger.recordOutput("Climb/VelocityIK/RightMeasured", measuredRightPosition);

    Logger.recordOutput("Climb/FF/FrontGravityVolts", frontGravityVolts);
    Logger.recordOutput("Climb/FF/FrontTotalVolts", frontTotalFFVolts);
    Logger.recordOutput("Climb/FF/BackGravityVolts", backGravityVolts);
    Logger.recordOutput("Climb/FF/BackTotalVolts", backTotalFFVolts);
    Logger.recordOutput("Climb/FF/IsPulling", isPulling);
    Logger.recordOutput("Climb/FF/LeftFront", leftFrontFF);
    Logger.recordOutput("Climb/FF/LeftBack", leftBackFF);
    Logger.recordOutput("Climb/FF/RightFront", rightFrontFF);
    Logger.recordOutput("Climb/FF/RightBack", rightBackFF);

    Logger.recordOutput("Climb/PosError/LeftX", leftPosition.getX() - measuredLeftPosition.getX());
    Logger.recordOutput("Climb/PosError/LeftY", leftPosition.getY() - measuredLeftPosition.getY());
    Logger.recordOutput(
        "Climb/PosError/RightX", rightPosition.getX() - measuredRightPosition.getX());
    Logger.recordOutput(
        "Climb/PosError/RightY", rightPosition.getY() - measuredRightPosition.getY());

    // Still calculate IK for position logging
    ClimbIKResult ikResult = ClimbIK.calculateBothSides(leftPosition, rightPosition);
    if (ikResult.isValid()) {
      Logger.recordOutput("Climb/IK/Valid", true);
      Logger.recordOutput("Climb/IK/LeftFrontSetpoint", ikResult.leftSide.frontMotorRotations);
      Logger.recordOutput("Climb/IK/LeftBackSetpoint", ikResult.leftSide.backMotorRotations);
      Logger.recordOutput("Climb/IK/RightFrontSetpoint", ikResult.rightSide.frontMotorRotations);
      Logger.recordOutput("Climb/IK/RightBackSetpoint", ikResult.rightSide.backMotorRotations);
      Logger.recordOutput(
          "Climb/IK/LeftJoint", new double[] {ikResult.leftSide.jointX, ikResult.leftSide.jointY});
      Logger.recordOutput(
          "Climb/IK/RightJoint",
          new double[] {ikResult.rightSide.jointX, ikResult.rightSide.jointY});
    }
  }

  // ===========================================================================
  // IMU CLIMB ASSIST (Auto-Level)
  // ===========================================================================

  /**
   * Set the IMU roll supplier for auto-level during climb. Called from RobotContainer when {@link
   * ClimbConstants.ImuAssist#ENABLED} is true.
   */
  public void setRollDegreesSupplier(DoubleSupplier supplier) {
    this.rollDegreesSupplier = supplier;
  }

  /** Toggle auto-level on/off at runtime (operator override). */
  public void setAutoLevelEnabled(boolean enabled) {
    this.autoLevelEnabled = enabled;
    if (!enabled) {
      autoLevelPID.reset();
    }
  }

  /** Command: toggle auto-level enabled state. */
  public Command toggleAutoLevelCommand() {
    return runOnce(() -> setAutoLevelEnabled(!autoLevelEnabled)).withName("ClimbToggleAutoLevel");
  }

  /**
   * Compute a velocity correction (m/s) to counteract robot tilt during climb. Positive output =
   * speed up left side / slow down right side. Returns 0 when disabled, not pulling, or within
   * deadband.
   */
  @SuppressWarnings("unused") // Feature-flagged off until tuned on real hardware
  private double getAutoLevelVelocityCorrection() {
    if (!ClimbConstants.ImuAssist.ENABLED || !autoLevelEnabled) return 0.0;
    if (!currentState.isPulling()) {
      autoLevelPID.reset();
      return 0.0;
    }

    double rollDeg = rollDegreesSupplier.getAsDouble();
    Logger.recordOutput("Climb/AutoLevel/RollDegrees", rollDeg);

    if (Math.abs(rollDeg) < ClimbConstants.ImuAssist.DEADBAND_DEGREES) {
      autoLevelPID.reset();
      return 0.0;
    }

    double correction = autoLevelPID.calculate(rollDeg, 0.0);
    return MathUtil.clamp(
        correction,
        -ClimbConstants.ImuAssist.MAX_VEL_CORRECTION_MPS,
        ClimbConstants.ImuAssist.MAX_VEL_CORRECTION_MPS);
  }

  /**
   * Apply velocity correction differentially while respecting limits. Clamp-then-bias preserves
   * relative speed difference when one side saturates.
   */
  private double[] applyClampedVelocityCorrection(
      double nominalVelY, double correction, double maxVel) {
    double rawLeft = nominalVelY + correction;
    double rawRight = nominalVelY - correction;

    double clampedLeft = MathUtil.clamp(rawLeft, -maxVel, maxVel);
    double clampedRight = MathUtil.clamp(rawRight, -maxVel, maxVel);

    // If one side was clamped, shift the other to preserve relative difference
    double leftClipAmount = rawLeft - clampedLeft;
    double rightClipAmount = rawRight - clampedRight;

    clampedRight -= leftClipAmount;
    clampedLeft -= rightClipAmount;

    // Final safety clamp
    clampedLeft = MathUtil.clamp(clampedLeft, -maxVel, maxVel);
    clampedRight = MathUtil.clamp(clampedRight, -maxVel, maxVel);

    return new double[] {clampedLeft, clampedRight};
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
    io.setLeftSecondaryHookAnglePosition(ClimbConstants.AngleServo.LEFT_STOWED_POSITION);
    io.setRightSecondaryHookAnglePosition(ClimbConstants.AngleServo.RIGHT_STOWED_POSITION);
    io.setLeftSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.LEFT_STOWED_POSITION);
    io.setRightSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.RIGHT_STOWED_POSITION);
  }

  /** Stow the left side secondary hook servos (angle + hardstop). */
  public void stowLeftServos() {
    io.setLeftSecondaryHookAnglePosition(ClimbConstants.AngleServo.LEFT_STOWED_POSITION);
    io.setLeftSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.LEFT_STOWED_POSITION);
  }

  /** Stow the right side secondary hook servos (angle + hardstop). */
  public void stowRightServos() {
    io.setRightSecondaryHookAnglePosition(ClimbConstants.AngleServo.RIGHT_STOWED_POSITION);
    io.setRightSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.RIGHT_STOWED_POSITION);
  }

  // ─── Release (deploy hooks) ───

  /** Release all 4 secondary hook servos to their deployed position. */
  public void releaseAllServos() {
    io.setLeftSecondaryHookAnglePosition(ClimbConstants.AngleServo.LEFT_RELEASED_POSITION);
    io.setRightSecondaryHookAnglePosition(ClimbConstants.AngleServo.RIGHT_RELEASED_POSITION);
    io.setLeftSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.LEFT_RELEASED_POSITION);
    io.setRightSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.RIGHT_RELEASED_POSITION);
  }

  /** Release the left side secondary hook servos (angle + hardstop). */
  public void releaseLeftServos() {
    io.setLeftSecondaryHookAnglePosition(ClimbConstants.AngleServo.LEFT_RELEASED_POSITION);
    io.setLeftSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.LEFT_RELEASED_POSITION);
  }

  /** Release the right side secondary hook servos (angle + hardstop). */
  public void releaseRightServos() {
    io.setRightSecondaryHookAnglePosition(ClimbConstants.AngleServo.RIGHT_RELEASED_POSITION);
    io.setRightSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.RIGHT_RELEASED_POSITION);
  }

  // ─── Individual angle / hardstop setters ───

  /** Release both angle servos only (no hardstop change). */
  public void releaseAngleServos() {
    io.setLeftSecondaryHookAnglePosition(ClimbConstants.AngleServo.LEFT_RELEASED_POSITION);
    io.setRightSecondaryHookAnglePosition(ClimbConstants.AngleServo.RIGHT_RELEASED_POSITION);
  }

  /** Release both hardstop servos only (no angle change). */
  public void releaseHardstopServos() {
    io.setLeftSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.LEFT_RELEASED_POSITION);
    io.setRightSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.RIGHT_RELEASED_POSITION);
  }

  /** Stow both angle servos only (no hardstop change). */
  public void stowAngleServos() {
    io.setLeftSecondaryHookAnglePosition(ClimbConstants.AngleServo.LEFT_STOWED_POSITION);
    io.setRightSecondaryHookAnglePosition(ClimbConstants.AngleServo.RIGHT_STOWED_POSITION);
  }

  /** Stow both hardstop servos only (no angle change). */
  public void stowHardstopServos() {
    io.setLeftSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.LEFT_STOWED_POSITION);
    io.setRightSecondaryHookHardstopPosition(ClimbConstants.HardstopServo.RIGHT_STOWED_POSITION);
  }

  // ─── Command-returning versions for teleop/auto ───

  /** Command: stow all secondary hook servos. Fire-and-forget (no subsystem requirement). */
  public Command stowAllServosCommand() {
    return Commands.runOnce(this::stowAllServos).withName("ClimbStowAllServos");
  }

  /** Command: release all secondary hook servos. Fire-and-forget (no subsystem requirement). */
  public Command releaseAllServosCommand() {
    return Commands.runOnce(this::releaseAllServos).withName("ClimbReleaseAllServos");
  }

  /** Command: stow left secondary hook servos. Fire-and-forget (no subsystem requirement). */
  public Command stowLeftServosCommand() {
    return Commands.runOnce(this::stowLeftServos).withName("ClimbStowLeftServos");
  }

  /** Command: stow right secondary hook servos. Fire-and-forget (no subsystem requirement). */
  public Command stowRightServosCommand() {
    return Commands.runOnce(this::stowRightServos).withName("ClimbStowRightServos");
  }

  /** Command: release left secondary hook servos. Fire-and-forget (no subsystem requirement). */
  public Command releaseLeftServosCommand() {
    return Commands.runOnce(this::releaseLeftServos).withName("ClimbReleaseLeftServos");
  }

  /** Command: release right secondary hook servos. Fire-and-forget (no subsystem requirement). */
  public Command releaseRightServosCommand() {
    return Commands.runOnce(this::releaseRightServos).withName("ClimbReleaseRightServos");
  }

  /** Command: stow both angle servos (left + right). Fire-and-forget (no subsystem requirement). */
  public Command stowAngleServosCommand() {
    return Commands.runOnce(this::stowAngleServos).withName("ClimbStowAngleServos");
  }

  /**
   * Command: release both angle servos (left + right). Fire-and-forget (no subsystem requirement).
   */
  public Command releaseAngleServosCommand() {
    return Commands.runOnce(this::releaseAngleServos).withName("ClimbReleaseAngleServos");
  }

  /**
   * Command: stow both hardstop servos (left + right). Fire-and-forget (no subsystem requirement).
   */
  public Command stowHardstopServosCommand() {
    return Commands.runOnce(this::stowHardstopServos).withName("ClimbStowHardstopServos");
  }

  /**
   * Command: release both hardstop servos (left + right). Fire-and-forget (no subsystem
   * requirement).
   */
  public Command releaseHardstopServosCommand() {
    return Commands.runOnce(this::releaseHardstopServos).withName("ClimbReleaseHardstopServos");
  }

  /**
   * Command: stow left angle servo only (no hardstop change). Fire-and-forget (no subsystem
   * requirement).
   */
  public Command stowLeftAngleServoCommand() {
    return Commands.runOnce(
            () ->
                io.setLeftSecondaryHookAnglePosition(
                    ClimbConstants.AngleServo.LEFT_STOWED_POSITION))
        .withName("ClimbStowLeftAngleServo");
  }

  /**
   * Command: release left angle servo only (no hardstop change). Fire-and-forget (no subsystem
   * requirement).
   */
  public Command releaseLeftAngleServoCommand() {
    return Commands.runOnce(
            () ->
                io.setLeftSecondaryHookAnglePosition(
                    ClimbConstants.AngleServo.LEFT_RELEASED_POSITION))
        .withName("ClimbReleaseLeftAngleServo");
  }

  /**
   * Command: stow left hardstop servo only (no angle change). Fire-and-forget (no subsystem
   * requirement).
   */
  public Command stowLeftHardstopServoCommand() {
    return Commands.runOnce(
            () ->
                io.setLeftSecondaryHookHardstopPosition(
                    ClimbConstants.HardstopServo.LEFT_STOWED_POSITION))
        .withName("ClimbStowLeftHardstopServo");
  }

  /**
   * Command: release left hardstop servo only (no angle change). Fire-and-forget (no subsystem
   * requirement).
   */
  public Command releaseLeftHardstopServoCommand() {
    return Commands.runOnce(
            () ->
                io.setLeftSecondaryHookHardstopPosition(
                    ClimbConstants.HardstopServo.LEFT_RELEASED_POSITION))
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
   * Advance climb by one operator-triggered step (POV-Right). L1 mode: 2 presses (extend/retract
   * auto L1). L2L3 mode: 11 presses (extend/retract L1-L3 with servo operations between levels).
   */
  public Command nextClimbStep() {
    return Commands.defer(
        () -> {
          // ── L1 mode: auto L1 sequence ──
          if (getClimbLevel() == ClimbLevel.L1) {
            switch (currentState) {
              case STOWED:
                return setStateCommand(ClimbState.EXTEND_L1_AUTO);
              case EXTEND_L1_AUTO:
                return setStateCommand(ClimbState.RETRACT_L1_AUTO);
              default:
                return Commands.none();
            }
          }

          // ── L2L3 mode: full teleop sequence ──
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
   * servo changes or running the reversed path as appropriate. Branches on operator climb level (L1
   * vs L2L3).
   */
  public Command previousClimbStep() {
    return Commands.defer(
        () -> {
          // ── L1 mode: reverse auto L1 sequence (no servos) ──
          if (getClimbLevel() == ClimbLevel.L1) {
            switch (currentState) {
              case EXTEND_L1_AUTO:
                // Direct path back to STOWED — no servos, no getPreviousState dependency
                return stowPathOnly();
              case RETRACT_L1_AUTO:
                // Reverse retract path back to EXTEND_L1_AUTO — path only, no servos
                return releaseFromAutoL1();
              default:
                return Commands.none();
            }
          }

          // ── L2L3 mode: full teleop previous ──
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

  /** Whether the climb is currently in manual operator control mode. */
  public boolean isInManualMode() {
    return manualMode;
  }

  /**
   * Enter manual control mode. Stops any active motion and allows the operator sticks to command
   * end-effector velocities directly.
   */
  public Command enterManualMode() {
    return runOnce(
            () -> {
              // Guard: can't enter manual while in calibration or emergency
              if (calibrationMode || currentState == ClimbState.EMERGENCY_STOP) return;
              manualMode = true;
              currentState = ClimbState.MANUAL;
              io.stop();
              Logger.recordOutput("Climb/ManualMode", true);
              Logger.recordOutput("Climb/CurrentState", currentState.getName());
              System.out.println("[Climb] Entered manual mode");
            })
        .withName("ClimbEnterManual");
  }

  /**
   * Exit manual control mode. Clears manual flag, then follows a path from the current end-effector
   * position back to STOWED (with proper servo stowing). The arm will smoothly retrace its way home
   * rather than snapping instantly.
   */
  public Command exitManualMode() {
    return Commands.sequence(
            runOnce(
                () -> {
                  manualMode = false;
                  io.stop();
                  Logger.recordOutput("Climb/ManualMode", false);
                  System.out.println("[Climb] Exiting manual mode — returning to stow");
                }),
            stowFromCurrentState())
        .withName("ClimbExitManual");
  }

  /**
   * Manual EE velocity control. Integrates vel*dt into target positions each cycle. No-op if not in
   * manual mode.
   */
  public void manualControl(Translation2d leftVel, Translation2d rightVel) {
    if (!manualMode || calibrationMode || currentState == ClimbState.EMERGENCY_STOP) return;

    // Keep state as MANUAL while operator is driving
    currentState = ClimbState.MANUAL;

    // Use measured (FK) positions as the integration base so the target doesn't
    // drift away from reality on the real robot. If FK fails, fall back to the
    // previous target so we don't lose the position entirely.
    Translation2d leftBase =
        ClimbIK.estimateEndEffectorPosition(
            inputs.leftFrontPositionRotations,
            inputs.leftBackPositionRotations,
            leftTargetPosition);
    Translation2d rightBase =
        ClimbIK.estimateEndEffectorPosition(
            inputs.rightFrontPositionRotations,
            inputs.rightBackPositionRotations,
            rightTargetPosition);
    if (leftBase == null) leftBase = leftTargetPosition;
    if (rightBase == null) rightBase = rightTargetPosition;

    // Integrate velocity into candidate positions (20 ms loop)
    final double dt = 0.02;
    Translation2d candidateLeft =
        new Translation2d(
            leftBase.getX() + leftVel.getX() * dt, leftBase.getY() + leftVel.getY() * dt);
    Translation2d candidateRight =
        new Translation2d(
            rightBase.getX() + rightVel.getX() * dt, rightBase.getY() + rightVel.getY() * dt);

    // Clamp to workspace so the operator can't drive the arm out of IK range
    candidateLeft = clampToWorkspace(candidateLeft);
    candidateRight = clampToWorkspace(candidateRight);

    // Validate IK before committing — if the new position is unreachable,
    // reject the move and hold at the last valid position instead of
    // letting the target drift into an unrecoverable invalid region.
    boolean leftValid = ClimbIK.calculateIK(candidateLeft, false).isValid;
    boolean rightValid = ClimbIK.calculateIK(candidateRight, false).isValid;

    if (leftValid) {
      leftTargetPosition = candidateLeft;
    }
    if (rightValid) {
      rightTargetPosition = candidateRight;
    }

    // If both sides are invalid, just hold position
    if (!leftValid && !rightValid) {
      moveToTargetPositions();
      return;
    }

    // Use the (possibly unchanged) target positions with the requested velocities.
    // Zero out velocity for any axis that was rejected to avoid fighting.
    Translation2d effectiveLeftVel = leftValid ? leftVel : Translation2d.kZero;
    Translation2d effectiveRightVel = rightValid ? rightVel : Translation2d.kZero;

    Logger.recordOutput("Climb/Manual/LeftVel", new double[] {leftVel.getX(), leftVel.getY()});
    Logger.recordOutput("Climb/Manual/RightVel", new double[] {rightVel.getX(), rightVel.getY()});
    Logger.recordOutput("Climb/Manual/LeftIKValid", leftValid);
    Logger.recordOutput("Climb/Manual/RightIKValid", rightValid);

    // Use same internal velocity path-following routine (will clamp motor velocities)
    setTargetVelocitiesInternal(
        leftTargetPosition, rightTargetPosition, effectiveLeftVel, effectiveRightVel, false);
  }

  /** Clamp a position to the mechanism workspace limits. */
  private static Translation2d clampToWorkspace(Translation2d pos) {
    double x =
        MathUtil.clamp(
            pos.getX(),
            ClimbConstants.WORKSPACE_MIN_X_METERS,
            ClimbConstants.WORKSPACE_MAX_X_METERS);
    double y =
        MathUtil.clamp(
            pos.getY(),
            ClimbConstants.WORKSPACE_MIN_Y_METERS,
            ClimbConstants.WORKSPACE_MAX_Y_METERS);
    return new Translation2d(x, y);
  }

  /**
   * Enter calibration mode. Stops all motors and allows individual motor voltage control. Normal
   * climb state commands are blocked while in calibration mode.
   */
  public Command enterCalibrationMode() {
    return runOnce(
            () -> {
              calibrationMode = true;
              currentState = ClimbState.CALIBRATION;
              io.stop();
              io.setCalibrationCurrentLimits();
              Logger.recordOutput("Climb/CalibrationMode", true);
              Logger.recordOutput("Climb/CurrentState", currentState.getName());
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
              io.setNormalCurrentLimits();
              io.recalibrateEncoders();
              calibrationMode = false;
              setState(ClimbState.STOWED);
              initializeMechanism2dToStowed();
              // Log stowed values so outputs don't freeze at calibration values
              // (periodic skips visualization when stowed + not in calibration/manual)
              Translation2d stowedPos = ClimbState.STOWED.getTargetPosition();
              Logger.recordOutput("Climb/LeftTargetPosition", stowedPos);
              Logger.recordOutput("Climb/RightTargetPosition", stowedPos);
              Logger.recordOutput("Climb/LeftMeasuredPosition", stowedPos);
              Logger.recordOutput("Climb/RightMeasuredPosition", stowedPos);
              ClimbIK.ClimbSideIKResult stowedIK = ClimbIK.calculateIK(stowedPos);
              double stowedFrontRot = stowedIK.isValid ? stowedIK.frontMotorRotations : 0.0;
              double stowedBackRot = stowedIK.isValid ? stowedIK.backMotorRotations : 0.0;
              Logger.recordOutput("Climb/LeftFrontRotations", stowedFrontRot);
              Logger.recordOutput("Climb/LeftBackRotations", stowedBackRot);
              Logger.recordOutput("Climb/RightFrontRotations", stowedFrontRot);
              Logger.recordOutput("Climb/RightBackRotations", stowedBackRot);
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
