package frc.robot.subsystems.intakepivot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.util.IntakePivotFF;
import org.littletonrobotics.junction.Logger;

public class IntakePivotSubsystem extends SubsystemBase {

  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  // --- Mechanism2d visualization ---
  // Shows the full intake pivot linkage: pivot O, V-link arm (O->E), coupler link (E->A),
  // rack endpoint A, and a COM marker. Canvas is sized to contain the full workspace.
  private static final double CANVAS_W = 0.6; // meters
  private static final double CANVAS_H = 0.6; // meters
  // Offset so pivot O appears near canvas center
  private static final double CANVAS_OX = 0.3;
  private static final double CANVAS_OY = 0.3;

  private final Mechanism2d mechanism;
  private final MechanismLigament2d vlinkArm; // O -> E (elbow radius, rotates with alpha)
  private final MechanismLigament2d couplerLink; // E -> A (fixed length, angle computed)
  private final MechanismLigament2d comArm; // O -> COM (shows mass location)
  private final MechanismLigament2d rackLine; // A0 -> A (shows rack displacement)

  // FF calculator used for geometry queries in Mechanism2d (not for motor control)
  private final IntakePivotFF ff;

  /** Constructs an {@link IntakePivotSubsystem} subsystem instance */
  public IntakePivotSubsystem(IntakePivotIO io) {
    this.io = io;

    // Build a local FF calculator purely for geometry/visualization queries.
    // The IOTalonFX layer has its own instance for motor feedforward.
    ff = new IntakePivotFF(buildParamsFromConstants());
    IntakePivotFF.Params p = ff.getParams();

    // ---- Build Mechanism2d ----
    mechanism = new Mechanism2d(CANVAS_W, CANVAS_H);

    // Fixed pivot O marker (white dot)
    MechanismRoot2d pivotRoot = mechanism.getRoot("Pivot_O", CANVAS_OX, CANVAS_OY);
    pivotRoot.append(new MechanismLigament2d("PivotDot", 0.01, 0, 6, new Color8Bit(Color.kWhite)));

    // V-link arm: O -> E (blue, rotates with alpha)
    vlinkArm =
        pivotRoot.append(
            new MechanismLigament2d(
                "VlinkArm", p.elbowRadius_m, 0, 5, new Color8Bit(Color.kDodgerBlue)));

    // Coupler link: E -> A (orange, appended to end of V-link arm)
    couplerLink =
        vlinkArm.append(
            new MechanismLigament2d(
                "CouplerLink", p.couplerLength_m, 0, 3, new Color8Bit(Color.kOrange)));

    // COM arm: O -> COM (red, separate root at same pivot point)
    MechanismRoot2d comRoot = mechanism.getRoot("COM_O", CANVAS_OX, CANVAS_OY);
    comArm =
        comRoot.append(
            new MechanismLigament2d("COMarker", p.comRadius_m, 0, 4, new Color8Bit(Color.kRed)));

    // Rack line: from pivot root, drawn as a vector toward rack attachment A
    MechanismRoot2d rackRoot = mechanism.getRoot("RackA0", CANVAS_OX, CANVAS_OY);
    rackLine =
        rackRoot.append(
            new MechanismLigament2d("RackLine", 0.0, 0, 2, new Color8Bit(Color.kLimeGreen)));

    // Publish the Mechanism2d structure once -- auto-updates from mutated ligaments each cycle
    SmartDashboard.putData("IntakePivot/Mechanism2d", mechanism);
  }

  /** Build IntakePivotFF.Params from Constants. Shared helper for subsystem + IOTalonFX. */
  private static IntakePivotFF.Params buildParamsFromConstants() {
    IntakePivotFF.Params p = new IntakePivotFF.Params();
    p.pinionRadius_m = IntakePivotConstants.FF_PINION_RADIUS_M;
    p.rackGearRatio = IntakePivotConstants.FF_RACK_GEAR_RATIO;
    p.rackTheta_rad = IntakePivotConstants.FF_RACK_THETA_RAD;
    p.A0x_m = IntakePivotConstants.FF_A0X_M;
    p.A0y_m = IntakePivotConstants.FF_A0Y_M;
    p.Ox_m = IntakePivotConstants.FF_OX_M;
    p.Oy_m = IntakePivotConstants.FF_OY_M;
    p.elbowRadius_m = IntakePivotConstants.FF_ELBOW_RADIUS_M;
    p.couplerLength_m = IntakePivotConstants.FF_COUPLER_LENGTH_M;
    p.mass_kg = IntakePivotConstants.FF_MASS_KG;
    p.comRadius_m = IntakePivotConstants.FF_COM_RADIUS_M;
    p.comAngleOffset_rad = IntakePivotConstants.FF_COM_ANGLE_OFFSET_RAD;
    p.motorKt_NmPerA = IntakePivotConstants.FF_MOTOR_KT;
    p.motorR_ohm = IntakePivotConstants.FF_MOTOR_R_OHM;
    p.efficiency = IntakePivotConstants.FF_EFFICIENCY;
    return p;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    // Update Mechanism2d visualization
    updateMechanism2d();
  }

  /**
   * Update the Mechanism2d ligament angles and lengths from the current motor position. Uses the
   * IntakePivotFF linkage solver to compute the full kinematic state from motor rotations.
   */
  private void updateMechanism2d() {
    IntakePivotFF.LinkageState state = ff.computeLinkageState(inputs.positionRotations);
    IntakePivotFF.Params p = ff.getParams();

    // V-link arm: from O to E, absolute angle in degrees
    double vlinkAngleDeg = Math.toDegrees(state.alpha_rad);
    vlinkArm.setAngle(vlinkAngleDeg);

    // Coupler link: from E to A, compute the angle relative to V-link arm
    double couplerAbsAngleDeg =
        Math.toDegrees(Math.atan2(state.Ay - state.Ey, state.Ax - state.Ex));
    couplerLink.setAngle(couplerAbsAngleDeg - vlinkAngleDeg);

    // COM arm: from O to COM, absolute angle in degrees
    double comAngleDeg = Math.toDegrees(state.alpha_rad + p.comAngleOffset_rad);
    comArm.setAngle(comAngleDeg);

    // Rack line: from pivot O to rack attachment A (drawn as a line for visualization)
    double rackDx = state.Ax - state.Ox;
    double rackDy = state.Ay - state.Oy;
    rackLine.setLength(Math.hypot(rackDx, rackDy));
    rackLine.setAngle(Math.toDegrees(Math.atan2(rackDy, rackDx)));

    // Log the linkage geometry for AdvantageScope
    Logger.recordOutput("IntakePivotFF/alpha_deg", vlinkAngleDeg);
    Logger.recordOutput("IntakePivotFF/rackS_m", state.rackS_m);
    // Mechanism2d auto-updates via SmartDashboard.putData in constructor
  }

  /**
   * Command to deploy/extend the intake (lower it down)
   *
   * @return A command that extends the intake pivot
   */
  public Command deploy() {
    return runOnce(
            () -> {
              io.setPosition(IntakePivotConstants.DEPLOYED_POSITION);
            })
        .withName("IntakePivotDeploy");
  }

  /**
   * Command to stow the intake (raise it up)
   *
   * @return A command that stows the intake pivot
   */
  public Command stow() {
    return runOnce(
            () -> {
              io.setPosition(IntakePivotConstants.STOWED_POSITION);
            })
        .withName("IntakePivotStow");
  }

  /** Get current pivot position in rotations */
  public double getPosition() {
    return inputs.positionRotations;
  }

  /** Check if pivot is at target position */
  public boolean atPosition(double targetPosition) {
    return Math.abs(inputs.positionRotations - targetPosition)
        < IntakePivotConstants.POSITION_TOLERANCE;
  }

  /** Check if intake is deployed */
  public boolean isDeployed() {
    return atPosition(IntakePivotConstants.DEPLOYED_POSITION);
  }

  /** Check if intake is stowed */
  public boolean isStowed() {
    return atPosition(IntakePivotConstants.STOWED_POSITION);
  }

  /** Immediately stop the pivot motor */
  public void stopMotor() {
    io.stop();
  }

  /**
   * Directly apply the deployed position. Called by Superstructure.periodic() for states that need
   * intake deployed. Unlike the deploy() command, this is a plain void method.
   */
  public void applyDeploy() {
    io.setPosition(IntakePivotConstants.DEPLOYED_POSITION);
  }

  /**
   * Directly apply the half-deployed position. Used for intake shake — oscillates between half and
   * full deployed to dislodge stuck fuel. This is a plain void method for Superstructure use.
   */
  public void applyHalfDeploy() {
    io.setPosition(IntakePivotConstants.HALF_DEPLOYED_POSITION);
  }

  /** Check if intake is at the half-deployed position */
  public boolean isHalfDeployed() {
    return atPosition(IntakePivotConstants.HALF_DEPLOYED_POSITION);
  }

  /**
   * Directly apply the stowed position. Called by Superstructure.periodic() for states that need
   * intake stowed. Unlike the stow() command, this is a plain void method.
   */
  public void applyStow() {
    io.setPosition(IntakePivotConstants.STOWED_POSITION);
  }
}
