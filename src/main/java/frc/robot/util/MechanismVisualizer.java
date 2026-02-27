package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.MechanismVisualization;
import org.littletonrobotics.junction.Logger;

/**
 * Computes and logs 3D component poses for AdvantageScope's articulated robot visualization.
 *
 * <p>AdvantageScope uses two arrays of {@link Pose3d} to render articulated components on the 3D
 * field:
 *
 * <ul>
 *   <li><b>ZeroedComponentPoses</b> — an array of identity (all-zero) poses, one per component.
 *       Used during the AdvantageScope calibration step (step 4 in the setup process) to align each
 *       component model to the robot origin. Log this once at startup or every cycle (it never
 *       changes).
 *   <li><b>ComponentPoses</b> — the real robot-relative pose for each component, updated every
 *       cycle. AdvantageScope applies the config's {@code zeroedRotations}/{@code zeroedPosition}
 *       transforms first (from the robot's config.json), then applies these poses to place each
 *       component on the robot.
 * </ul>
 *
 * <p>Component index mapping (must match the robot model's config.json "components" array order):
 *
 * <ul>
 *   <li>0 = Turret (yaw)
 *   <li>1 = Hood (pitch, rides on turret)
 *   <li>2 = Intake Pivot (pitch, on rear of frame)
 * </ul>
 */
public class MechanismVisualizer {

  private static final int N = MechanismVisualization.NUM_COMPONENTS;

  /** All-zeros poses for calibration — never changes. */
  private final Pose3d[] zeroedPoses;

  /** Live robot-relative component poses — updated every cycle. */
  private final Pose3d[] componentPoses;

  public MechanismVisualizer() {
    zeroedPoses = new Pose3d[N];
    componentPoses = new Pose3d[N];
    for (int i = 0; i < N; i++) {
      zeroedPoses[i] = new Pose3d();
      componentPoses[i] = new Pose3d();
    }
  }

  /**
   * Recompute component poses from current mechanism positions and log everything.
   *
   * @param turretAngleRad turret yaw in radians (0 = forward, CCW-positive from above)
   * @param hoodAngleRad hood pitch in radians from horizontal (positive = tilted up)
   * @param intakePivotRotations intake pivot position in motor rotations (0 = stowed, deployed =
   *     full travel)
   */
  public void update(double turretAngleRad, double hoodAngleRad, double intakePivotRotations) {

    // --- Component 0: Turret ---
    // Turret rotates around the Z axis at its fixed mounting point on the frame.
    componentPoses[0] =
        new Pose3d(
            new Translation3d(
                MechanismVisualization.TURRET_X_M,
                MechanismVisualization.TURRET_Y_M,
                MechanismVisualization.TURRET_HEIGHT_M),
            new Rotation3d(0, 0, turretAngleRad));

    // --- Component 1: Hood ---
    // The hood pitches on the turret. Its position is turret-relative, so we compose:
    //   turret translation + turret yaw rotation applied to hood's local offset.
    // Hood pitches around the robot-Y axis (negative pitch = nose up in WPILib convention,
    // but hood angle from horizontal is positive-up, so negate for Rotation3d pitch).
    double hoodX = MechanismVisualization.HOOD_X_FROM_TURRET_M;
    double hoodZ = MechanismVisualization.HOOD_Z_ABOVE_TURRET_M;

    // Rotate the hood's turret-local offset by the turret yaw
    double cosYaw = Math.cos(turretAngleRad);
    double sinYaw = Math.sin(turretAngleRad);
    double hoodWorldX = MechanismVisualization.TURRET_X_M + (hoodX * cosYaw);
    double hoodWorldY = MechanismVisualization.TURRET_Y_M + (hoodX * sinYaw);
    double hoodWorldZ = MechanismVisualization.TURRET_HEIGHT_M + hoodZ;

    componentPoses[1] =
        new Pose3d(
            new Translation3d(hoodWorldX, hoodWorldY, hoodWorldZ),
            // Yaw follows the turret, pitch is the hood angle + viz offset
            new Rotation3d(
                0, hoodAngleRad + MechanismVisualization.HOOD_PITCH_OFFSET_RAD, turretAngleRad));

    // --- Component 2: Intake Pivot ---
    // Stowed = 90° pitch (pointing straight up), deployed = 0° (horizontal).
    // Convert motor rotations → angle: full travel sweeps from π/2 down to 0.
    double pivotFraction =
        intakePivotRotations / MechanismVisualization.ROTATIONS_PER_90_DEG; // 0..1
    double pivotAngleRad =
        (Math.PI / 2.0) - pivotFraction * (Math.PI / 2.0); // π/2 = stowed (up), 0 = deployed

    componentPoses[2] =
        new Pose3d(
            new Translation3d(
                MechanismVisualization.INTAKE_PIVOT_X_M,
                MechanismVisualization.INTAKE_PIVOT_Y_M,
                MechanismVisualization.INTAKE_PIVOT_HEIGHT_M),
            // Intake pivots around Y axis: π/2 = stowed (up), 0 = deployed (horizontal)
            new Rotation3d(0, pivotAngleRad, 0));

    // --- Log to AdvantageScope ---
    // "ZeroedComponentPoses" is used during the calibration process (step 4).
    // "ComponentPoses" is the live visualization — add as a "Component" child
    //   to the robot object in the 3D Field tab.
    Logger.recordOutput("Mechanism3d/ZeroedComponentPoses", zeroedPoses);
    Logger.recordOutput("Mechanism3d/ComponentPoses", componentPoses);
  }
}
