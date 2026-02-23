package frc.robot.subsystems.climb.util;

import frc.robot.Constants.ClimbConstants;

/**
 * =========================================================================== Cable Drum Layer
 * Model — Step-Function Circumference
 * ===========================================================================
 *
 * <p>As cable winds onto the drum, it stacks in layers. Each full layer is {@link
 * ClimbConstants#ROTATIONS_PER_LAYER} rotations. When a layer is complete, the effective winding
 * circumference jumps by {@link ClimbConstants#CIRCUMFERENCE_PER_LAYER_METERS} because the next
 * layer wraps around the previous cable.
 *
 * <h3>Reference Point</h3>
 *
 * All rotations are expressed relative to the <b>bare drum</b> (zero cable wound). At the STOW pose
 * the drum has {@link ClimbConstants#STOW_ABSOLUTE_ROTATIONS} of cable already wound on. IK/FK
 * works in <i>rotations relative to stow</i>, so we convert:
 *
 * <pre>
 *   absoluteRot = stowAbsoluteRot − relativeRot
 *       (retracting = negative relative = more cable wound = larger absolute)
 *       (releasing  = positive relative = less cable wound = smaller absolute)
 * </pre>
 *
 * <h3>Sign Convention (matches ClimbIK)</h3>
 *
 * <ul>
 *   <li>Positive drum rotation = <b>release</b> cable (cable gets longer, delta &gt; 0)
 *   <li>Negative drum rotation = <b>retract</b> cable (cable gets shorter, delta &lt; 0)
 * </ul>
 *
 * <h3>Step-Function Model</h3>
 *
 * Within a single layer the circumference is constant. The cable length gained or lost during
 * rotations {@code n1..n2} that all fall within one layer is simply {@code (n2 − n1) ×
 * circumferenceOfThatLayer}. Transitions across layer boundaries are accumulated layer by layer.
 */
public class CableDrumModel {

  // ── Shorthand constants ──────────────────────────────────────────────────
  private static final double C0 = ClimbConstants.CABLE_DRUM_CIRCUMFERENCE_METERS;
  private static final double DC = ClimbConstants.CIRCUMFERENCE_PER_LAYER_METERS;
  private static final double RPL = ClimbConstants.ROTATIONS_PER_LAYER;
  private static final double STOW_ABS = ClimbConstants.STOW_ABSOLUTE_ROTATIONS;

  // ── Helpers ──────────────────────────────────────────────────────────────

  /** Circumference for the layer that contains absolute rotation {@code absRot}. */
  private static double circumferenceAtAbsoluteRotation(double absRot) {
    if (absRot < 0) return C0; // below bare drum — shouldn't happen, clamp
    int layer = (int) (absRot / RPL); // integer layer index (0-based)
    return C0 + layer * DC;
  }

  /**
   * Total cable length wound on the drum when exactly {@code absRot} rotations of cable are stored
   * (measured from bare drum = 0).
   *
   * <p>Computed by summing full layers, then adding the partial layer.
   */
  private static double cableLengthAtAbsoluteRotation(double absRot) {
    if (absRot <= 0) return 0.0;

    int fullLayers = (int) (absRot / RPL);
    double remainder = absRot - fullLayers * RPL;

    double length = 0.0;
    for (int i = 0; i < fullLayers; i++) {
      length += RPL * (C0 + i * DC); // each full layer
    }
    // partial layer
    length += remainder * (C0 + fullLayers * DC);
    return length;
  }

  /**
   * Inverse of {@link #cableLengthAtAbsoluteRotation}: given a total cable length wound on the
   * drum, return how many absolute rotations that corresponds to.
   */
  private static double absoluteRotationAtCableLength(double cableLength) {
    if (cableLength <= 0) return 0.0;

    double remaining = cableLength;
    int layer = 0;

    while (true) {
      double layerCirc = C0 + layer * DC;
      double layerCapacity = RPL * layerCirc;
      if (remaining <= layerCapacity + 1e-12) {
        // Within this layer
        return layer * RPL + remaining / layerCirc;
      }
      remaining -= layerCapacity;
      layer++;
      // Safety: prevent infinite loop if something is wildly off
      if (layer > 100) return layer * RPL;
    }
  }

  // ── Public API (stow-relative) ─────────────────────────────────────────

  /** Cable length stored on the drum at the STOW position (absolute, meters). */
  private static final double STOW_CABLE_LENGTH = cableLengthAtAbsoluteRotation(STOW_ABS);

  /**
   * Convert a cable length <b>delta</b> from stow (meters) to drum rotations from stow.
   *
   * <ul>
   *   <li>{@code deltaCable > 0} → cable got longer → drum released → positive rotations
   *   <li>{@code deltaCable < 0} → cable got shorter → drum retracted → negative rotations
   * </ul>
   *
   * @param deltaCableMeters Change in cable length from stow (meters)
   * @return Drum (mechanism) rotations relative to stow position
   */
  public static double cableDeltaToRotations(double deltaCableMeters) {
    // Target absolute cable on drum = stow cable − delta
    //   (releasing makes cable longer but drum has LESS cable stored)
    double targetAbsCable = STOW_CABLE_LENGTH - deltaCableMeters;
    if (targetAbsCable < 0) targetAbsCable = 0; // can't have negative cable on drum

    double targetAbsRot = absoluteRotationAtCableLength(targetAbsCable);

    // Relative rotation from stow:
    //   relativeRot = -(targetAbsRot - STOW_ABS)
    //   because retract (more wound) = more absRot = negative relative
    return -(targetAbsRot - STOW_ABS);
  }

  /**
   * Convert drum rotations from stow to a cable length <b>delta</b> (meters).
   *
   * <ul>
   *   <li>Positive rotations → cable released → positive delta
   *   <li>Negative rotations → cable retracted → negative delta
   * </ul>
   *
   * @param drumRotationsFromStow Drum rotations relative to stow (positive = release)
   * @return Change in cable length from stow (meters)
   */
  public static double rotationsToCableDelta(double drumRotationsFromStow) {
    // absoluteRot = STOW_ABS − relativeRot
    double absRot = STOW_ABS - drumRotationsFromStow;
    if (absRot < 0) absRot = 0;

    double absCable = cableLengthAtAbsoluteRotation(absRot);
    // delta = stowCable − absCable  (released means less on drum = positive delta)
    return STOW_CABLE_LENGTH - absCable;
  }

  /**
   * Effective circumference at the current drum rotation from stow. Useful for Jacobian scaling if
   * needed.
   *
   * @param drumRotationsFromStow Drum rotations relative to stow
   * @return Effective circumference (meters per rotation) at that position
   */
  public static double circumferenceAtRotation(double drumRotationsFromStow) {
    double absRot = STOW_ABS - drumRotationsFromStow;
    if (absRot < 0) absRot = 0;
    return circumferenceAtAbsoluteRotation(absRot);
  }
}
