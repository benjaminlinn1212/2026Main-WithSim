package frc.robot.subsystems.climb.util;

import frc.robot.Constants.ClimbConstants;

/**
 * Step-function cable drum model. Cable stacks in layers on the drum; each full layer ({@link
 * ClimbConstants#ROTATIONS_PER_LAYER} rotations) increases the effective circumference by {@link
 * ClimbConstants#CIRCUMFERENCE_PER_LAYER_METERS}. All rotations are relative to bare drum; stow has
 * {@link ClimbConstants#STOW_ABSOLUTE_ROTATIONS} already wound. Positive rotation = release cable.
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

  /** Total cable length wound at {@code absRot} rotations. Sums full layers + partial layer. */
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

  /** Inverse of {@link #cableLengthAtAbsoluteRotation}: cable length → absolute rotations. */
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
   * Convert cable length delta from stow (m) to drum rotations from stow. Positive delta (longer
   * cable) → positive rotations (release).
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
   * Convert drum rotations from stow to cable length delta (m). Positive rotations (release) →
   * positive delta (longer cable).
   */
  public static double rotationsToCableDelta(double drumRotationsFromStow) {
    // absoluteRot = STOW_ABS − relativeRot
    double absRot = STOW_ABS - drumRotationsFromStow;
    if (absRot < 0) absRot = 0;

    double absCable = cableLengthAtAbsoluteRotation(absRot);
    // delta = stowCable − absCable  (released means less on drum = positive delta)
    return STOW_CABLE_LENGTH - absCable;
  }

  /** Effective circumference (m/rot) at the given drum rotation from stow. */
  public static double circumferenceAtRotation(double drumRotationsFromStow) {
    double absRot = STOW_ABS - drumRotationsFromStow;
    if (absRot < 0) absRot = 0;
    return circumferenceAtAbsoluteRotation(absRot);
  }
}
