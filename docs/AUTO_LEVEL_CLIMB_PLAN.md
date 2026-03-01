# Auto-Level Climb Design Plan

**Team 10922 — Amped (REBUILT 2026)**
**Status:** Implemented — Feature-flagged off (`ClimbConstants.ImuAssist.ENABLED = false`)

---

## Problem Statement

When climbing, the robot can tilt/rock side-to-side due to uneven cable tension, friction differences between left and right arms, or bar flex. The goal is to use the onboard IMU (Pigeon2) to detect tilt and actively correct it by differentially adjusting left vs. right cable **velocities** during the RETRACT phases of the climb.

Both arms must reach the **same final position** — the bar is symmetric. The correction must happen **during** the path, not by shifting the end targets apart.

## Architecture Overview

```
IMU (roll angle) → PID Controller → velocity correction (m/s)
                                          ↓
PathExecutor trajectory velocity ──→ clamp & bias ──→ left/right end-effector velocities
                                          ↓
                              Jacobian velocity IK → motor velocities → TalonFX
```

The auto-level system **only activates during RETRACT states** (`isPulling() == true`) where the robot is hanging and tilt matters. During EXTEND states the robot is on the ground, so leveling is irrelevant.

## Approach: Differential Velocity Correction

### Core Idea
1. Read the robot's **roll angle** from the IMU every cycle (20ms)
2. Run a **PID controller** (P or PD) that outputs a **velocity correction** in m/s
3. Apply the correction as a **relative speed difference** between left and right
4. **Clamp-then-bias** so neither side exceeds path or motor velocity limits
5. Positions stay identical for both sides — only the speed to get there differs

### Why Velocity, Not Position?
- **Position offset** would cause the two arms to finish at different (x, y) targets — wrong, both hooks must end at the same pose
- **Position offset** is sluggish during dynamic motion — the correction chases the tilt rather than preventing it
- The root cause of tilt is a **speed mismatch** (one side pulling faster due to friction, cable stretch, etc.), so the fix belongs at the velocity level
- When roll returns to 0°, the correction goes to zero and both sides track the nominal trajectory — no residual drift

### Why Roll?
The climb arms are symmetric left/right. If one side pulls faster than the other, the robot tilts about its longitudinal (forward) axis — that's roll. Pitch (tilt forward/back) is controlled by the front/back cable ratio within each arm, which is already handled by ClimbIK waypoints.

### Velocity Clamping Strategy

The correction must not push either side's velocity beyond the path or motor limits. Instead of a symmetric `+offset / −offset`, we use **clamp-then-bias**:

```
nominalVelY = trajectory velocity (same for both sides)
correction  = PID output (m/s, positive = speed up left / slow down right)

// Raw desired velocities
rawLeftVelY  = nominalVelY + correction
rawRightVelY = nominalVelY - correction

// Clamp both to [−PATH_MAX_VELOCITY, +PATH_MAX_VELOCITY]
clampedLeftVelY  = clamp(rawLeftVelY,  −max, +max)
clampedRightVelY = clamp(rawRightVelY, −max, +max)

// Preserve relative speed difference: if one side saturated, shift the other
// instead of losing the correction entirely.
// Example: if nominalVelY = −0.4 m/s (pulling down), max = 0.45 m/s, correction = 0.08:
//   rawLeft  = −0.32    rawRight = −0.48 → clamped to −0.45
//   Lost 0.03 on right side. Shift left by the same amount:
//   finalLeft = −0.32 + 0.03 = −0.29   finalRight = −0.45
//   Relative difference preserved (0.16 m/s), both within limits.
```

This ensures:
- Neither side exceeds `PATH_MAX_VELOCITY_MPS` or `CRUISE_VELOCITY` (motor limit)
- The *relative* speed difference is maintained as much as physically possible
- The correction degrades gracefully at the limits instead of being clipped asymmetrically

## Implementation Plan

### 1. Feature Flag
`ClimbConstants.ImuAssist.ENABLED` — a compile-time constant that gates all auto-level behavior. When `false`, no supplier is wired, no velocity modification occurs.

### 2. IMU Roll Access
- The Pigeon2 is already on the drivetrain (CTRE swerve). Access via `drive.getDriveIO().getPigeon2().getRoll()`.
- A `DoubleSupplier` is wired to ClimbSubsystem in RobotContainer: `climb.setRollDegreesSupplier(...)`.
- **Sign convention:** Positive roll = robot tilting toward the LEFT (looking from behind). Verify on real hardware and flip sign if needed.

### 3. Constants — `ClimbConstants.ImuAssist`
Nested class in `ClimbConstants` (consistent with `AngleServo`/`HardstopServo` pattern):
```java
public static class ImuAssist {
    public static final boolean ENABLED = false;
    public static final double KP = 0.003;                    // m/s per degree of roll
    public static final double KI = 0.0;
    public static final double KD = 0.001;                    // dampen oscillation
    public static final double MAX_VEL_CORRECTION_MPS = 0.015; // max ±15mm/s (~33% of path speed)
    public static final double DEADBAND_DEGREES = 1.0;        // ignore roll below this
}
```

### 4. ClimbSubsystem Changes

**New fields:**
```java
private DoubleSupplier rollDegreesSupplier = () -> 0.0;
private boolean autoLevelEnabled = ClimbConstants.ImuAssist.ENABLED;
private final PIDController autoLevelPID = new PIDController(
    ClimbConstants.ImuAssist.KP,
    ClimbConstants.ImuAssist.KI,
    ClimbConstants.ImuAssist.KD);
```

**New method — velocity correction from IMU roll:**
```java
private double getAutoLevelVelocityCorrection() {
    if (!autoLevelEnabled || !ClimbConstants.ImuAssist.ENABLED) return 0.0;
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
    return MathUtil.clamp(correction,
        -ClimbConstants.ImuAssist.MAX_VEL_CORRECTION_MPS,
        ClimbConstants.ImuAssist.MAX_VEL_CORRECTION_MPS);
}
```

**New method — clamp-then-bias to preserve relative speed difference:**
```java
/**
 * Apply a velocity correction differentially while respecting velocity limits.
 * Returns {leftVelY, rightVelY} with the relative speed difference preserved
 * even when one side saturates at the limit.
 *
 * @param nominalVelY  Base trajectory Y velocity (m/s), same for both sides
 * @param correction   Desired speed difference (m/s). Positive = left faster.
 * @param maxVel       Maximum allowed Y velocity magnitude (m/s)
 */
private double[] applyClampedVelocityCorrection(
        double nominalVelY, double correction, double maxVel) {
    double rawLeft  = nominalVelY + correction;
    double rawRight = nominalVelY - correction;

    double clampedLeft  = MathUtil.clamp(rawLeft,  -maxVel, maxVel);
    double clampedRight = MathUtil.clamp(rawRight, -maxVel, maxVel);

    // If one side was clamped, shift the other to preserve relative difference
    double leftClipAmount  = rawLeft  - clampedLeft;   // positive if left was over max
    double rightClipAmount = rawRight - clampedRight;   // positive if right was over max

    // Shift the opposite side by the clip amount (subtracting, since the clip
    // means we lost speed on one side — compensate on the other)
    clampedRight -= leftClipAmount;
    clampedLeft  -= rightClipAmount;

    // Final safety clamp
    clampedLeft  = MathUtil.clamp(clampedLeft,  -maxVel, maxVel);
    clampedRight = MathUtil.clamp(clampedRight, -maxVel, maxVel);

    return new double[] { clampedLeft, clampedRight };
}
```

**Injection in `runPath` command's `execute()`:**
```java
@Override
public void execute() {
    if (executor != null) {
        Translation2d[] targets = executor.getCurrentTargets();
        Translation2d[] velocities = executor.getCurrentVelocities();

        // ── IMU auto-level: adjust Y velocities differentially ──
        double velCorrection = getAutoLevelVelocityCorrection();
        if (velCorrection != 0.0 && params.isPulling()) {
            double maxVel = ClimbConstants.PATH_MAX_VELOCITY_MPS;
            double[] adjustedY = applyClampedVelocityCorrection(
                velocities[0].getY(), velCorrection, maxVel);
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
```

Note: positions (`targets[0]`, `targets[1]`) are **never modified** — both sides always target the same pose.

### 5. Wiring in RobotContainer
```java
// Only wire the IMU supplier if the feature is enabled
if (Constants.ClimbConstants.ImuAssist.ENABLED) {
    climb.setRollDegreesSupplier(
        () -> drive.getDriveIO().getPigeon2().getRoll().getValueAsDouble());
}
```

### 6. Operator Controls
- **Toggle auto-level:** `climb.toggleAutoLevelCommand()` — bind to an unused operator button
- Only functional when `ImuAssist.ENABLED` is `true`

### 7. Logging
```java
Logger.recordOutput("Climb/AutoLevel/Enabled", autoLevelEnabled);
Logger.recordOutput("Climb/AutoLevel/RollDegrees", rollDeg);
Logger.recordOutput("Climb/AutoLevel/VelCorrectionMPS", velCorrection);
Logger.recordOutput("Climb/AutoLevel/LeftAdjustedVelY", adjustedLeftVelY);
Logger.recordOutput("Climb/AutoLevel/RightAdjustedVelY", adjustedRightVelY);
```

## Safety Considerations

1. **Feature flag:** `ImuAssist.ENABLED = false` by default — no auto-level behavior until explicitly enabled and tuned on real hardware.
2. **Max velocity correction clamp:** `ImuAssist.MAX_VEL_CORRECTION_MPS` (±30mm/s) bounds the PID output before it reaches the clamp-then-bias logic.
3. **Clamp-then-bias:** Neither side's velocity can exceed `PATH_MAX_VELOCITY_MPS`. The relative speed difference degrades gracefully at the limits.
4. **Motor limits respected:** After Jacobian conversion, motor velocities are still clamped to `CRUISE_VELOCITY` in `setTargetVelocitiesInternal()` (existing code, unchanged).
5. **Only during RETRACT:** Auto-level is disabled during EXTEND (robot on ground) and servo-only states.
6. **Deadband:** The 1.0° deadband prevents constant micro-corrections that waste energy and cause vibration.
7. **Operator override:** Auto-level can be toggled off at any time via `toggleAutoLevelCommand()`.
8. **Emergency stop:** The existing `EMERGENCY_STOP` state bypasses auto-level (stops all motors).
9. **Same final position:** Both arms always target the same end-effector position — no risk of asymmetric final pose.

## Tuning Procedure

1. **Verify roll sign convention:** Tilt robot left by hand, confirm roll reads positive (or negative — flip sign in the velocity correction accordingly).
2. **Enable feature:** Set `ImuAssist.ENABLED = true`, deploy.
3. **Start with P-only:** Set KP = 0.002, KD = 0. Climb L1, observe if tilt is reduced.
4. **Increase KP** until tilt is corrected within ~0.5s during retract. Typical range: 0.003–0.01.
5. **Add D-term if needed:** If the robot oscillates side-to-side, add KD ≈ 0.001 to dampen.
6. **Test max correction:** Deliberately introduce tilt (shim one side). Verify the correction saturates cleanly at `ImuAssist.MAX_VEL_CORRECTION_MPS` without jerking.
7. **Verify final pose:** After retract completes, confirm both sides hold the same position (check `Climb/LeftTargetPosition` == `Climb/RightTargetPosition` in logs).
8. **Test at velocity limits:** Run a retract path at max speed and introduce tilt. Verify the clamp-then-bias keeps both sides within limits (check `Climb/VelocityIK/*Vel` logs).

## Alternative Approaches Considered

### A. Position-Based Correction (Rejected)
Offset the end-effector target Y position differentially (`left += offset, right -= offset`). Rejected because:
- Causes the two arms to finish at **different final positions** — both hooks must end at the same (x, y)
- Sluggish during dynamic retract — position offset chases the tilt rather than preventing it
- The root cause is a speed mismatch, so the fix should be at the velocity level

### B. Feed-Forward Tilt Compensation (Complement, Future)
Pre-compute expected cable length differences based on bar geometry and center of gravity. Could be added as a static velocity bias on top of the PID correction. Requires accurate CG measurement.

### C. Full 6-DOF IMU Fusion (Overkill)
Using pitch + roll + angular rates for a full model-based controller. Way too complex for FRC — the simple roll PID on velocity should suffice.

## Dependencies

- `ClimbConstants.ImuAssist` nested class (feature flag + PID gains)
- IMU roll data accessible from ClimbSubsystem (via `DoubleSupplier`, wired in RobotContainer)
- No new hardware required
- No changes to ClimbState waypoints, ClimbIK math, or target positions
- Minimal impact on existing climb control flow — only the velocity vector in `execute()` is modified

## Remaining Work

- Set `ImuAssist.ENABLED = true` and tune PID gains on real hardware
- Bind `climb.toggleAutoLevelCommand()` to an operator button
- Verify roll sign convention on physical robot
