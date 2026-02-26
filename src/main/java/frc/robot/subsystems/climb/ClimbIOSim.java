package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.util.ClimbIK;

/**
 * Simulated ClimbIO that models MotionMagic-style trapezoidal motion profiles.
 *
 * <p>All positions and velocities in this class are in <b>mechanism (drum) rotations</b>, matching
 * the IO interface contract. The real hardware converts to motor rotations via gear ratio
 * internally (TalonFX SensorToMechanismRatio=1.0, code divides by GEAR_RATIO before sending to
 * motor).
 *
 * <p>The sim enforces the same cruise-velocity and acceleration constraints as the real MotionMagic
 * config, so the Mechanism2d visualization moves at a physically realistic speed.
 */
public class ClimbIOSim implements ClimbIO {

  // ── Per-motor simulated state ──
  private double rightFrontPositionRotations;
  private double rightFrontVelocityRotPerSec = 0.0;
  private double rightFrontAppliedVolts = 0.0;

  private double rightBackPositionRotations;
  private double rightBackVelocityRotPerSec = 0.0;
  private double rightBackAppliedVolts = 0.0;

  private double leftFrontPositionRotations;
  private double leftFrontVelocityRotPerSec = 0.0;
  private double leftFrontAppliedVolts = 0.0;

  private double leftBackPositionRotations;
  private double leftBackVelocityRotPerSec = 0.0;
  private double leftBackAppliedVolts = 0.0;

  // Secondary Hook Angle Servos
  private double leftSecondaryHookAngleServoPosition = 0.0;
  private double rightSecondaryHookAngleServoPosition = 0.0;

  // Secondary Hook Hardstop Servos
  private double leftSecondaryHookHardstopServoPosition = 0.0;
  private double rightSecondaryHookHardstopServoPosition = 0.0;

  // ── Simulation timing ──
  private static final double DT = 0.02; // 20ms loop time

  // ── Motion profile constraints (mechanism rotations) ──
  // These match ClimbConstants; they are the *mechanism-side* limits.
  private static final double CRUISE_VEL = ClimbConstants.CRUISE_VELOCITY; // mech rot/s
  private static final double MAX_ACCEL = ClimbConstants.ACCELERATION; // mech rot/s²

  public ClimbIOSim() {
    // Initialize motor positions to the STOWED cable-length rotations so the sim starts
    // with the arm in the correct position (not at 0 rotations which is physically wrong).
    Translation2d stowedPos = ClimbState.STOWED.getTargetPosition();
    ClimbIK.ClimbSideIKResult stowedIK = ClimbIK.calculateIK(stowedPos);
    if (stowedIK.isValid) {
      leftFrontPositionRotations = stowedIK.frontMotorRotations;
      leftBackPositionRotations = stowedIK.backMotorRotations;
      rightFrontPositionRotations = stowedIK.frontMotorRotations;
      rightBackPositionRotations = stowedIK.backMotorRotations;
    } else {
      leftFrontPositionRotations = 0.0;
      leftBackPositionRotations = 0.0;
      rightFrontPositionRotations = 0.0;
      rightBackPositionRotations = 0.0;
    }
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // Simple velocity-based simulation (Motion Magic handles position control)
    // In real robot, Phoenix does position control; here we simulate converging to target

    // Right Front Motor
    inputs.rightFrontPositionRotations = rightFrontPositionRotations;
    inputs.rightFrontVelocityRotPerSec = rightFrontVelocityRotPerSec;
    inputs.rightFrontAppliedVolts = rightFrontAppliedVolts;
    inputs.rightFrontCurrentAmps =
        Math.abs(rightFrontVelocityRotPerSec) * 0.5; // Simplified current

    // Right Back Motor
    inputs.rightBackPositionRotations = rightBackPositionRotations;
    inputs.rightBackVelocityRotPerSec = rightBackVelocityRotPerSec;
    inputs.rightBackAppliedVolts = rightBackAppliedVolts;
    inputs.rightBackCurrentAmps = Math.abs(rightBackVelocityRotPerSec) * 0.5;

    // Left Front Motor
    inputs.leftFrontPositionRotations = leftFrontPositionRotations;
    inputs.leftFrontVelocityRotPerSec = leftFrontVelocityRotPerSec;
    inputs.leftFrontAppliedVolts = leftFrontAppliedVolts;
    inputs.leftFrontCurrentAmps = Math.abs(leftFrontVelocityRotPerSec) * 0.5;

    // Left Back Motor
    inputs.leftBackPositionRotations = leftBackPositionRotations;
    inputs.leftBackVelocityRotPerSec = leftBackVelocityRotPerSec;
    inputs.leftBackAppliedVolts = leftBackAppliedVolts;
    inputs.leftBackCurrentAmps = Math.abs(leftBackVelocityRotPerSec) * 0.5;

    // Secondary Hook Angle Servos
    inputs.leftSecondaryHookAngleServoPosition = leftSecondaryHookAngleServoPosition;
    inputs.rightSecondaryHookAngleServoPosition = rightSecondaryHookAngleServoPosition;

    // Secondary Hook Hardstop Servos
    inputs.leftSecondaryHookHardstopServoPosition = leftSecondaryHookHardstopServoPosition;
    inputs.rightSecondaryHookHardstopServoPosition = rightSecondaryHookHardstopServoPosition;

    // Update Mechanism2d visualization
    // Note: Mechanism2d visualization would need to be sent to SmartDashboard separately
    // For now, the end effector and joint positions logged in ClimbSubsystem are sufficient
  }

  @Override
  public void setRightFrontPosition(double positionRotations) {
    rightFrontVelocityRotPerSec =
        simulateMotionMagicStep(
            rightFrontPositionRotations, rightFrontVelocityRotPerSec, positionRotations);
    rightFrontPositionRotations += rightFrontVelocityRotPerSec * DT;
  }

  @Override
  public void setRightBackPosition(double positionRotations) {
    rightBackVelocityRotPerSec =
        simulateMotionMagicStep(
            rightBackPositionRotations, rightBackVelocityRotPerSec, positionRotations);
    rightBackPositionRotations += rightBackVelocityRotPerSec * DT;
  }

  @Override
  public void setLeftFrontPosition(double positionRotations) {
    leftFrontVelocityRotPerSec =
        simulateMotionMagicStep(
            leftFrontPositionRotations, leftFrontVelocityRotPerSec, positionRotations);
    leftFrontPositionRotations += leftFrontVelocityRotPerSec * DT;
  }

  @Override
  public void setLeftBackPosition(double positionRotations) {
    leftBackVelocityRotPerSec =
        simulateMotionMagicStep(
            leftBackPositionRotations, leftBackVelocityRotPerSec, positionRotations);
    leftBackPositionRotations += leftBackVelocityRotPerSec * DT;
  }

  @Override
  public void setRightFrontVelocity(double velocityRotPerSec, double feedforwardVolts) {
    rightFrontVelocityRotPerSec = rampVelocity(rightFrontVelocityRotPerSec, velocityRotPerSec);
    rightFrontPositionRotations += rightFrontVelocityRotPerSec * DT;
  }

  @Override
  public void setRightBackVelocity(double velocityRotPerSec, double feedforwardVolts) {
    rightBackVelocityRotPerSec = rampVelocity(rightBackVelocityRotPerSec, velocityRotPerSec);
    rightBackPositionRotations += rightBackVelocityRotPerSec * DT;
  }

  @Override
  public void setLeftFrontVelocity(double velocityRotPerSec, double feedforwardVolts) {
    leftFrontVelocityRotPerSec = rampVelocity(leftFrontVelocityRotPerSec, velocityRotPerSec);
    leftFrontPositionRotations += leftFrontVelocityRotPerSec * DT;
  }

  @Override
  public void setLeftBackVelocity(double velocityRotPerSec, double feedforwardVolts) {
    leftBackVelocityRotPerSec = rampVelocity(leftBackVelocityRotPerSec, velocityRotPerSec);
    leftBackPositionRotations += leftBackVelocityRotPerSec * DT;
  }

  // ── MotionMagic-style trapezoidal profile simulation ──

  /**
   * Simulate one DT step of a trapezoidal motion profile (MotionMagic). The motor accelerates
   * toward cruise velocity, decelerates to stop at the target, and respects the acceleration limit.
   *
   * @param currentPos Current mechanism position (rotations)
   * @param currentVel Current mechanism velocity (rot/s)
   * @param targetPos Desired mechanism position (rotations)
   * @return New velocity for this cycle (rot/s)
   */
  private static double simulateMotionMagicStep(
      double currentPos, double currentVel, double targetPos) {
    double error = targetPos - currentPos;
    double absError = Math.abs(error);
    double direction = Math.signum(error);

    // If very close and slow enough, snap to zero velocity (holding position)
    if (absError < 1e-5 && Math.abs(currentVel) < 1e-3) {
      return 0.0;
    }

    // Calculate braking distance at current velocity: d = v² / (2*a)
    double brakingDistance = (currentVel * currentVel) / (2.0 * MAX_ACCEL);

    double desiredVel;
    if (absError <= brakingDistance + CRUISE_VEL * DT) {
      // Deceleration phase: ramp down to stop at target
      // Target velocity to reach zero at target position: v = sqrt(2 * a * remaining)
      desiredVel = direction * Math.sqrt(Math.max(0, 2.0 * MAX_ACCEL * absError));
      // Don't exceed cruise
      desiredVel = MathUtil.clamp(desiredVel, -CRUISE_VEL, CRUISE_VEL);
    } else {
      // Acceleration/cruise phase
      desiredVel = direction * CRUISE_VEL;
    }

    // Apply acceleration limit to velocity change
    return rampVelocity(currentVel, desiredVel);
  }

  /**
   * Ramp current velocity toward a target velocity, limited by MAX_ACCEL * DT per cycle.
   *
   * @param currentVel Current velocity (rot/s)
   * @param targetVel Desired velocity (rot/s)
   * @return New velocity after one DT step, acceleration-limited
   */
  private static double rampVelocity(double currentVel, double targetVel) {
    double maxDeltaVel = MAX_ACCEL * DT;
    double deltaVel = targetVel - currentVel;
    deltaVel = MathUtil.clamp(deltaVel, -maxDeltaVel, maxDeltaVel);
    double newVel = currentVel + deltaVel;
    // Also clamp absolute velocity to cruise
    return MathUtil.clamp(newVel, -CRUISE_VEL, CRUISE_VEL);
  }

  @Override
  public void setRightFrontVoltage(double volts) {
    this.rightFrontAppliedVolts = volts;
  }

  @Override
  public void setRightBackVoltage(double volts) {
    this.rightBackAppliedVolts = volts;
  }

  @Override
  public void setLeftFrontVoltage(double volts) {
    this.leftFrontAppliedVolts = volts;
  }

  @Override
  public void setLeftBackVoltage(double volts) {
    this.leftBackAppliedVolts = volts;
  }

  @Override
  public void setLeftSecondaryHookAnglePosition(double position) {
    this.leftSecondaryHookAngleServoPosition = position;
  }

  @Override
  public void setRightSecondaryHookAnglePosition(double position) {
    this.rightSecondaryHookAngleServoPosition = position;
  }

  @Override
  public void setLeftSecondaryHookHardstopPosition(double position) {
    this.leftSecondaryHookHardstopServoPosition = position;
  }

  @Override
  public void setRightSecondaryHookHardstopPosition(double position) {
    this.rightSecondaryHookHardstopServoPosition = position;
  }

  @Override
  public void stop() {
    this.rightFrontAppliedVolts = 0.0;
    this.rightBackAppliedVolts = 0.0;
    this.leftFrontAppliedVolts = 0.0;
    this.leftBackAppliedVolts = 0.0;
    this.rightFrontVelocityRotPerSec = 0.0;
    this.rightBackVelocityRotPerSec = 0.0;
    this.leftFrontVelocityRotPerSec = 0.0;
    this.leftBackVelocityRotPerSec = 0.0;
  }

  @Override
  public void resetToStowed() {
    Translation2d stowedPos = ClimbState.STOWED.getTargetPosition();
    ClimbIK.ClimbSideIKResult stowedIK = ClimbIK.calculateIK(stowedPos);
    if (stowedIK.isValid) {
      leftFrontPositionRotations = stowedIK.frontMotorRotations;
      leftBackPositionRotations = stowedIK.backMotorRotations;
      rightFrontPositionRotations = stowedIK.frontMotorRotations;
      rightBackPositionRotations = stowedIK.backMotorRotations;
    }
    // Zero all velocities and voltages
    stop();
  }

  @Override
  public void recalibrateEncoders() {
    // Re-seed simulated positions to match STOWED cable-length rotations (same as resetToStowed
    // but without zeroing velocities/voltages — just re-align the encoder frame).
    Translation2d stowedPos = ClimbState.STOWED.getTargetPosition();
    ClimbIK.ClimbSideIKResult stowedIK = ClimbIK.calculateIK(stowedPos);
    if (stowedIK.isValid) {
      leftFrontPositionRotations = stowedIK.frontMotorRotations;
      leftBackPositionRotations = stowedIK.backMotorRotations;
      rightFrontPositionRotations = stowedIK.frontMotorRotations;
      rightBackPositionRotations = stowedIK.backMotorRotations;
    }
  }
}
