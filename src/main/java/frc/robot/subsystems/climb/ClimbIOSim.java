package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.util.ClimbIK;

/**
 * Simulated ClimbIO with MotionMagic-style trapezoidal profiles. All positions/velocities in drum
 * (mechanism) rotations, matching the IO interface contract.
 */
public class ClimbIOSim implements ClimbIO {

  // Per-motor simulated state
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
  private static final double DT = 0.02;

  // Motion profile constraints (mechanism rotations, matching ClimbConstants)
  // Front and back motors have different gear ratios → different max drum speeds.
  private static final double FRONT_CRUISE_VEL = ClimbConstants.CRUISE_VELOCITY;
  private static final double FRONT_MAX_ACCEL = ClimbConstants.ACCELERATION;
  private static final double BACK_CRUISE_VEL = ClimbConstants.BACK_CRUISE_VELOCITY;
  private static final double BACK_MAX_ACCEL = ClimbConstants.BACK_ACCELERATION;

  public ClimbIOSim() {
    // Initialize to STOWED cable-length rotations so the sim starts at the correct position
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
  }

  @Override
  public void setRightFrontPosition(double positionRotations) {
    rightFrontVelocityRotPerSec =
        simulateMotionMagicStep(
            rightFrontPositionRotations,
            rightFrontVelocityRotPerSec,
            positionRotations,
            FRONT_CRUISE_VEL,
            FRONT_MAX_ACCEL);
    rightFrontPositionRotations += rightFrontVelocityRotPerSec * DT;
  }

  @Override
  public void setRightBackPosition(double positionRotations) {
    rightBackVelocityRotPerSec =
        simulateMotionMagicStep(
            rightBackPositionRotations,
            rightBackVelocityRotPerSec,
            positionRotations,
            BACK_CRUISE_VEL,
            BACK_MAX_ACCEL);
    rightBackPositionRotations += rightBackVelocityRotPerSec * DT;
  }

  @Override
  public void setLeftFrontPosition(double positionRotations) {
    leftFrontVelocityRotPerSec =
        simulateMotionMagicStep(
            leftFrontPositionRotations,
            leftFrontVelocityRotPerSec,
            positionRotations,
            FRONT_CRUISE_VEL,
            FRONT_MAX_ACCEL);
    leftFrontPositionRotations += leftFrontVelocityRotPerSec * DT;
  }

  @Override
  public void setLeftBackPosition(double positionRotations) {
    leftBackVelocityRotPerSec =
        simulateMotionMagicStep(
            leftBackPositionRotations,
            leftBackVelocityRotPerSec,
            positionRotations,
            BACK_CRUISE_VEL,
            BACK_MAX_ACCEL);
    leftBackPositionRotations += leftBackVelocityRotPerSec * DT;
  }

  @Override
  public void setRightFrontVelocity(double velocityRotPerSec, double feedforwardVolts) {
    rightFrontVelocityRotPerSec =
        rampVelocity(
            rightFrontVelocityRotPerSec, velocityRotPerSec, FRONT_CRUISE_VEL, FRONT_MAX_ACCEL);
    rightFrontPositionRotations += rightFrontVelocityRotPerSec * DT;
  }

  @Override
  public void setRightBackVelocity(double velocityRotPerSec, double feedforwardVolts) {
    rightBackVelocityRotPerSec =
        rampVelocity(
            rightBackVelocityRotPerSec, velocityRotPerSec, BACK_CRUISE_VEL, BACK_MAX_ACCEL);
    rightBackPositionRotations += rightBackVelocityRotPerSec * DT;
  }

  @Override
  public void setLeftFrontVelocity(double velocityRotPerSec, double feedforwardVolts) {
    leftFrontVelocityRotPerSec =
        rampVelocity(
            leftFrontVelocityRotPerSec, velocityRotPerSec, FRONT_CRUISE_VEL, FRONT_MAX_ACCEL);
    leftFrontPositionRotations += leftFrontVelocityRotPerSec * DT;
  }

  @Override
  public void setLeftBackVelocity(double velocityRotPerSec, double feedforwardVolts) {
    leftBackVelocityRotPerSec =
        rampVelocity(leftBackVelocityRotPerSec, velocityRotPerSec, BACK_CRUISE_VEL, BACK_MAX_ACCEL);
    leftBackPositionRotations += leftBackVelocityRotPerSec * DT;
  }

  // ── MotionMagic-style trapezoidal profile simulation ──

  // P gain that mimics TalonFX PID closing the loop around the profile setpoint
  private static final double POSITION_CORRECTION_KP = 50.0;

  /**
   * Simulate one DT step of trapezoidal motion profile. Accel → cruise → decel with proportional
   * position correction to eliminate steady-state integration drift.
   */
  private static double simulateMotionMagicStep(
      double currentPos, double currentVel, double targetPos, double cruiseVel, double maxAccel) {
    double error = targetPos - currentPos;
    double absError = Math.abs(error);
    double direction = Math.signum(error);

    // If very close and slow enough, snap to zero velocity (holding position)
    if (absError < 1e-3 && Math.abs(currentVel) < 0.1) {
      return 0.0;
    }

    // Calculate braking distance at current velocity: d = v² / (2*a)
    double brakingDistance = (currentVel * currentVel) / (2.0 * maxAccel);

    double desiredVel;
    if (absError <= brakingDistance + cruiseVel * DT) {
      // Deceleration phase: ramp down to stop at target
      // Target velocity to reach zero at target position: v = sqrt(2 * a * remaining)
      desiredVel = direction * Math.sqrt(Math.max(0, 2.0 * maxAccel * absError));
      // Don't exceed cruise
      desiredVel = MathUtil.clamp(desiredVel, -cruiseVel, cruiseVel);
    } else {
      // Acceleration/cruise phase
      desiredVel = direction * cruiseVel;
    }

    // Add proportional position correction (like real TalonFX PID slot)
    desiredVel += error * POSITION_CORRECTION_KP;

    // Apply acceleration limit to velocity change
    return rampVelocity(currentVel, desiredVel, cruiseVel, maxAccel);
  }

  /** Ramp velocity toward target, limited by maxAccel * DT per cycle. */
  private static double rampVelocity(
      double currentVel, double targetVel, double cruiseVel, double maxAccel) {
    double maxDeltaVel = maxAccel * DT;
    double deltaVel = targetVel - currentVel;
    deltaVel = MathUtil.clamp(deltaVel, -maxDeltaVel, maxDeltaVel);
    double newVel = currentVel + deltaVel;
    // Also clamp absolute velocity to cruise
    return MathUtil.clamp(newVel, -cruiseVel, cruiseVel);
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
    // Re-seed simulated positions to STOWED (re-aligns encoder frame)
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
