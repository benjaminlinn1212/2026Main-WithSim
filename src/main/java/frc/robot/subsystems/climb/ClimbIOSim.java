package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOSim implements ClimbIO {

  private double rightFrontPositionRotations = 0.0;
  private double rightFrontVelocityRotPerSec = 0.0;
  private double rightFrontAppliedVolts = 0.0;

  private double rightBackPositionRotations = 0.0;
  private double rightBackVelocityRotPerSec = 0.0;
  private double rightBackAppliedVolts = 0.0;

  private double leftFrontPositionRotations = 0.0;
  private double leftFrontVelocityRotPerSec = 0.0;
  private double leftFrontAppliedVolts = 0.0;

  private double leftBackPositionRotations = 0.0;
  private double leftBackVelocityRotPerSec = 0.0;
  private double leftBackAppliedVolts = 0.0;

  // Passive Hook Servos
  private double leftHookServoPosition = 0.0;
  private double rightHookServoPosition = 0.0;

  // Mechanism2d visualization
  private final Mechanism2d mechanism;
  private final MechanismRoot2d leftWinch1Root;
  private final MechanismLigament2d leftLink1;
  private final MechanismLigament2d leftLink2;

  // Simple physics parameters
  private static final double DT = 0.02; // 20ms loop time

  public ClimbIOSim() {
    // Create Mechanism2d for visualization (1.5m x 1.5m)
    mechanism = new Mechanism2d(1.5, 1.5);

    // Left side winches and links
    leftWinch1Root = mechanism.getRoot("LeftWinch1", 0.0, 0.0);

    // Two-link arm (approximate angles, updated in updateInputs)
    leftLink1 =
        leftWinch1Root.append(
            new MechanismLigament2d(
                "LeftLink1",
                ClimbConstants.LINK_1_LENGTH_METERS,
                45,
                6,
                new Color8Bit(Color.kBlue)));
    leftLink2 =
        leftLink1.append(
            new MechanismLigament2d(
                "LeftLink2",
                ClimbConstants.LINK_2_LENGTH_METERS,
                -30,
                4,
                new Color8Bit(Color.kGreen)));
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

    // Passive Hook Servos
    inputs.leftHookServoPosition = leftHookServoPosition;
    inputs.rightHookServoPosition = rightHookServoPosition;

    // Update Mechanism2d visualization
    // Note: Mechanism2d visualization would need to be sent to SmartDashboard separately
    // For now, the end effector and joint positions logged in ClimbSubsystem are sufficient
  }

  @Override
  public void setRightFrontPosition(double positionRotations) {
    // Simulate motion toward target with simple velocity model
    double error = positionRotations - this.rightFrontPositionRotations;
    double targetVelocity =
        Math.signum(error) * Math.min(Math.abs(error) / DT, ClimbConstants.CRUISE_VELOCITY);
    this.rightFrontVelocityRotPerSec = targetVelocity;
    this.rightFrontPositionRotations += targetVelocity * DT;
  }

  @Override
  public void setRightBackPosition(double positionRotations) {
    double error = positionRotations - this.rightBackPositionRotations;
    double targetVelocity =
        Math.signum(error) * Math.min(Math.abs(error) / DT, ClimbConstants.CRUISE_VELOCITY);
    this.rightBackVelocityRotPerSec = targetVelocity;
    this.rightBackPositionRotations += targetVelocity * DT;
  }

  @Override
  public void setLeftFrontPosition(double positionRotations) {
    double error = positionRotations - this.leftFrontPositionRotations;
    double targetVelocity =
        Math.signum(error) * Math.min(Math.abs(error) / DT, ClimbConstants.CRUISE_VELOCITY);
    this.leftFrontVelocityRotPerSec = targetVelocity;
    this.leftFrontPositionRotations += targetVelocity * DT;
  }

  @Override
  public void setLeftBackPosition(double positionRotations) {
    double error = positionRotations - this.leftBackPositionRotations;
    double targetVelocity =
        Math.signum(error) * Math.min(Math.abs(error) / DT, ClimbConstants.CRUISE_VELOCITY);
    this.leftBackVelocityRotPerSec = targetVelocity;
    this.leftBackPositionRotations += targetVelocity * DT;
  }

  @Override
  public void setRightFrontVelocity(double velocityRotPerSec, double feedforwardVolts) {
    this.rightFrontVelocityRotPerSec = velocityRotPerSec;
    this.rightFrontPositionRotations += velocityRotPerSec * DT;
  }

  @Override
  public void setRightBackVelocity(double velocityRotPerSec, double feedforwardVolts) {
    this.rightBackVelocityRotPerSec = velocityRotPerSec;
    this.rightBackPositionRotations += velocityRotPerSec * DT;
  }

  @Override
  public void setLeftFrontVelocity(double velocityRotPerSec, double feedforwardVolts) {
    this.leftFrontVelocityRotPerSec = velocityRotPerSec;
    this.leftFrontPositionRotations += velocityRotPerSec * DT;
  }

  @Override
  public void setLeftBackVelocity(double velocityRotPerSec, double feedforwardVolts) {
    this.leftBackVelocityRotPerSec = velocityRotPerSec;
    this.leftBackPositionRotations += velocityRotPerSec * DT;
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
  public void setLeftHookPosition(double position) {
    this.leftHookServoPosition = position;
  }

  @Override
  public void setRightHookPosition(double position) {
    this.rightHookServoPosition = position;
  }

  @Override
  public void stop() {
    this.rightFrontAppliedVolts = 0.0;
    this.rightBackAppliedVolts = 0.0;
    this.leftFrontAppliedVolts = 0.0;
    this.leftBackAppliedVolts = 0.0;
  }
}
