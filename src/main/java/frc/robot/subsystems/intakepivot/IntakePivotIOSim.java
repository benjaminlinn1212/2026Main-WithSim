package frc.robot.subsystems.intakepivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.IntakePivotConstants;

/**
 * Simulation implementation of IntakePivotIO using WPILib ElevatorSim. Treats the elevator "height"
 * directly as motor rotations (0 = stowed, 28 = max extension). Uses a ProfiledPIDController +
 * ElevatorFeedforward (kS, kV, kA, kG) to mimic Motion Magic.
 */
public class IntakePivotIOSim implements IntakePivotIO {

  private final ElevatorSim sim;
  private final ProfiledPIDController profiledPID;
  private final ElevatorFeedforward feedforward;
  private double appliedVolts = 0.0;
  private boolean closedLoop = false;

  // --- Sim-only PID gains (separate from real robot) ---
  private static final double SIM_KP = 1.5;
  private static final double SIM_KI = 0.0;
  private static final double SIM_KD = 0.4;

  // --- Sim-only feedforward gains ---
  private static final double SIM_KS = 0.0;
  private static final double SIM_KG = 0.05;
  private static final double SIM_KV = 0.0;
  private static final double SIM_KA = 0.0;

  // --- Sim-only Motion Magic constraints ---
  private static final double SIM_CRUISE_VELOCITY = 100.0; // rot/s
  private static final double SIM_ACCELERATION = 300.0; // rot/s^2

  public IntakePivotIOSim() {
    sim =
        new ElevatorSim(
            DCMotor.getKrakenX60(1),
            1.0, // gear ratio (direct drive)
            1.0, // carriage mass (kg)
            1.0 / (2.0 * Math.PI), // drum radius so 1 rot = 1 "meter"
            IntakePivotConstants.SOFT_LIMIT_REVERSE, // min (0)
            IntakePivotConstants.SOFT_LIMIT_FORWARD, // max (28)
            false, // no gravity
            IntakePivotConstants.STOWED_POSITION); // start stowed (0)

    profiledPID =
        new ProfiledPIDController(
            SIM_KP,
            SIM_KI,
            SIM_KD,
            new TrapezoidProfile.Constraints(SIM_CRUISE_VELOCITY, SIM_ACCELERATION));
    profiledPID.setTolerance(0.2);

    feedforward = new ElevatorFeedforward(SIM_KS, SIM_KG, SIM_KV, SIM_KA);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    if (closedLoop) {
      double pidVolts = profiledPID.calculate(sim.getPositionMeters());
      TrapezoidProfile.State setpoint = profiledPID.getSetpoint();
      double ffVolts = feedforward.calculate(setpoint.velocity);
      appliedVolts = MathUtil.clamp(pidVolts + ffVolts, -12.0, 12.0);
    }

    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.positionRotations = sim.getPositionMeters();
    inputs.velocityRotPerSec = sim.getVelocityMetersPerSecond();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.temperatureCelsius = 25.0;
  }

  @Override
  public void setPosition(double positionRotations) {
    closedLoop = true;
    double clamped =
        MathUtil.clamp(
            positionRotations,
            IntakePivotConstants.SOFT_LIMIT_REVERSE,
            IntakePivotConstants.SOFT_LIMIT_FORWARD);
    profiledPID.setGoal(clamped);
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
    profiledPID.reset(sim.getPositionMeters());
  }
}
