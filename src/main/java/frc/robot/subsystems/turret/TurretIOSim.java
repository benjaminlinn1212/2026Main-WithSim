package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/** Simulation implementation of TurretIO using physics simulation. */
public class TurretIOSim implements TurretIO {
  private final SingleJointedArmSim sim;
  private double appliedVolts = 0.0;
  private double positionSetpointRad = 0.0;
  private double velocitySetpointRadPerSec = 0.0;
  private boolean closedLoop = false;

  public TurretIOSim() {
    // Create a simulated turret using SingleJointedArmSim
    // Parameters: DCMotor, gear ratio, MOI, length, min angle, max angle, simulate gravity,
    // starting angle
    sim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            Constants.TurretConstants.GEAR_RATIO,
            0.06,
            0.15,
            Constants.TurretConstants.MIN_POSITION_RAD,
            Constants.TurretConstants.MAX_POSITION_RAD,
            false,
            0.0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Perform closed-loop control in simulation
    if (closedLoop) {
      // Simple P controller for simulation
      double error = positionSetpointRad - sim.getAngleRads();
      double ffVolts = velocitySetpointRadPerSec * Constants.TurretConstants.KV;
      double fbVolts = error * Constants.TurretConstants.KP;
      appliedVolts = MathUtil.clamp(ffVolts + fbVolts, -12.0, 12.0);
    }

    // Update simulation
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02); // 20ms update period

    // Read simulated values (convert from radians to rotations)
    inputs.positionRot = Units.radiansToRotations(sim.getAngleRads());
    inputs.velocityRotPerSec = Units.radiansToRotations(sim.getVelocityRadPerSec());
    inputs.absolutePosition = sim.getAngleRads();
    inputs.appliedVolts = appliedVolts;
    inputs.currentStatorAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.currentSupplyAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.temperatureCelsius = 25.0; // Constant temp in sim
  }

  @Override
  public void setPositionSetpoint(double rotationsFromCenter, double feedforwardVolts) {
    closedLoop = true;
    // Convert rotations to radians for internal sim calculations
    positionSetpointRad =
        MathUtil.clamp(
            Units.rotationsToRadians(rotationsFromCenter),
            Constants.TurretConstants.MIN_POSITION_RAD,
            Constants.TurretConstants.MAX_POSITION_RAD);
    // In sim, we can derive feedforward velocity from voltage if needed
    // For now, just use the position setpoint (sim doesn't need perfect FF)
    velocitySetpointRadPerSec = 0.0;
  }

  @Override
  public void setOpenLoopDutyCycle(double dutyCycle) {
    closedLoop = false;
    appliedVolts = dutyCycle * 12.0;
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }
}
