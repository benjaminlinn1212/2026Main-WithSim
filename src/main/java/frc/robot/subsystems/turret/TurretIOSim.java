package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
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
            DCMotor.getFalcon500(1), // Motor
            Constants.TurretConstants.GEAR_RATIO, // Gear ratio
            0.5, // Moment of inertia (kg*m^2) - adjust based on actual mechanism
            0.2, // Arm length (meters) - for simulation purposes
            Constants.TurretConstants.MIN_POSITION_RAD, // Min angle
            Constants.TurretConstants.MAX_POSITION_RAD, // Max angle
            false, // Simulate gravity (false for turret, true for arm)
            0.0); // Starting angle
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

    // Read simulated values
    inputs.positionRad = sim.getAngleRads();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.absolutePosition = sim.getAngleRads();
    inputs.appliedVolts = appliedVolts;
    inputs.currentStatorAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.currentSupplyAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.temperatureCelsius = 25.0; // Constant temp in sim
  }

  @Override
  public void setPositionSetpoint(double radiansFromCenter, double radPerSecond) {
    closedLoop = true;
    positionSetpointRad =
        MathUtil.clamp(
            radiansFromCenter,
            Constants.TurretConstants.MIN_POSITION_RAD,
            Constants.TurretConstants.MAX_POSITION_RAD);
    velocitySetpointRadPerSec = radPerSecond;
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
