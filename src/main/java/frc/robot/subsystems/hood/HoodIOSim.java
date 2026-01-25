package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/** Simulation implementation of HoodIO using physics simulation. */
public class HoodIOSim implements HoodIO {
  private final SingleJointedArmSim sim;
  private double appliedVolts = 0.0;
  private double positionSetpointRad = 0.0;
  private double velocitySetpointRadPerSec = 0.0;
  private boolean closedLoop = false;

  public HoodIOSim() {
    // Create a simulated hood using SingleJointedArmSim
    // Parameters: DCMotor, gear ratio, MOI, length, min angle, max angle, simulate gravity,
    // starting angle
    sim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1), // Motor
            Constants.HoodConstants.GEAR_RATIO, // Gear ratio
            0.3, // Moment of inertia (kg*m^2) - adjust based on actual mechanism
            0.3, // Arm length (meters) - approximate hood length
            Constants.HoodConstants.MIN_POSITION_RAD, // Min angle
            Constants.HoodConstants.MAX_POSITION_RAD, // Max angle
            true, // Simulate gravity (true for arm/hood that fights gravity)
            Constants.HoodConstants.MIN_POSITION_RAD); // Starting angle
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Perform closed-loop control in simulation
    if (closedLoop) {
      // Simple PID controller for simulation
      double error = positionSetpointRad - sim.getAngleRads();
      double ffVolts = velocitySetpointRadPerSec * Constants.HoodConstants.KV;
      double fbVolts = error * Constants.HoodConstants.KP;

      // Add gravity compensation
      double gravityVolts = Constants.HoodConstants.KG * Math.cos(sim.getAngleRads());

      appliedVolts = MathUtil.clamp(ffVolts + fbVolts + gravityVolts, -12.0, 12.0);
    }

    // Update simulation
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02); // 20ms update period

    // Read simulated values
    inputs.positionRad = sim.getAngleRads();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentStatorAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.currentSupplyAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.temperatureCelsius = 25.0; // Constant temp in sim
  }

  @Override
  public void setPositionSetpoint(double radiansFromHorizontal, double radPerSecond) {
    closedLoop = true;
    positionSetpointRad =
        MathUtil.clamp(
            radiansFromHorizontal,
            Constants.HoodConstants.MIN_POSITION_RAD,
            Constants.HoodConstants.MAX_POSITION_RAD);
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
