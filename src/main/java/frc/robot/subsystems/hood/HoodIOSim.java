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
  private boolean closedLoop = false;

  // Sim-tuned PID gains (different from hardware MotionMagic gains)
  // Hardware uses MotionMagic with motion profiling; sim uses a simple PD controller
  // so we tune separately for realistic sim behavior.
  private static final double SIM_KP = 200.0; // volts per mechanism radian of error
  private static final double SIM_KD = 10.0; // volts per mechanism rad/s

  public HoodIOSim() {
    // Create a simulated hood using SingleJointedArmSim
    // Parameters: DCMotor, gear ratio, MOI, length, min angle, max angle, simulate gravity,
    // starting angle
    sim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            1.0
                / Constants.HoodConstants
                    .GEAR_RATIO, // Gear ratio (motor rotations per mechanism rotation)
            0.3, // Moment of inertia (kg*m^2) - adjust based on actual mechanism
            0.3, // Arm length (meters) - approximate hood length
            Constants.HoodConstants.MIN_POSITION_RAD, // Min angle
            Constants.HoodConstants.MAX_POSITION_RAD, // Max angle
            false, // Disable gravity — hood range is small and KG is hardware-tuned
            Constants.HoodConstants.MIN_POSITION_RAD); // Starting angle
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Perform closed-loop control in simulation
    if (closedLoop) {
      // Use sim-tuned PD controller (not hardware Phoenix6 gains)
      double errorRad = positionSetpointRad - sim.getAngleRads();
      double fbVolts = errorRad * SIM_KP - sim.getVelocityRadPerSec() * SIM_KD;

      appliedVolts = MathUtil.clamp(fbVolts, -12.0, 12.0);
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
