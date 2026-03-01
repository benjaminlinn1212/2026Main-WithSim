package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/** Simulation implementation of TurretIO using physics simulation. */
public class TurretIOSim implements TurretIO {
  private final SingleJointedArmSim sim;
  private double appliedVolts = 0.0;
  private double positionSetpointRad = 0.0;
  private double velocitySetpointRadPerSec = 0.0;
  private boolean closedLoop = false;

  public TurretIOSim() {
    // WPILib expects gear ratio as motor_rot/mechanism_rot (reduction);
    // our GEAR_RATIO is mechanism/motor, so invert it.
    sim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            1.0 / Constants.TurretConstants.GEAR_RATIO,
            0.06,
            0.15,
            Constants.TurretConstants.MIN_POSITION_RAD,
            Constants.TurretConstants.MAX_POSITION_RAD,
            false,
            Constants.TurretConstants.BOOT_POSITION_RAD);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (closedLoop) {
      double error = positionSetpointRad - sim.getAngleRads();
      // KV is in volts per (mechanism rotations/sec), convert rad/s to rot/s
      double ffVolts =
          Units.radiansToRotations(velocitySetpointRadPerSec) * Constants.TurretConstants.KV;
      double fbVolts = error * Constants.TurretConstants.KP;
      appliedVolts = MathUtil.clamp(ffVolts + fbVolts, -12.0, 12.0);
      Logger.recordOutput("Turret/Sim/PErrorDeg", Math.toDegrees(error));
      Logger.recordOutput("Turret/Sim/AppliedVolts", appliedVolts);
    }

    // Update simulation
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

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
    double rawRad = Units.rotationsToRadians(rotationsFromCenter);
    positionSetpointRad =
        MathUtil.clamp(
            rawRad,
            Constants.TurretConstants.MIN_POSITION_RAD,
            Constants.TurretConstants.MAX_POSITION_RAD);
    // Log what the sim receives vs what it clamps to
    Logger.recordOutput("Turret/Sim/RawSetpointDeg", Math.toDegrees(rawRad));
    Logger.recordOutput("Turret/Sim/ClampedSetpointDeg", Math.toDegrees(positionSetpointRad));
    Logger.recordOutput("Turret/Sim/SimAngleDeg", Math.toDegrees(sim.getAngleRads()));
    Logger.recordOutput("Turret/Sim/WasClamped", rawRad != positionSetpointRad);
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
