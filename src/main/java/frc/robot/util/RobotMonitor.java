package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized robot health monitoring — battery voltage, brownout detection, and controller rumble
 * feedback. Intended to be called from robotPeriodic() every cycle.
 *
 * <p>Motor temperature monitoring and sticky fault detection are handled per-subsystem in their
 * respective IO implementations, since each subsystem already reads its own motor data via
 * updateInputs().
 */
public class RobotMonitor {
  // ── Battery / Brownout ──
  private static final double BROWNOUT_VOLTAGE_THRESHOLD = 7.0; // RoboRIO brownouts at 6.8V
  private static final double BATTERY_LOG_INTERVAL_SEC = 1.0; // Log battery stats every 1s
  private double lastBatteryLogTime = 0.0;
  private double minVoltage = 13.0;
  private boolean brownoutDetected = false;
  private int brownoutCount = 0;

  // ── Controller Rumble ──
  private final GenericHID driverController;
  private final GenericHID operatorController;
  private double rumbleEndTime = 0.0;

  /**
   * Create the RobotMonitor.
   *
   * @param driverController the driver controller for rumble feedback
   * @param operatorController the operator controller for rumble feedback
   */
  public RobotMonitor(GenericHID driverController, GenericHID operatorController) {
    this.driverController = driverController;
    this.operatorController = operatorController;
  }

  /** Call every robot periodic cycle to update all monitoring. */
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double voltage = RobotController.getBatteryVoltage();

    // Track min voltage
    if (voltage < minVoltage) {
      minVoltage = voltage;
    }

    // Brownout detection
    if (voltage < BROWNOUT_VOLTAGE_THRESHOLD && !brownoutDetected) {
      brownoutDetected = true;
      brownoutCount++;
      System.err.println(
          "[RobotMonitor] BROWNOUT DETECTED! Voltage: "
              + String.format("%.2f", voltage)
              + "V (count: "
              + brownoutCount
              + ")");
      // Rumble both controllers to alert drivers
      rumbleBoth(1.0, 1.0);
    } else if (voltage >= BROWNOUT_VOLTAGE_THRESHOLD + 0.5) {
      // Hysteresis — only clear brownout after voltage recovers by 0.5V
      brownoutDetected = false;
    }

    // Periodic battery logging
    if (now - lastBatteryLogTime >= BATTERY_LOG_INTERVAL_SEC) {
      lastBatteryLogTime = now;
      Logger.recordOutput("Monitor/BatteryVoltage", voltage);
      Logger.recordOutput("Monitor/MinVoltage", minVoltage);
      Logger.recordOutput("Monitor/BrownoutCount", brownoutCount);
      Logger.recordOutput("Monitor/IsBrownout", brownoutDetected);
      Logger.recordOutput("Monitor/RoboRIO_CPUTemp", RobotController.getCPUTemp());
      var canStatus = RobotController.getCANStatus();
      Logger.recordOutput("Monitor/CAN_Utilization", canStatus.percentBusUtilization);
      Logger.recordOutput("Monitor/CAN_OffCount", canStatus.busOffCount);
      Logger.recordOutput("Monitor/CAN_TxErrors", canStatus.transmitErrorCount);
      Logger.recordOutput("Monitor/CAN_RxErrors", canStatus.receiveErrorCount);
    }

    // Manage rumble timeout
    if (now > rumbleEndTime) {
      driverController.setRumble(RumbleType.kBothRumble, 0.0);
      operatorController.setRumble(RumbleType.kBothRumble, 0.0);
    }
  }

  /**
   * Trigger rumble on the driver controller only.
   *
   * @param intensity rumble intensity (0.0 to 1.0)
   * @param durationSec how long to rumble in seconds
   */
  public void rumbleDriver(double intensity, double durationSec) {
    driverController.setRumble(RumbleType.kBothRumble, intensity);
    rumbleEndTime = Math.max(rumbleEndTime, Timer.getFPGATimestamp() + durationSec);
  }

  /**
   * Trigger rumble on both controllers.
   *
   * @param intensity rumble intensity (0.0 to 1.0)
   * @param durationSec how long to rumble in seconds
   */
  public void rumbleBoth(double intensity, double durationSec) {
    driverController.setRumble(RumbleType.kBothRumble, intensity);
    operatorController.setRumble(RumbleType.kBothRumble, intensity);
    rumbleEndTime = Math.max(rumbleEndTime, Timer.getFPGATimestamp() + durationSec);
  }

  /** Get the minimum battery voltage recorded since boot. */
  public double getMinVoltage() {
    return minVoltage;
  }

  /** Get the number of brownout events detected since boot. */
  public int getBrownoutCount() {
    return brownoutCount;
  }
}
