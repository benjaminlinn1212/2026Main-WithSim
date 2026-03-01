package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Pre-match self-test that verifies system health during disabled. Run by pressing Back button on
 * the driver controller. Logs results to AdvantageKit and prints to console.
 *
 * <p>This does NOT move any mechanisms — it only checks battery, CAN bus, and system status.
 * Individual motor temperatures are already logged per-subsystem by their IO implementations.
 */
public class SelfTest {
  private static final double MIN_BATTERY_VOLTAGE = 12.0;

  /**
   * Create a self-test command that runs once and reports results.
   *
   * <p>Safe to run while disabled — does not actuate any mechanisms.
   */
  public static Command selfTestCommand(Superstructure superstructure) {
    return Commands.runOnce(
            () -> {
              List<String> warnings = new ArrayList<>();
              List<String> errors = new ArrayList<>();

              System.out.println("══════════════════════════════════════");
              System.out.println("        PRE-MATCH SELF TEST");
              System.out.println("══════════════════════════════════════");

              // ── Battery Check ──
              double voltage = RobotController.getBatteryVoltage();
              if (voltage < MIN_BATTERY_VOLTAGE) {
                errors.add(
                    "Battery voltage LOW: " + String.format("%.2f", voltage) + "V (min 12.0V)");
              } else {
                System.out.println("[PASS] Battery: " + String.format("%.2f", voltage) + "V");
              }

              // ── CAN Bus Check ──
              var canStatus = RobotController.getCANStatus();
              if (canStatus.percentBusUtilization > 0.8) {
                warnings.add(
                    "CAN bus utilization HIGH: "
                        + String.format("%.0f%%", canStatus.percentBusUtilization * 100));
              } else {
                System.out.println(
                    "[PASS] CAN bus: "
                        + String.format("%.0f%%", canStatus.percentBusUtilization * 100)
                        + " utilization");
              }
              if (canStatus.busOffCount > 0) {
                errors.add("CAN bus-off events: " + canStatus.busOffCount);
              }
              if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
                warnings.add(
                    "CAN errors — TX: "
                        + canStatus.transmitErrorCount
                        + " RX: "
                        + canStatus.receiveErrorCount);
              }

              // ── RoboRIO Check ──
              double cpuTemp = RobotController.getCPUTemp();
              if (cpuTemp > 75.0) {
                warnings.add("RoboRIO CPU temp HIGH: " + String.format("%.1f", cpuTemp) + "°C");
              } else {
                System.out.println("[PASS] RoboRIO CPU: " + String.format("%.1f", cpuTemp) + "°C");
              }

              // ── Superstructure State ──
              var state = superstructure.getState();
              if (state == Superstructure.SuperstructureState.EMERGENCY) {
                warnings.add("Superstructure is in EMERGENCY state — clear before match");
              } else {
                System.out.println("[PASS] Superstructure state: " + state.name());
              }

              // ── Report ──
              System.out.println("──────────────────────────────────────");
              if (errors.isEmpty() && warnings.isEmpty()) {
                System.out.println("✓ ALL CHECKS PASSED — READY TO COMPETE");
              } else {
                for (String w : warnings) {
                  System.out.println("[WARN] " + w);
                }
                for (String e : errors) {
                  System.out.println("[FAIL] " + e);
                }
                System.out.println(
                    "Result: " + errors.size() + " error(s), " + warnings.size() + " warning(s)");
              }
              System.out.println("══════════════════════════════════════");

              // Log to AdvantageKit
              Logger.recordOutput("SelfTest/Passed", errors.isEmpty());
              Logger.recordOutput("SelfTest/ErrorCount", errors.size());
              Logger.recordOutput("SelfTest/WarningCount", warnings.size());
              Logger.recordOutput(
                  "SelfTest/Errors", errors.isEmpty() ? "none" : String.join("; ", errors));
              Logger.recordOutput(
                  "SelfTest/Warnings", warnings.isEmpty() ? "none" : String.join("; ", warnings));
            })
        .ignoringDisable(true)
        .withName("SelfTest");
  }
}
