/**
 * Dashboard-driven autonomous system for the 2026 REBUILT game.
 *
 * <h2>Architecture Overview</h2>
 *
 * <p>This package implements a runtime-configurable autonomous system inspired by Teams 254 and
 * 6328. Instead of pre-coded autonomous routines, the drive team configures settings on the
 * dashboard before each match, and the system generates a valid autonomous sequence at runtime.
 *
 * <h3>REBUILT Game Context</h3>
 *
 * <p>In REBUILT, robots score FUEL (foam balls) into their alliance's HUB, collect FUEL from the
 * OUTPOST (human player), DEPOT (floor bin), or NEUTRAL ZONE (scattered on field), and climb their
 * alliance's TOWER (3 RUNG levels) for endgame points. The field features TRENCHES (low tunnels)
 * and BUMPS (ramps) that create natural lane boundaries. AUTO is 20 seconds.
 *
 * <h3>Key Classes</h3>
 *
 * <ul>
 *   <li>{@link frc.robot.auto.dashboard.AutoSettings} — All dashboard-configurable settings (start
 *       pose, lanes, shooting priority, risk level, climb level, preload count, etc.).
 *   <li>{@link frc.robot.auto.dashboard.FieldConstants} — REBUILT field geometry: HUB scoring
 *       waypoints, FUEL intake locations (OUTPOST/DEPOT/NEUTRAL ZONE), TOWER climb levels, zones.
 *   <li>{@link frc.robot.auto.dashboard.AutoAction} — Abstract action types that compose into a
 *       sequence (ScoreAt, IntakeAt, DriveTo, Climb, Wait, etc.).
 *   <li>{@link frc.robot.auto.dashboard.AutoPlanner} — The planning engine: reads settings,
 *       generates an optimal sequence of actions that respects constraints and fits within 20s.
 *   <li>{@link frc.robot.auto.dashboard.AutoCommandBuilder} — Converts planned actions into
 *       runnable WPILib Commands (PathPlanner pathfinding + superstructure commands).
 *   <li>{@link frc.robot.auto.dashboard.DashboardAutoManager} — Lifecycle manager: polls dashboard,
 *       triggers replanning, publishes previews, provides the auto command.
 * </ul>
 *
 * <h3>Workflow</h3>
 *
 * <ol>
 *   <li>Before match: configure settings in the "Auto Settings" Shuffleboard tab.
 *   <li>During disabled: {@code DashboardAutoManager.update()} reads settings and replans on
 *       change. The plan preview is logged to AdvantageKit.
 *   <li>At autonomousInit: {@code DashboardAutoManager.getAutoCommand()} builds a fresh command
 *       tree from the latest plan.
 *   <li>During auto: the command executes — pathfinding, collecting FUEL, shooting into HUB,
 *       climbing TOWER.
 * </ol>
 *
 * <h3>Dashboard Settings</h3>
 *
 * <table>
 *   <tr><th>Setting</th><th>Type</th><th>Example</th></tr>
 *   <tr><td>Start Pose</td><td>String</td><td>UPPER, CENTER, LOWER</td></tr>
 *   <tr><td>Shooting Priority</td><td>String</td><td>HUB_BACK_CENTER,HUB_UPPER_CLOSE</td></tr>
 *   <tr><td>Preferred Intake</td><td>String</td><td>OUTPOST, DEPOT, NEUTRAL_ZONE_CENTER</td></tr>
 *   <tr><td>Allowed Lanes</td><td>String</td><td>UPPER,CENTER</td></tr>
 *   <tr><td>Partner Lanes</td><td>String</td><td>LOWER</td></tr>
 *   <tr><td>Attempt TOWER Climb</td><td>Boolean</td><td>true/false</td></tr>
 *   <tr><td>Climb Level</td><td>String</td><td>LEVEL_1, LEVEL_2, LEVEL_3</td></tr>
 *   <tr><td>Shoot While Driving</td><td>Boolean</td><td>true/false</td></tr>
 *   <tr><td>Risk Level</td><td>String</td><td>CONSERVATIVE, BALANCED, AGGRESSIVE</td></tr>
 *   <tr><td>Preload FUEL Count</td><td>Integer</td><td>8 (max per game manual)</td></tr>
 *   <tr><td>Max Cycles</td><td>Integer</td><td>4</td></tr>
 * </table>
 */
package frc.robot.auto.dashboard;
