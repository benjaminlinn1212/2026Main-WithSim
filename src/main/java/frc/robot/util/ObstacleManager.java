// Copyright (c) 2026 FRC Team 0 (Amped)
// Obstacle management for PathPlanner dynamic pathfinding

package frc.robot.util;

import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;

/**
 * Manages dynamic obstacles for PathPlanner's AD* pathfinding. Inspired by Team 254's obstacle mode
 * switching (teleop/auto/backoff).
 *
 * <p>Usage: - Call setTeleopObstacles() during teleop to avoid other robots - Call
 * setAutoObstacles() during autonomous for safe navigation - Call clearObstacles() to remove all
 * dynamic obstacles
 */
public class ObstacleManager {
  // Obstacle zones (example coordinates - adjust for your field)
  private static final List<ObstacleZone> TELEOP_OBSTACLES = new ArrayList<>();
  private static final List<ObstacleZone> AUTO_OBSTACLES = new ArrayList<>();

  static {
    // Define teleop obstacle zones (e.g., avoid center field during teleop)
    TELEOP_OBSTACLES.add(new ObstacleZone(new Translation2d(8.0, 3.5), 2.0, 1.5));

    // Define auto obstacle zones (e.g., avoid stage during auto)
    AUTO_OBSTACLES.add(new ObstacleZone(new Translation2d(5.5, 4.0), 1.5, 1.5));
  }

  /** Set obstacles for teleop mode (avoid other robots in crowded areas). */
  public static void setTeleopObstacles() {
    applyObstacles(TELEOP_OBSTACLES, "Teleop");
  }

  /** Set obstacles for autonomous mode (safe navigation around fixed structures). */
  public static void setAutoObstacles() {
    applyObstacles(AUTO_OBSTACLES, "Auto");
  }

  /**
   * Set custom obstacles based on alliance.
   *
   * @param blueObstacles Obstacles for blue alliance
   * @param redObstacles Obstacles for red alliance
   */
  public static void setAllianceObstacles(
      List<ObstacleZone> blueObstacles, List<ObstacleZone> redObstacles) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      applyObstacles(redObstacles, "Red Alliance");
    } else {
      applyObstacles(blueObstacles, "Blue Alliance");
    }
  }

  /** Clear all dynamic obstacles. */
  public static void clearObstacles() {
    Pathfinding.setDynamicObstacles(new ArrayList<>(), new Translation2d());
    System.out.println("[ObstacleManager] Cleared all obstacles");
  }

  /**
   * Add a single rectangular obstacle.
   *
   * @param center Center of the obstacle
   * @param width Width (x-direction)
   * @param height Height (y-direction)
   */
  public static void addObstacle(Translation2d center, double width, double height) {
    List<ObstacleZone> obstacles = new ArrayList<>();
    obstacles.add(new ObstacleZone(center, width, height));
    applyObstacles(obstacles, "Custom");
  }

  /**
   * Apply a list of obstacle zones.
   *
   * @param zones List of obstacle zones
   * @param modeName Name of the mode (for logging)
   */
  private static void applyObstacles(List<ObstacleZone> zones, String modeName) {
    List<edu.wpi.first.math.Pair<Translation2d, Translation2d>> obstacles = new ArrayList<>();

    for (ObstacleZone zone : zones) {
      // Create bounding box corners
      Translation2d corner1 =
          new Translation2d(
              zone.center.getX() - zone.width / 2, zone.center.getY() - zone.height / 2);
      Translation2d corner2 =
          new Translation2d(
              zone.center.getX() + zone.width / 2, zone.center.getY() + zone.height / 2);

      obstacles.add(new edu.wpi.first.math.Pair<>(corner1, corner2));
    }

    // Apply to pathfinding system
    Pathfinding.setDynamicObstacles(obstacles, new Translation2d());
    System.out.println("[ObstacleManager] Applied " + zones.size() + " " + modeName + " obstacles");
  }

  /** Helper class for defining rectangular obstacle zones. */
  public static class ObstacleZone {
    public final Translation2d center;
    public final double width;
    public final double height;

    public ObstacleZone(Translation2d center, double width, double height) {
      this.center = center;
      this.width = width;
      this.height = height;
    }
  }

  /**
   * Example: Create obstacles around game pieces to avoid running them over.
   *
   * @param gamePiecePositions List of game piece positions
   * @param radius Buffer radius around each piece
   */
  public static void setGamePieceObstacles(List<Translation2d> gamePiecePositions, double radius) {
    List<ObstacleZone> zones = new ArrayList<>();
    for (Translation2d pos : gamePiecePositions) {
      zones.add(new ObstacleZone(pos, radius * 2, radius * 2));
    }
    applyObstacles(zones, "Game Pieces");
  }
}
