package frc.robot.utils;

import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;

/**
 * Lookup table for mapping distance from hub to shooter velocity.
 * Uses linear interpolation to find the appropriate velocity for a given distance.
 */
public class ShooterLookupTable {

  // Hardcoded distance (meters) to velocity (rotations per second) mappings
  // TODO: Tune these values using Phoenix Tuner X at 0.5m intervals
  private static final Map<Double, Double> DISTANCE_TO_VELOCITY_MAP = new TreeMap<>();
  
  static {
    // Initialize the lookup table with placeholder values
    // Format: distance in meters -> velocity in rotations per second
    DISTANCE_TO_VELOCITY_MAP.put(0.5, 40.);   // 0.5m 
    DISTANCE_TO_VELOCITY_MAP.put(1.0, 40.);   // 1.0m 
    DISTANCE_TO_VELOCITY_MAP.put(1.5, 40.);   // 1.5m 
    DISTANCE_TO_VELOCITY_MAP.put(2.0, 40.);   // 2.0m 
    DISTANCE_TO_VELOCITY_MAP.put(2.5, 45.);   // 2.5m 
    // DISTANCE_TO_VELOCITY_MAP.put(3.0, 45.);   // 3.0m 
    DISTANCE_TO_VELOCITY_MAP.put(3.5, 53.);   // 3.5m 
    DISTANCE_TO_VELOCITY_MAP.put(4.0, 53.);   // 4.0m 
    DISTANCE_TO_VELOCITY_MAP.put(4.5, 53.);   // 4.5m 
    DISTANCE_TO_VELOCITY_MAP.put(5.0, 53.);   // 5.0m 
    DISTANCE_TO_VELOCITY_MAP.put(5.5, 53.);   // 5.5m 
    DISTANCE_TO_VELOCITY_MAP.put(6.0, 53.);   // 6.0m 

  }
  
  /**
   * Gets the target shooter velocity for a given distance from the hub.
   * Uses linear interpolation between the closest distance points.
   *
   * @param distance Distance from hub in meters
   * @return Optional containing target velocity in rotations per second,
   *         or empty if distance is outside the table range or table is empty
   */
  public static Optional<Double> getVelocityForDistance(double distance) {
    if (DISTANCE_TO_VELOCITY_MAP.isEmpty()) {
      return Optional.empty();
    }

    TreeMap<Double, Double> map = (TreeMap<Double, Double>) DISTANCE_TO_VELOCITY_MAP;

    // Get the floor and ceiling entries
    Map.Entry<Double, Double> lowerEntry = map.floorEntry(distance);
    Map.Entry<Double, Double> upperEntry = map.ceilingEntry(distance);

    // If distance is outside the range, return empty
    if (lowerEntry == null || upperEntry == null) {
      return Optional.empty();
    }

    // If both entries are the same (exact match), return that value
    if (lowerEntry.getKey().equals(upperEntry.getKey())) {
      return Optional.of(lowerEntry.getValue());
    }

    // Linear interpolation between the two closest points
    double interpolatedVelocity = linearInterpolate(
        lowerEntry.getKey(), lowerEntry.getValue(),
        upperEntry.getKey(), upperEntry.getValue(),
        distance
    );

    return Optional.of(interpolatedVelocity);
  }

  /**
   * Performs linear interpolation between two points.
   *
   * @param x1 Lower x value (distance)
   * @param y1 Lower y value (velocity)
   * @param x2 Upper x value (distance)
   * @param y2 Upper y value (velocity)
   * @param x Target x value (distance)
   * @return Interpolated y value (velocity)
   */
  private static double linearInterpolate(double x1, double y1, double x2, double y2, double x) {
    double ratio = (x - x1) / (x2 - x1);
    return y1 + ratio * (y2 - y1);
  }

  /**
   * Checks if a velocity value exists for the given distance (within min/max range).
   *
   * @param distance Distance from hub in meters
   * @return true if a velocity can be determined for this distance
   */
  public static boolean hasVelocityForDistance(double distance) {
    return getVelocityForDistance(distance).isPresent();
  }
  
  /**
   * Gets the minimum distance in the lookup table.
   * 
   * @return Minimum distance in meters, or null if table is empty
   */
  public static Double getMinDistance() {
    TreeMap<Double, Double> map = (TreeMap<Double, Double>) DISTANCE_TO_VELOCITY_MAP;
    return map.isEmpty() ? null : map.firstKey();
  }
  
  /**
   * Gets the maximum distance in the lookup table.
   * 
   * @return Maximum distance in meters, or null if table is empty
   */
  public static Double getMaxDistance() {
    TreeMap<Double, Double> map = (TreeMap<Double, Double>) DISTANCE_TO_VELOCITY_MAP;
    return map.isEmpty() ? null : map.lastKey();
  }
}

