package frc.robot.utils;

import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ShooterConstants;

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
    DISTANCE_TO_VELOCITY_MAP.put(1.6, 43.);   // prolly too close but can still make some 
    DISTANCE_TO_VELOCITY_MAP.put(2.0, 48.);   
    DISTANCE_TO_VELOCITY_MAP.put(2.25, 49.);
    DISTANCE_TO_VELOCITY_MAP.put(2.5, 52.);
    DISTANCE_TO_VELOCITY_MAP.put(2.8, 53.); 
    DISTANCE_TO_VELOCITY_MAP.put(3.0, 57.);
    DISTANCE_TO_VELOCITY_MAP.put(3.15, 57.);
    DISTANCE_TO_VELOCITY_MAP.put(3.5, 59.);
    DISTANCE_TO_VELOCITY_MAP.put(3.7, 61.);
    DISTANCE_TO_VELOCITY_MAP.put(4.0, 63.);
    DISTANCE_TO_VELOCITY_MAP.put(4.5, 56.); // <<    
    // DISTANCE_TO_VELOCITY_MAP.put(5.1, 63.);   // corner
    // DISTANCE_TO_VELOCITY_MAP.put(5.6, 65.);   // bogus
    // DISTANCE_TO_VELOCITY_MAP.put(100.0, 70.);   // bogus for shuttling
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

    // Calculate multiplier (bogus for calgames based on manual velocities)
      // clamped for distances between tested manual shooter distances
    double multiplier = ShooterConstants.SHOOTER_RPM_BASE_MULTIPLIER * (distance - ShooterConstants.SHOOTER_MANUAL_MIN_MULTIPLIER_DISTANCE);
    multiplier = MathUtil.clamp(multiplier, 1.0, ShooterConstants.SHOOTER_RPM_BASE_MULTIPLIER);

    // If both entries are the same (exact match), return that value
    if (lowerEntry.getKey().equals(upperEntry.getKey())) {
      return Optional.of(lowerEntry.getValue() * multiplier);
    }

    // Linear interpolation between the two closest points
    double interpolatedVelocity = linearInterpolate(
        lowerEntry.getKey(), lowerEntry.getValue(),
        upperEntry.getKey(), upperEntry.getValue(),
        distance
    );

    return Optional.of(interpolatedVelocity * multiplier);
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

