package frc.robot.utils;

import edu.wpi.first.math.MathUtil;

public class MechanismUtil {
  /**
   * Clamps a value, typically velocity or voltage, with position-based constraints.
   *
   * @param value the value to clamp
   * @param currentPosition the current position
   * @param minPosition the minimum allowed position
   * @param maxPosition the maximum allowed position
   * @param minValue the minimum value when position is within bounds
   * @param maxValue the maximum value when position is within bounds
   * @param bypassMinValue the value to allow when position is below minimum
   * @param bypassMaxValue the value to allow when position is above maximum
   * @return the clamped value
   */
  public static double clamp(double value, double currentPosition, double minPosition, double maxPosition,
                      double minValue, double maxValue, double bypassMinValue, double bypassMaxValue) {
    double clampedValue = MathUtil.clamp(value, minValue, maxValue);

    // if position is too low, only allow upward movement
    if (currentPosition < minPosition) {
      clampedValue = MathUtil.clamp(clampedValue, bypassMinValue, maxValue);
    }
    // if position is too high, only allow downward movement
    if (currentPosition > maxPosition) {
      clampedValue = MathUtil.clamp(clampedValue, minValue, bypassMaxValue);
    }

    return clampedValue;
  }
}
