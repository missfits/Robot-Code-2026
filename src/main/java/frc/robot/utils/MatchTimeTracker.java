// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utility class for tracking and displaying match time information.
 * Logs countdown timer and current game period to SmartDashboard.
 *
 * Match structure:
 * - Autonomous: 20 seconds
 * - Transition: 10 seconds
 * - Shift 1: 25 seconds
 * - Shift 2: 25 seconds
 * - Shift 3: 25 seconds
 * - Shift 4: 25 seconds
 * - Endgame: 30 seconds
 * Total teleop time: 140 seconds (2:20)
 */
public class MatchTimeTracker {

  // Teleop period boundaries (time remaining, counting down)
  private static final double ENDGAME_START = 30.0;         // 0-30s: Endgame
  private static final double SHIFT_4_START = 55.0;         // 30-55s: Shift 4
  private static final double SHIFT_3_START = 80.0;         // 55-80s: Shift 3
  private static final double SHIFT_2_START = 105.0;        // 80-105s: Shift 2
  private static final double SHIFT_1_START = 130.0;        // 105-130s: Shift 1
  private static final double TRANSITION_START = 140.0;     // 130-140s: Transition

  /**
   * Updates and logs match time information to SmartDashboard.
   * Should be called periodically (e.g., in robotPeriodic()).
   * This method is safe and will not throw runtime exceptions.
   */
  public static void updateMatchTime() {
    try {
      double matchTime = DriverStation.getMatchTime();

      // Handle invalid values from DriverStation
      if (Double.isNaN(matchTime) || Double.isInfinite(matchTime)) {
        matchTime = -1.0;
      }

      String currentPeriod = getCurrentPeriod(matchTime);

      // Log raw time remaining in seconds
      SmartDashboard.putNumber("matchTime/timeRemaining", matchTime);

      // Format time as MM:SS for easier reading
      String timeString = formatTime(matchTime);
      SmartDashboard.putString("matchTime/countdown", timeString);

      // Log current game period/shift
      SmartDashboard.putString("matchTime/currentPeriod", currentPeriod);
    } catch (Exception e) {
      // Catch any unexpected exceptions to prevent robot loop crashes
      // Log error to SmartDashboard for debugging
      SmartDashboard.putString("matchTime/error", "Error: " + e.getMessage());
      SmartDashboard.putString("matchTime/countdown", "--:--");
      SmartDashboard.putString("matchTime/currentPeriod", "Error");
    }
  }

  /**
   * Determines the current game period based on match time and DriverStation mode.
   *
   * @param matchTime The remaining time in the current period
   * @return String describing the current period
   */
  private static String getCurrentPeriod(double matchTime) {
    if (DriverStation.isDisabled()) {
      return "Disabled";
    } else if (DriverStation.isAutonomous()) {
      return "Autonomous";
    } else if (DriverStation.isTeleop()) {
      // Determine which teleop period we're in based on time remaining
      if (matchTime < 0) {
        return "Post-Match";
      } else if (matchTime <= ENDGAME_START) {
        return "Endgame";
      } else if (matchTime <= SHIFT_4_START) {
        return "Shift 4";
      } else if (matchTime <= SHIFT_3_START) {
        return "Shift 3";
      } else if (matchTime <= SHIFT_2_START) {
        return "Shift 2";
      } else if (matchTime <= SHIFT_1_START) {
        return "Shift 1";
      } else if (matchTime <= TRANSITION_START) {
        return "Transition";
      } else {
        return "Pre-Teleop";
      }
    } else if (DriverStation.isTest()) {
      return "Test";
    } else {
      return "Unknown";
    }
  }

  /**
   * Formats time in seconds to MM:SS format.
   * Safe from runtime exceptions - handles all edge cases.
   *
   * @param timeSeconds Time in seconds
   * @return Formatted time string (e.g., "2:07"), or "--:--" for invalid input
   */
  private static String formatTime(double timeSeconds) {
    // Handle invalid values (NaN, Infinity, negative)
    if (timeSeconds < 0 || Double.isNaN(timeSeconds) || Double.isInfinite(timeSeconds)) {
      return "--:--";
    }

    try {
      int minutes = (int) (timeSeconds / 60);
      int seconds = (int) (timeSeconds % 60);

      // Sanity check to prevent unreasonable values
      if (minutes < 0 || seconds < 0 || minutes > 999) {
        return "--:--";
      }

      return String.format("%d:%02d", minutes, seconds);
    } catch (Exception e) {
      // Fallback in case of any formatting issues
      return "--:--";
    }
  }
}

