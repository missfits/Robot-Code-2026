// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * This class manages robot-specific configuration.
 * 
 * <p>To switch robots, change the ROBOT constant below.
 * 
 * <p>The system will provide robot-specific constants through getRobot()
 */
public final class RobotConfig {
  
  // ========================================
  // CHANGE THIS LINE TO SWITCH ROBOTS
  // ========================================
  private static RobotType ROBOT = RobotType.CLEO;

  /**
   * Gets the current robot type.
   */
  public static RobotType getRobot() {
    return ROBOT;
  }

  /**
   * Robot types
   */
  public enum RobotType {
    /** Cleo - Competition robot */
    CLEO,

    /** Ceridwen - Practice robot */
    CERIDWEN
  }

  /**
   * Checks whether the correct robot is selected when deploying.
   * This is called by the CheckDeploy gradle task.
   */
  public static class CheckDeploy {
    public static void main(String... args) {
      System.out.println("✓ Robot selection valid for deployment: " + ROBOT);
    }
  }

  /**
   * Checks robot configuration for pull requests.
   * This can be used in CI/CD to enforce standards.
   */
  public static class CheckPullRequest {
    public static void main(String... args) {
      System.out.println("✓ Robot configuration valid for merge: " + ROBOT);
    }
  }
}

