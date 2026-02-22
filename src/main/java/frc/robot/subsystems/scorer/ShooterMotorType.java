package frc.robot.subsystems.scorer;

import frc.robot.Constants.ShooterConstants;

public enum ShooterMotorType {
  INFLUENCER (
      ShooterConstants.INFLUENCER_MOTOR_ID,
      ShooterConstants.INFLUENCER_MOTOR_STATOR_LIMIT,
      "shooter/influencer/",
      new Gains(
          ShooterConstants.INFLUENCER_kP,
          ShooterConstants.INFLUENCER_kI,
          ShooterConstants.INFLUENCER_kD,
          ShooterConstants.INFLUENCER_kS,
          ShooterConstants.INFLUENCER_kV,
          ShooterConstants.INFLUENCER_kA
      )
  ),
  FOLLOWER (
      ShooterConstants.FOLLOWER_MOTOR_ID,
      ShooterConstants.FOLLOWER_MOTOR_STATOR_LIMIT,
      "shooter/follower/",
      new Gains(
          ShooterConstants.FOLLOWER_kP,
          ShooterConstants.FOLLOWER_kI,
          ShooterConstants.FOLLOWER_kD,
          ShooterConstants.FOLLOWER_kS,
          ShooterConstants.FOLLOWER_kV,
          ShooterConstants.FOLLOWER_kA
      )
  );

  public final int id;
  public final int statorLimit;
  public final String logPrefix;
  public final Gains gains;

  ShooterMotorType(int id, int statorLimit, String logPrefix, Gains gains) {
    this.id = id;
    this.statorLimit = statorLimit;
    this.logPrefix = logPrefix;
    this.gains = gains;
  }

  /**
   * Record to group PID and feedforward gains.
   */
  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
