package frc.robot.subsystems.scorer;

import frc.robot.Constants.ShooterConstants;

public enum ShooterMotorType {
  INFLUENCER (
      ShooterConstants.INFLUENCER_MOTOR_ID,
      ShooterConstants.INFLUENCER_MOTOR_STATOR_LIMIT,
      "shooter/influencer/"
  ),
  FOLLOWER (
      ShooterConstants.FOLLOWER_MOTOR_ID,
      ShooterConstants.FOLLOWER_MOTOR_STATOR_LIMIT,
      "shooter/follower/"
  );

  public final int id;
  public final int statorLimit;
  public final String logPrefix;

  ShooterMotorType(int id, int statorLimit, String logPrefix) {
    this.id = id;
    this.statorLimit = statorLimit;
    this.logPrefix = logPrefix;
  }

  /**
   * Get the current gains for this motor type.
   * Reads directly from ShooterConstants to support runtime tuning.
   */
  public Gains gains() {
    return switch (this) {
      case INFLUENCER -> new Gains(
          ShooterConstants.INFLUENCER_kP,
          ShooterConstants.INFLUENCER_kI,
          ShooterConstants.INFLUENCER_kD,
          ShooterConstants.INFLUENCER_kS,
          ShooterConstants.INFLUENCER_kV,
          ShooterConstants.INFLUENCER_kA
      );
      case FOLLOWER -> new Gains(
          ShooterConstants.FOLLOWER_kP,
          ShooterConstants.FOLLOWER_kI,
          ShooterConstants.FOLLOWER_kD,
          ShooterConstants.FOLLOWER_kS,
          ShooterConstants.FOLLOWER_kV,
          ShooterConstants.FOLLOWER_kA
      );
    };
  }

  /**
   * Record to group PID and feedforward gains.
   */
  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
