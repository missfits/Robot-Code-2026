package frc.robot.subsystems.scorer;

import frc.robot.Constants.ScorerConstants;

public enum ShooterMotorType {
  INFLUENCER (
      ScorerConstants.INFLUENCER_MOTOR_ID,
      ScorerConstants.INFLUENCER_MOTOR_STATOR_LIMIT,
      "shooter/influencer/",
      new Gains(
          ScorerConstants.INFLUENCER_kP,
          ScorerConstants.INFLUENCER_kI,
          ScorerConstants.INFLUENCER_kD,
          ScorerConstants.INFLUENCER_kS,
          ScorerConstants.INFLUENCER_kV,
          ScorerConstants.INFLUENCER_kA
      ),
      ScorerConstants.INFLUENCER_DEGREES_PER_REVOLUTION
  ),
  FOLLOWER (
      ScorerConstants.FOLLOWER_MOTOR_ID,
      ScorerConstants.FOLLOWER_MOTOR_STATOR_LIMIT,
      "shooter/follower/",
      new Gains(
          ScorerConstants.FOLLOWER_kP,
          ScorerConstants.FOLLOWER_kI,
          ScorerConstants.FOLLOWER_kD,
          ScorerConstants.FOLLOWER_kS,
          ScorerConstants.FOLLOWER_kV,
          ScorerConstants.FOLLOWER_kA
      ),
      ScorerConstants.FOLLOWER_DEGREES_PER_REVOLUTION
  );

  public final int id;
  public final int statorLimit;
  public final String logPrefix;
  public final Gains gains;
  public final double degreesPerRevolution;

  ShooterMotorType(int id, int statorLimit, String logPrefix, Gains gains, double degreesPerRevolution) {
    this.id = id;
    this.statorLimit = statorLimit;
    this.logPrefix = logPrefix;
    this.gains = gains;
    this.degreesPerRevolution = degreesPerRevolution;
  }

  /**
   * Record to group PID and feedforward gains.
   */
  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
