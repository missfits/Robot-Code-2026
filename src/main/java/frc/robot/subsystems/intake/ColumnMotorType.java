package frc.robot.subsystems.intake;

import frc.robot.Constants.ColumnConstants;

public enum ColumnMotorType {
  INFLUENCER (
      ColumnConstants.INFLUENCER_ID,
      ColumnConstants.INFLUENCER_STATOR_LIMIT,
      ColumnConstants.INFLUENCER_SUPPLY_LIMIT,
      "column/influencer/"
  ),
  FOLLOWER (
      ColumnConstants.FOLLOWER_ID,
      ColumnConstants.FOLLOWER_STATOR_LIMIT,
      ColumnConstants.FOLLOWER_SUPPLY_LIMIT,
      "column/follower/"
  );

  public final int id;
  public final int statorLimit;
  public final int supplyLimit;
  public final String logPrefix;

  ColumnMotorType(int id, int statorLimit, int supplyLimit, String logPrefix) {
    this.id = id;
    this.statorLimit = statorLimit;
    this.supplyLimit = supplyLimit;
    this.logPrefix = logPrefix;
  }

  /**
   * Get the current gains for this motor type.
   * Reads directly from ColumnConstants to support runtime tuning.
   */
  public Gains gains() {
    return switch (this) {
      case INFLUENCER -> new Gains(
          ColumnConstants.INFLUENCER_kP,
          ColumnConstants.INFLUENCER_kI,
          ColumnConstants.INFLUENCER_kD,
          ColumnConstants.INFLUENCER_kS,
          ColumnConstants.INFLUENCER_kV,
          ColumnConstants.INFLUENCER_kA
      );
      case FOLLOWER -> new Gains(
          ColumnConstants.FOLLOWER_kP,
          ColumnConstants.FOLLOWER_kI,
          ColumnConstants.FOLLOWER_kD,
          ColumnConstants.FOLLOWER_kS,
          ColumnConstants.FOLLOWER_kV,
          ColumnConstants.FOLLOWER_kA
      );
    };
  }

  /**
   * Record to group PID and feedforward gains.
   */
  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}

