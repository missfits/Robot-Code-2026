package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.ScorerConstants;

import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ShooterIOHardware extends MechanismsIOHardwareBase {

  private final ShooterMotorType type;

  public ShooterIOHardware(ShooterMotorType type) {
    super(getMotorId(type), getStatorLimit(type), getLogPrefix(type));
    this.type = type;
    resetSlot0Gains();
  }

  private static int getMotorId(ShooterMotorType type) {
    return switch (type) {
      case INFLUENCER -> ScorerConstants.INFLUENCER_MOTOR_ID;
      case FOLLOWER   -> ScorerConstants.FOLLOWER_MOTOR_ID;
    };
  }

  private static int getStatorLimit(ShooterMotorType type){
    return switch (type) {
      case INFLUENCER -> ScorerConstants.INFLUENCER_MOTOR_STATOR_LIMIT;
      case FOLLOWER   -> ScorerConstants.FOLLOWER_MOTOR_STATOR_LIMIT;
    };
  }

  private static String getLogPrefix(ShooterMotorType type) {
    return switch (type) {
      case INFLUENCER -> "shooter/influencer/";
      case FOLLOWER   -> "shooter/follower/";
    };
  }

  public void resetSlot0Gains() {
    var configs = new TalonFXConfiguration();
    var slot0 = configs.Slot0;

    switch (type) {
      case INFLUENCER -> {
        slot0.kP = ScorerConstants.INFLUENCER_kP;
        slot0.kI = ScorerConstants.INFLUENCER_kI;
        slot0.kD = ScorerConstants.INFLUENCER_kD;
        slot0.kS = ScorerConstants.INFLUENCER_kS;
        slot0.kV = ScorerConstants.INFLUENCER_kV;
        slot0.kA = ScorerConstants.INFLUENCER_kA;
      }
      case FOLLOWER -> {
        slot0.kP = ScorerConstants.FOLLOWER_kP;
        slot0.kI = ScorerConstants.FOLLOWER_kI;
        slot0.kD = ScorerConstants.FOLLOWER_kD;
        slot0.kS = ScorerConstants.FOLLOWER_kS;
        slot0.kV = ScorerConstants.FOLLOWER_kV;
        slot0.kA = ScorerConstants.FOLLOWER_kA;
      }
    }

    motor.getConfigurator().apply(configs);
  }

  private double getDegreesPerRevolution() {
    return switch (type) {
      case INFLUENCER -> ScorerConstants.INFLUENCER_DEGREES_PER_REVOLUTION;
      case FOLLOWER   -> ScorerConstants.FOLLOWER_DEGREES_PER_REVOLUTION;
    };
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * getDegreesPerRevolution());
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * getDegreesPerRevolution();
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * getDegreesPerRevolution());
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * getDegreesPerRevolution();
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / getDegreesPerRevolution();
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / getDegreesPerRevolution();
    setPositionRevolutions(revolutions);
  }
}