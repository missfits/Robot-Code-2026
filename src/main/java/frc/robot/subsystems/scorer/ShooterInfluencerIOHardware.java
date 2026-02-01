package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.ScorerConstants;

import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ShooterInfluencerIOHardware extends MechanismsIOHardwareBase {

  public ShooterInfluencerIOHardware(int motorID) {
    super(motorID, ScorerConstants.INFLUENCER_MOTOR_STATOR_LIMIT);
    resetSlot0Gains();
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    
    //PID
    slot0Configs.kP = ScorerConstants.INFLUENCER_kP;
    slot0Configs.kI = ScorerConstants.INFLUENCER_kI;
    slot0Configs.kD = ScorerConstants.INFLUENCER_kD;

    //feed forward values
    slot0Configs.kS = ScorerConstants.INFLUENCER_kS;
    slot0Configs.kV = ScorerConstants.INFLUENCER_kV;
    slot0Configs.kA = ScorerConstants.INFLUENCER_kA;

    motor.getConfigurator().apply(talonFXConfigs);
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * ScorerConstants.INFLUENCER_DEGREES_PER_ROTATION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * ScorerConstants.INFLUENCER_DEGREES_PER_ROTATION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * ScorerConstants.INFLUENCER_DEGREES_PER_ROTATION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * ScorerConstants.INFLUENCER_DEGREES_PER_ROTATION;
  }

  public void setPositionRadians(double radians) {
    double rotations = Math.toRadians(radians / ScorerConstants.INFLUENCER_DEGREES_PER_ROTATION);
    setPositionRotations(rotations);
  }

  public void setPositionDegrees(double degrees) {
    double rotations = degrees / ScorerConstants.INFLUENCER_DEGREES_PER_ROTATION;
    setPositionRotations(rotations);
  }
}