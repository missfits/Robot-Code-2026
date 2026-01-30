package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ShooterInfluencerIOHardware extends MechanismsIOHardwareBase {

  public ShooterInfluencerIOHardware(int motorID) {
    super(motorID, ScorerConstants.INFLUENCER_MOTOR_STATOR_LIMIT);
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    
    slot0Configs.kP = IntakeConstants.ROLLER_kP;
    slot0Configs.kI = IntakeConstants.ROLLER_kI;
    slot0Configs.kD = IntakeConstants.ROLLER_kD;
    
    motor.getConfigurator().apply(talonFXConfigs);
  }

  public double getPositionRadians() {
    return Math.toRadians(getRotation() * ScorerConstants.INFLUENCER_DEGREES_PER_ROTATION);
  }

  public double getPositionDegrees() {
    return getRotation() * ScorerConstants.INFLUENCER_DEGREES_PER_ROTATION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRPS() * ScorerConstants.INFLUENCER_DEGREES_PER_ROTATION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRPS() * ScorerConstants.INFLUENCER_DEGREES_PER_ROTATION;
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