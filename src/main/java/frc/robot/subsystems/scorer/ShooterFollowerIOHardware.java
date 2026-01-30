package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ShooterFollowerIOHardware extends MechanismsIOHardwareBase {

  public ShooterFollowerIOHardware(int motorID) {
    super(motorID, ScorerConstants.FOLLOWER_MOTOR_STATOR_LIMIT);
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    
    slot0Configs.kP = ScorerConstants.FOLLOWER_kP;
    slot0Configs.kI = ScorerConstants.FOLLOWER_kI;
    slot0Configs.kD = ScorerConstants.FOLLOWER_kD;
    
    motor.getConfigurator().apply(talonFXConfigs);
  }

  public double getPositionRadians() {
    return Math.toRadians(getRotation() * ScorerConstants.FOLLOWER_DEGREES_PER_ROTATION);
  }

  public double getPositionDegrees() {
    return Math.toDegrees(getRotation() * ScorerConstants.FOLLOWER_DEGREES_PER_ROTATION);
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRPS() * ScorerConstants.FOLLOWER_DEGREES_PER_ROTATION);
  }

  public double getVelocityDegreesPerSecond() {
    return Math.toDegrees(getMotorVelocityRPS() * ScorerConstants.FOLLOWER_DEGREES_PER_ROTATION);
  }

  public void setPositionRadians(double radians) {
    double rotations = Math.toRadians(radians / ScorerConstants.FOLLOWER_DEGREES_PER_ROTATION);
    setPositionRotations(rotations);
  }

  public void setPositionDegrees(double degrees) {
    double rotations = degrees / ScorerConstants.FOLLOWER_DEGREES_PER_ROTATION;
    setPositionRotations(rotations);
  }
}