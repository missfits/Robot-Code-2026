package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ShooterFollowerIOHardware extends MechanismsIOHardwareBase {

  public ShooterFollowerIOHardware(int motorID) {
    super(motorID, ScorerConstants.FOLLOWER_MOTOR_STATOR_LIMIT, "shooter/follower/");
    resetSlot0Gains();
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    
    //PID
    slot0Configs.kP = ScorerConstants.FOLLOWER_kP;
    slot0Configs.kI = ScorerConstants.FOLLOWER_kI;
    slot0Configs.kD = ScorerConstants.FOLLOWER_kD;

    //feed forward values
    slot0Configs.kS = ScorerConstants.FOLLOWER_kS;
    slot0Configs.kV = ScorerConstants.FOLLOWER_kV;
    slot0Configs.kA = ScorerConstants.FOLLOWER_kA;
    
    motor.getConfigurator().apply(talonFXConfigs);
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * ScorerConstants.FOLLOWER_DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * ScorerConstants.FOLLOWER_DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * ScorerConstants.FOLLOWER_DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * ScorerConstants.FOLLOWER_DEGREES_PER_REVOLUTION;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / ScorerConstants.FOLLOWER_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / ScorerConstants.FOLLOWER_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }
}