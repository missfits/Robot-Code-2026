package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.BaseStatusSignal;

import frc.robot.Constants.ShooterConstants;

import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ShooterInfluencerIOHardware extends MechanismsIOHardwareBase {

  public ShooterInfluencerIOHardware(int motorID) {
    super(motorID, ShooterConstants.INFLUENCER_MOTOR_STATOR_LIMIT,
        ShooterConstants.PEAK_FORWARD_DUTY_CYCLE, ShooterConstants.PEAK_REVERSE_DUTY_CYCLE, "shooter/influencer/");
    
    BaseStatusSignal.setUpdateFrequencyForAll(200, positionSignal, velocitySignal, voltageSignal, currentSignal);
    resetSlot0Gains();
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    //PID
    slot0Configs.kP = ShooterConstants.INFLUENCER_kP;
    slot0Configs.kI = ShooterConstants.INFLUENCER_kI;
    slot0Configs.kD = ShooterConstants.INFLUENCER_kD;

    //feed forward values
    slot0Configs.kS = ShooterConstants.INFLUENCER_kS;
    slot0Configs.kV = ShooterConstants.INFLUENCER_kV;
    slot0Configs.kA = ShooterConstants.INFLUENCER_kA;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    motorOutputConfigs.PeakForwardDutyCycle = ShooterConstants.PEAK_FORWARD_DUTY_CYCLE;
    motorOutputConfigs.PeakReverseDutyCycle = ShooterConstants.PEAK_REVERSE_DUTY_CYCLE;

    motor.getConfigurator().apply(talonFXConfigs);
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * ShooterConstants.INFLUENCER_DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * ShooterConstants.INFLUENCER_DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * ShooterConstants.INFLUENCER_DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * ShooterConstants.INFLUENCER_DEGREES_PER_REVOLUTION;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / ShooterConstants.INFLUENCER_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / ShooterConstants.INFLUENCER_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }
}