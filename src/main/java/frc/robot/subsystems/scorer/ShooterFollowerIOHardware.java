package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ShooterFollowerIOHardware extends MechanismsIOHardwareBase {

  public ShooterFollowerIOHardware(int motorID) {
    super(motorID, ShooterConstants.FOLLOWER_MOTOR_STATOR_LIMIT,
        ShooterConstants.PEAK_FORWARD_DUTY_CYCLE, ShooterConstants.PEAK_REVERSE_DUTY_CYCLE, "shooterIO/follower/");
    resetSlot0Gains();
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    //PID
    slot0Configs.kP = ShooterConstants.FOLLOWER_kP;
    slot0Configs.kI = ShooterConstants.FOLLOWER_kI;
    slot0Configs.kD = ShooterConstants.FOLLOWER_kD;

    //feed forward values
    slot0Configs.kS = ShooterConstants.FOLLOWER_kS;
    slot0Configs.kV = ShooterConstants.FOLLOWER_kV;
    slot0Configs.kA = ShooterConstants.FOLLOWER_kA;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    motorOutputConfigs.PeakForwardDutyCycle = ShooterConstants.PEAK_FORWARD_DUTY_CYCLE;
    motorOutputConfigs.PeakReverseDutyCycle = ShooterConstants.PEAK_REVERSE_DUTY_CYCLE;

    motor.getConfigurator().apply(talonFXConfigs);
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * ShooterConstants.FOLLOWER_DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * ShooterConstants.FOLLOWER_DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * ShooterConstants.FOLLOWER_DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * ShooterConstants.FOLLOWER_DEGREES_PER_REVOLUTION;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / ShooterConstants.FOLLOWER_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / ShooterConstants.FOLLOWER_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }
}