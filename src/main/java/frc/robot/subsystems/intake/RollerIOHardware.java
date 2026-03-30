package frc.robot.subsystems.intake;

import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class RollerIOHardware extends MechanismsIOHardwareBase {

  public RollerIOHardware(int motorID) {
    super(motorID, "roller/");
    resetConfigs();
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * RollerConstants.DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * RollerConstants.DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * RollerConstants.DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * RollerConstants.DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRotationsPerSecond() {
    return getVelocityDegreesPerSecond() / 360.0;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / RollerConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / RollerConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void resetConfigs() {
    resetSlot0Gains();

    motorConfigs.MotorOutput.PeakForwardDutyCycle = RollerConstants.PEAK_FORWARD_DUTY_CYCLE;
    motorConfigs.MotorOutput.PeakReverseDutyCycle = RollerConstants.PEAK_REVERSE_DUTY_CYCLE;

    motorConfigs.CurrentLimits.StatorCurrentLimit = RollerConstants.MOTOR_STATOR_LIMIT;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(motorConfigs);
    setInverted(RollerConstants.IS_INVERTED);
  }

  public void resetSlot0Gains() {
    var slot0Configs = motorConfigs.Slot0;

    //PID
    slot0Configs.kP = RollerConstants.kP;
    slot0Configs.kI = RollerConstants.kI;
    slot0Configs.kD = RollerConstants.kD;

    //feed forward
    slot0Configs.kS = RollerConstants.kS;
    slot0Configs.kV = RollerConstants.kV;
    slot0Configs.kA = RollerConstants.kA;

    motor.getConfigurator().apply(motorConfigs);
  }
}