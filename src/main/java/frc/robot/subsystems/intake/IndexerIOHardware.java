package frc.robot.subsystems.intake;

import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class IndexerIOHardware extends MechanismsIOHardwareBase {

  public IndexerIOHardware(int motorID) {
    super(motorID, "indexer/");
    resetConfigs();
  }

  public void resetConfigs() {
    resetSlot0Gains();

    motorConfigs.MotorOutput.PeakForwardDutyCycle = IndexerConstants.PEAK_FORWARD_DUTY_CYCLE;
    motorConfigs.MotorOutput.PeakReverseDutyCycle = IndexerConstants.PEAK_REVERSE_DUTY_CYCLE;

    motorConfigs.CurrentLimits.StatorCurrentLimit = IndexerConstants.MOTOR_STATOR_LIMIT;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(motorConfigs);
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * IndexerConstants.DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * IndexerConstants.DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * IndexerConstants.DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * IndexerConstants.DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRotationsPerSecond() {
    return getVelocityDegreesPerSecond() / 360.0;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / IndexerConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / IndexerConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void resetSlot0Gains() {
    var slot0Configs = motorConfigs.Slot0;

    //PID
    slot0Configs.kP = IndexerConstants.kP;
    slot0Configs.kI = IndexerConstants.kI;
    slot0Configs.kD = IndexerConstants.kD;

    //feed forward values
    slot0Configs.kS = IndexerConstants.kS;
    slot0Configs.kV = IndexerConstants.kV;
    slot0Configs.kA = IndexerConstants.kA;

    motor.getConfigurator().apply(motorConfigs);
  }
}