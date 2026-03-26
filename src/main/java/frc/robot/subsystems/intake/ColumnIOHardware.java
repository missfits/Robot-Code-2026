package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ColumnIOHardware extends MechanismsIOHardwareBase {

  public ColumnIOHardware(int motorID) {
    super(motorID, ColumnConstants.MOTOR_STATOR_LIMIT, ColumnConstants.PEAK_FORWARD_DUTY_CYCLE, ColumnConstants.PEAK_REVERSE_DUTY_CYCLE, "columnIO/");
    resetSlot0Gains();
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * ColumnConstants.DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * ColumnConstants.DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * ColumnConstants.DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * ColumnConstants.DEGREES_PER_REVOLUTION;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / ColumnConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / ColumnConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public double degreesToMotorRevolutions(double degrees) {
    return degrees / ColumnConstants.DEGREES_PER_REVOLUTION;
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    //PID
    slot0Configs.kP = ColumnConstants.kP;
    slot0Configs.kI = ColumnConstants.kI;
    slot0Configs.kD = ColumnConstants.kD;

    //feed forward values
    slot0Configs.kS = ColumnConstants.kS;
    slot0Configs.kV = ColumnConstants.kV;
    slot0Configs.kA = ColumnConstants.kA;

    var currentLimitsConfigs = talonFXConfigs.CurrentLimits;
    currentLimitsConfigs.StatorCurrentLimit = ColumnConstants.MOTOR_STATOR_LIMIT;
    currentLimitsConfigs.StatorCurrentLimitEnable = true; 

    motor.getConfigurator().apply(talonFXConfigs);
  }
}