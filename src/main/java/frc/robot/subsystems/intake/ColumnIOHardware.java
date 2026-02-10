package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.ColumnWheelsConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ColumnIOHardware extends MechanismsIOHardwareBase {

  public ColumnIOHardware(int motorID) {
    super(motorID, ColumnWheelsConstants.MOTOR_STATOR_LIMIT, ColumnWheelsConstants.PEAK_FORWARD_DUTY_CYCLE, ColumnWheelsConstants.PEAK_REVERSE_DUTY_CYCLE, "column/");
    resetSlot0Gains();
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * ColumnWheelsConstants.DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * ColumnWheelsConstants.DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * ColumnWheelsConstants.DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * ColumnWheelsConstants.DEGREES_PER_REVOLUTION;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / ColumnWheelsConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / ColumnWheelsConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public double degreesToMotorRevolutions(double degrees) {
    return degrees / ColumnWheelsConstants.DEGREES_PER_REVOLUTION;
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    //PID
    slot0Configs.kP = ColumnWheelsConstants.kP;
    slot0Configs.kI = ColumnWheelsConstants.kI;
    slot0Configs.kD = ColumnWheelsConstants.kD;

    //feed forward values
    slot0Configs.kS = ColumnWheelsConstants.kS;
    slot0Configs.kV = ColumnWheelsConstants.kV;
    slot0Configs.kA = ColumnWheelsConstants.kA;

    motor.getConfigurator().apply(talonFXConfigs);
  }
}