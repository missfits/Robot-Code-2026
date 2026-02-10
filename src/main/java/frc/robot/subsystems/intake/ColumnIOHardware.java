package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ColumnIOHardware extends MechanismsIOHardwareBase {

  public ColumnIOHardware(int motorID) {
    super(motorID, IntakeConstants.COLUMN_MOTOR_STATOR_LIMIT, "column/");
    resetSlot0Gains();
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * IntakeConstants.COLUMN_DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * IntakeConstants.COLUMN_DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * IntakeConstants.COLUMN_DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * IntakeConstants.COLUMN_DEGREES_PER_REVOLUTION;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / IntakeConstants.COLUMN_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / IntakeConstants.COLUMN_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public double degreesToMotorRevolutions(double degrees) {
    return degrees / IntakeConstants.COLUMN_DEGREES_PER_REVOLUTION;
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    
    //PID
    slot0Configs.kP = IntakeConstants.COLUMN_kP;
    slot0Configs.kI = IntakeConstants.COLUMN_kI;
    slot0Configs.kD = IntakeConstants.COLUMN_kD;

    //feed forward values
    slot0Configs.kS = IntakeConstants.COLUMN_kS;
    slot0Configs.kV = IntakeConstants.COLUMN_kV;
    slot0Configs.kA = IntakeConstants.COLUMN_kA;

    motor.getConfigurator().apply(talonFXConfigs);
  }
}