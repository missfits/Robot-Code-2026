package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class RollerIOHardware extends MechanismsIOHardwareBase {

  public RollerIOHardware(int motorID) {
    super(motorID, IntakeConstants.ROLLER_MOTOR_STATOR_LIMIT,
        IntakeConstants.PEAK_FORWARD_DUTY_CYCLE, IntakeConstants.PEAK_REVERSE_DUTY_CYCLE, "roller/");
    resetSlot0Gains();
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    
    //PID
    slot0Configs.kP = IntakeConstants.ROLLER_kP;
    slot0Configs.kI = IntakeConstants.ROLLER_kI;
    slot0Configs.kD = IntakeConstants.ROLLER_kD;

    //feed forward
    slot0Configs.kS = IntakeConstants.ROLLER_kS;
    slot0Configs.kV = IntakeConstants.ROLLER_kV;
    slot0Configs.kA = IntakeConstants.ROLLER_kA;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    motorOutputConfigs.PeakForwardDutyCycle = IntakeConstants.PEAK_FORWARD_DUTY_CYCLE;
    motorOutputConfigs.PeakReverseDutyCycle = IntakeConstants.PEAK_REVERSE_DUTY_CYCLE;

    motor.getConfigurator().apply(talonFXConfigs);
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * IntakeConstants.ROLLER_DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * IntakeConstants.ROLLER_DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * IntakeConstants.ROLLER_DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * IntakeConstants.ROLLER_DEGREES_PER_REVOLUTION;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / IntakeConstants.ROLLER_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / IntakeConstants.ROLLER_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }
}