package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class RollerIOHardware extends MechanismsIOHardwareBase {

  public RollerIOHardware(int motorID) {
    super(motorID, IntakeRollerConstants.MOTOR_STATOR_LIMIT,
        IntakeRollerConstants.PEAK_FORWARD_DUTY_CYCLE, IntakeRollerConstants.PEAK_REVERSE_DUTY_CYCLE, "roller/");
    resetSlot0Gains();
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    //PID
    slot0Configs.kP = IntakeRollerConstants.kP;
    slot0Configs.kI = IntakeRollerConstants.kI;
    slot0Configs.kD = IntakeRollerConstants.kD;

    //feed forward
    slot0Configs.kS = IntakeRollerConstants.kS;
    slot0Configs.kV = IntakeRollerConstants.kV;
    slot0Configs.kA = IntakeRollerConstants.kA;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    motorOutputConfigs.PeakForwardDutyCycle = IntakeRollerConstants.PEAK_FORWARD_DUTY_CYCLE;
    motorOutputConfigs.PeakReverseDutyCycle = IntakeRollerConstants.PEAK_REVERSE_DUTY_CYCLE;

    motor.getConfigurator().apply(talonFXConfigs);
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * IntakeRollerConstants.DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * IntakeRollerConstants.DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * IntakeRollerConstants.DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * IntakeRollerConstants.DEGREES_PER_REVOLUTION;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / IntakeRollerConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / IntakeRollerConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }
}