package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class RollerIOHardware extends MechanismsIOHardwareBase {

  public RollerIOHardware(int motorID) {
    super(motorID, RollerConstants.MOTOR_STATOR_LIMIT,
        RollerConstants.PEAK_FORWARD_DUTY_CYCLE, RollerConstants.PEAK_REVERSE_DUTY_CYCLE, "rollerIO/");
    resetSlot0Gains();
    setInverted(true);
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    //PID
    slot0Configs.kP = RollerConstants.kP;
    slot0Configs.kI = RollerConstants.kI;
    slot0Configs.kD = RollerConstants.kD;

    //feed forward
    slot0Configs.kS = RollerConstants.kS;
    slot0Configs.kV = RollerConstants.kV;
    slot0Configs.kA = RollerConstants.kA;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    motorOutputConfigs.PeakForwardDutyCycle = RollerConstants.PEAK_FORWARD_DUTY_CYCLE;
    motorOutputConfigs.PeakReverseDutyCycle = RollerConstants.PEAK_REVERSE_DUTY_CYCLE;

    motor.getConfigurator().apply(talonFXConfigs);
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

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / RollerConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / RollerConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }
}