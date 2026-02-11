package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class PivotIOHardware extends MechanismsIOHardwareBase {

  public PivotIOHardware(int motorID) {
    super(motorID, PivotConstants.MOTOR_STATOR_LIMIT,
        PivotConstants.PEAK_FORWARD_DUTY_CYCLE, PivotConstants.PEAK_REVERSE_DUTY_CYCLE, "pivot/");
    resetSlot0Gains();
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * PivotConstants.DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * PivotConstants.DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * PivotConstants.DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * PivotConstants.DEGREES_PER_REVOLUTION;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / PivotConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / PivotConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public double degreesToMotorRevolutions(double degrees) {
    return degrees / PivotConstants.DEGREES_PER_REVOLUTION;
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    //PID
    slot0Configs.kP = PivotConstants.kP;
    slot0Configs.kI = PivotConstants.kI;
    slot0Configs.kD = PivotConstants.kD;

    //feed forward values
    slot0Configs.kS = PivotConstants.kS;
    slot0Configs.kV = PivotConstants.kV;
    slot0Configs.kA = PivotConstants.kA;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = PivotConstants.CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = PivotConstants.ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = PivotConstants.JERK;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    motorOutputConfigs.PeakForwardDutyCycle = PivotConstants.PEAK_FORWARD_DUTY_CYCLE;
    motorOutputConfigs.PeakReverseDutyCycle = PivotConstants.PEAK_REVERSE_DUTY_CYCLE;

    motor.getConfigurator().apply(talonFXConfigs);
  }
}