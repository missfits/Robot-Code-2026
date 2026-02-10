package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.IntakePivotConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class PivotIOHardware extends MechanismsIOHardwareBase {

  public PivotIOHardware(int motorID) {
    super(motorID, IntakePivotConstants.MOTOR_STATOR_LIMIT,
        IntakePivotConstants.PEAK_FORWARD_DUTY_CYCLE, IntakePivotConstants.PEAK_REVERSE_DUTY_CYCLE, "pivot/");
    resetSlot0Gains();
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * IntakePivotConstants.DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * IntakePivotConstants.DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * IntakePivotConstants.DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * IntakePivotConstants.DEGREES_PER_REVOLUTION;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / IntakePivotConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / IntakePivotConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public double degreesToMotorRevolutions(double degrees) {
    return degrees / IntakePivotConstants.DEGREES_PER_REVOLUTION;
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    //PID
    slot0Configs.kP = IntakePivotConstants.kP;
    slot0Configs.kI = IntakePivotConstants.kI;
    slot0Configs.kD = IntakePivotConstants.kD;

    //feed forward values
    slot0Configs.kS = IntakePivotConstants.kS;
    slot0Configs.kV = IntakePivotConstants.kV;
    slot0Configs.kA = IntakePivotConstants.kA;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = IntakePivotConstants.CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = IntakePivotConstants.ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = IntakePivotConstants.JERK;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    motorOutputConfigs.PeakForwardDutyCycle = IntakePivotConstants.PEAK_FORWARD_DUTY_CYCLE;
    motorOutputConfigs.PeakReverseDutyCycle = IntakePivotConstants.PEAK_REVERSE_DUTY_CYCLE;

    motor.getConfigurator().apply(talonFXConfigs);
  }
}