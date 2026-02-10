package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class PivotIOHardware extends MechanismsIOHardwareBase {

  public PivotIOHardware(int motorID) {
    super(motorID, IntakeConstants.PIVOT_MOTOR_STATOR_LIMIT,
        IntakeConstants.PEAK_FORWARD_DUTY_CYCLE, IntakeConstants.PEAK_REVERSE_DUTY_CYCLE, "pivot/");
    resetSlot0Gains();
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * IntakeConstants.PIVOT_DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * IntakeConstants.PIVOT_DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * IntakeConstants.PIVOT_DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * IntakeConstants.PIVOT_DEGREES_PER_REVOLUTION;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / IntakeConstants.PIVOT_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / IntakeConstants.PIVOT_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public double degreesToMotorRevolutions(double degrees) {
    return degrees / IntakeConstants.PIVOT_DEGREES_PER_REVOLUTION;
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    
    //PID
    slot0Configs.kP = IntakeConstants.PIVOT_kP;
    slot0Configs.kI = IntakeConstants.PIVOT_kI;
    slot0Configs.kD = IntakeConstants.PIVOT_kD;

    //feed forward values
    slot0Configs.kS = IntakeConstants.PIVOT_kS;
    slot0Configs.kV = IntakeConstants.PIVOT_kV;
    slot0Configs.kA = IntakeConstants.PIVOT_kA;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.PIVOT_CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.PIVOT_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = IntakeConstants.PIVOT_JERK;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    motorOutputConfigs.PeakForwardDutyCycle = IntakeConstants.PEAK_FORWARD_DUTY_CYCLE;
    motorOutputConfigs.PeakReverseDutyCycle = IntakeConstants.PEAK_REVERSE_DUTY_CYCLE;

    motor.getConfigurator().apply(talonFXConfigs);
  }
}