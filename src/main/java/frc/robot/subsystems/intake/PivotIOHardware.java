package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;
import frc.robot.utils.MechanismUtil;

public class PivotIOHardware extends MechanismsIOHardwareBase {

  public PivotIOHardware(int motorID) {
    super(motorID, PivotConstants.MOTOR_STATOR_LIMIT,
        PivotConstants.PEAK_FORWARD_DUTY_CYCLE, PivotConstants.PEAK_REVERSE_DUTY_CYCLE, "pivotIO/");
    resetSlot0Gains();
    setNeutralMode(NeutralModeValue.Brake);
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

  @Override
  public void setVoltage(double volts) {
    volts = MechanismUtil.clamp(volts, getPositionDegrees(), PivotConstants.STORE_POSITION_DEGREES, PivotConstants.DEPLOY_POSITION_DEGREES,
        -PivotConstants.MAX_VOLTAGE, PivotConstants.MAX_VOLTAGE, 0, 0);
    SmartDashboard.putNumber(logPrefix + "commandedVoltage", volts);
    motor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocityVoltage(double velocityRevolutionsPerSecond) {
    velocityRevolutionsPerSecond = MechanismUtil.clamp(velocityRevolutionsPerSecond, getPositionDegrees(), PivotConstants.STORE_POSITION_DEGREES, PivotConstants.DEPLOY_POSITION_DEGREES,
        -PivotConstants.MAX_VELOCITY, PivotConstants.MAX_VELOCITY, 0, 0);
    SmartDashboard.putNumber(logPrefix + "targetVelocityRevolutionsPerSecond", velocityRevolutionsPerSecond);
    motor.setControl(new VelocityVoltage(velocityRevolutionsPerSecond));
  }

  @Override
  public void setVelocityVoltage(VelocityVoltage request) {
    request.Velocity = MechanismUtil.clamp(request.Velocity, getPositionDegrees(), PivotConstants.STORE_POSITION_DEGREES, PivotConstants.DEPLOY_POSITION_DEGREES,
        -PivotConstants.MAX_VELOCITY, PivotConstants.MAX_VELOCITY, 0, 0);
    motor.setControl(request);
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
    motionMagicConfigs.MotionMagicCruiseVelocity = PivotConstants.CRUISE_VELOCITY / PivotConstants.DEGREES_PER_REVOLUTION * 360;
    motionMagicConfigs.MotionMagicAcceleration = PivotConstants.ACCELERATION / PivotConstants.DEGREES_PER_REVOLUTION * 360;
    motionMagicConfigs.MotionMagicJerk = PivotConstants.JERK / PivotConstants.DEGREES_PER_REVOLUTION * 360;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    motorOutputConfigs.PeakForwardDutyCycle = PivotConstants.PEAK_FORWARD_DUTY_CYCLE;
    motorOutputConfigs.PeakReverseDutyCycle = PivotConstants.PEAK_REVERSE_DUTY_CYCLE;

    motor.getConfigurator().apply(talonFXConfigs);
  }
}