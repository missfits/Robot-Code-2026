package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;
import frc.robot.utils.MechanismUtil;

public class PivotIOHardware extends MechanismsIOHardwareBase {

  public PivotIOHardware(int motorID) {
    super(motorID, "pivot/");
    resetConfigs();
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

  public double getVelocityRotationsPerSecond() {
    return getVelocityDegreesPerSecond() / 360.0;
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
  public void setVelocityVoltage(double velocityRevolutionsPerSecond) {
    velocityRevolutionsPerSecond = MechanismUtil.clamp(velocityRevolutionsPerSecond, getPositionDegrees(), PivotConstants.STORE_POSITION_DEGREES, PivotConstants.DEPLOY_POSITION_DEGREES,
        -PivotConstants.MAX_VELOCITY, PivotConstants.MAX_VELOCITY, 0, 0);
    SmartDashboard.putNumber(logPrefix + "targetVelocityRotationsPerSecond", velocityRevolutionsPerSecond);
    motor.setControl(new VelocityVoltage(velocityRevolutionsPerSecond));
  }

  @Override
  public void setVelocityVoltage(VelocityVoltage request) {
    request.Velocity = MechanismUtil.clamp(request.Velocity, getPositionDegrees(), PivotConstants.STORE_POSITION_DEGREES, PivotConstants.DEPLOY_POSITION_DEGREES,
        -PivotConstants.MAX_VELOCITY, PivotConstants.MAX_VELOCITY, 0, 0);
    motor.setControl(request);
  }

  @Override
  public void goToPositionProfiled(MotionMagicVoltage request) {
    motor.setControl(request.withFeedForward(getGravityFeedForward(getPositionDegrees())));
  }

  // calculate gravity feedforward from arm position in degrees
  private double getGravityFeedForward(double position) {
    return PivotConstants.kG * Math.cos(Math.toRadians(position + PivotConstants.GRAVITY_FEEDFORWARD_OFFSET));
  }

  public void resetConfigs() {
    resetSlot0Gains();

    motorConfigs.MotorOutput.PeakForwardDutyCycle = PivotConstants.PEAK_FORWARD_DUTY_CYCLE;
    motorConfigs.MotorOutput.PeakReverseDutyCycle = PivotConstants.PEAK_REVERSE_DUTY_CYCLE;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfigs.CurrentLimits.StatorCurrentLimit = PivotConstants.MOTOR_STATOR_LIMIT;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(motorConfigs);
    setInverted(PivotConstants.IS_INVERTED);
    setNeutralMode(NeutralModeValue.Brake);
  }

  public void resetSlot0Gains() {
    var slot0Configs = motorConfigs.Slot0;

    //PID
    slot0Configs.kP = PivotConstants.kP;
    slot0Configs.kI = PivotConstants.kI;
    slot0Configs.kD = PivotConstants.kD;

    //feed forward values
    slot0Configs.kS = PivotConstants.kS;
    slot0Configs.kV = PivotConstants.kV;
    slot0Configs.kA = PivotConstants.kA;

    var motionMagicConfigs = motorConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = PivotConstants.CRUISE_VELOCITY / PivotConstants.DEGREES_PER_REVOLUTION * 360;
    motionMagicConfigs.MotionMagicAcceleration = PivotConstants.ACCELERATION / PivotConstants.DEGREES_PER_REVOLUTION * 360;
    motionMagicConfigs.MotionMagicJerk = PivotConstants.JERK / PivotConstants.DEGREES_PER_REVOLUTION * 360;

    motor.getConfigurator().apply(motorConfigs);
  }
}