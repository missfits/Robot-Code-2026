package frc.robot.subsystems.intake;

import frc.robot.Constants.ColumnConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ColumnIOHardware extends MechanismsIOHardwareBase {

  private final ColumnMotorType type;

  public ColumnIOHardware(ColumnMotorType type) {
    super(type.id, type.logPrefix);
    this.type = type;
    resetConfigs();
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * ColumnConstants.DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * ColumnConstants.DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * ColumnConstants.DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * ColumnConstants.DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRotationsPerSecond() {
    return getVelocityDegreesPerSecond() / 360.0;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / ColumnConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / ColumnConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public double degreesToMotorRevolutions(double degrees) {
    return degrees / ColumnConstants.DEGREES_PER_REVOLUTION;
  }

  public void resetConfigs() {
    resetSlot0Gains();
    setInverted(ColumnConstants.IS_INFLUENCER_INVERTED);

    motorConfigs.MotorOutput.PeakForwardDutyCycle = ColumnConstants.PEAK_FORWARD_DUTY_CYCLE;
    motorConfigs.MotorOutput.PeakReverseDutyCycle = ColumnConstants.PEAK_REVERSE_DUTY_CYCLE;

    motorConfigs.CurrentLimits.StatorCurrentLimit = ColumnConstants.INFLUENCER_STATOR_LIMIT;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(motorConfigs);
  }

  public void resetSlot0Gains() {
    var slot0Configs = motorConfigs.Slot0;

    // Get current gains from ColumnConstants to support runtime tuning
    var gains = type.gains();
    slot0Configs.kP = gains.kP();
    slot0Configs.kI = gains.kI();
    slot0Configs.kD = gains.kD();
    slot0Configs.kS = gains.kS();
    slot0Configs.kV = gains.kV();
    slot0Configs.kA = gains.kA();

    motor.getConfigurator().apply(motorConfigs);
  }
}