package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.BaseStatusSignal;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ShooterIOHardware extends MechanismsIOHardwareBase {

  private final ShooterMotorType type;

  public ShooterIOHardware(ShooterMotorType type) {
    super(type.id, type.logPrefix);
    this.type = type;
    BaseStatusSignal.setUpdateFrequencyForAll(200, positionSignal, velocitySignal, voltageSignal, currentSignal);
    resetConfigs();
  }

  private double getDegreesPerRevolution() {
    return ShooterConstants.SHOOTER_DEGREES_PER_REVOLUTION;
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * getDegreesPerRevolution());
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * getDegreesPerRevolution();
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * getDegreesPerRevolution());
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * getDegreesPerRevolution();
  }

  public double getVelocityRotationsPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * getDegreesPerRevolution() / 360;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / getDegreesPerRevolution();
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / getDegreesPerRevolution();
    setPositionRevolutions(revolutions);
  }

  public void resetConfigs() {
    resetSlot0Gains();

    motorConfigs.MotorOutput.PeakForwardDutyCycle = ShooterConstants.PEAK_FORWARD_DUTY_CYCLE;
    motorConfigs.MotorOutput.PeakReverseDutyCycle = ShooterConstants.PEAK_REVERSE_DUTY_CYCLE;

    motorConfigs.CurrentLimits.StatorCurrentLimit = type.statorLimit;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(motorConfigs);
    setInverted(ShooterConstants.IS_INFLUENCER_INVERTED);
  }

  public void resetSlot0Gains() {
    var slot0 = motorConfigs.Slot0;

    // Get current gains from ShooterConstants to support runtime tuning
    var gains = type.gains();
    slot0.kP = gains.kP();
    slot0.kI = gains.kI();
    slot0.kD = gains.kD();
    slot0.kS = gains.kS();
    slot0.kV = gains.kV();
    slot0.kA = gains.kA();

    motor.getConfigurator().apply(motorConfigs);
  }
}