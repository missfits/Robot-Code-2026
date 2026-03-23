package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.ColumnConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ColumnIOHardware extends MechanismsIOHardwareBase {

  private final ColumnMotorType type;

  public ColumnIOHardware(ColumnMotorType type) {
    super(type.id, type.statorLimit,
          ColumnConstants.PEAK_FORWARD_DUTY_CYCLE, ColumnConstants.PEAK_REVERSE_DUTY_CYCLE,
          type.logPrefix);
    this.type = type;
    resetSlot0Gains();
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

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    // Get current gains from ColumnConstants to support runtime tuning
    var gains = type.gains();
    slot0Configs.kP = gains.kP();
    slot0Configs.kI = gains.kI();
    slot0Configs.kD = gains.kD();
    slot0Configs.kS = gains.kS();
    slot0Configs.kV = gains.kV();
    slot0Configs.kA = gains.kA();

    motor.getConfigurator().apply(talonFXConfigs);
  }
}