package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ShooterIOHardware extends MechanismsIOHardwareBase {

  private final ShooterMotorType type;

  public ShooterIOHardware(ShooterMotorType type) {
    super(type.id, type.statorLimit, type.logPrefix);
    this.type = type;
    resetSlot0Gains();
  }

  public void resetSlot0Gains() {
    var configs = new TalonFXConfiguration();
    var slot0 = configs.Slot0;

    slot0.kP = type.gains.kP();
    slot0.kI = type.gains.kI();
    slot0.kD = type.gains.kD();
    slot0.kS = type.gains.kS();
    slot0.kV = type.gains.kV();
    slot0.kA = type.gains.kA();

    motor.getConfigurator().apply(configs);
  }

  private double getDegreesPerRevolution() {
    return ScorerConstants.SHOOTER_DEGREES_PER_REVOLUTION;
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

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / getDegreesPerRevolution();
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / getDegreesPerRevolution();
    setPositionRevolutions(revolutions);
  }
}