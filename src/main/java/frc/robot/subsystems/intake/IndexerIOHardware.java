package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class IndexerIOHardware extends MechanismsIOHardwareBase {

  public IndexerIOHardware(int motorID) {
    super(motorID, IndexerConstants.MOTOR_STATOR_LIMIT,
        IndexerConstants.PEAK_FORWARD_DUTY_CYCLE, IndexerConstants.PEAK_REVERSE_DUTY_CYCLE, "indexer/");
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * IndexerConstants.DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * IndexerConstants.DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * IndexerConstants.DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * IndexerConstants.DEGREES_PER_REVOLUTION;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / IndexerConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / IndexerConstants.DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    //PID
    slot0Configs.kP = IndexerConstants.kP;
    slot0Configs.kI = IndexerConstants.kI;
    slot0Configs.kD = IndexerConstants.kD;

    //feed forward values
    slot0Configs.kS = IndexerConstants.kS;
    slot0Configs.kV = IndexerConstants.kV;
    slot0Configs.kA = IndexerConstants.kA;

    motor.getConfigurator().apply(talonFXConfigs);
  }
}