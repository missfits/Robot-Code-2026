package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class IndexerIOHardware extends MechanismsIOHardwareBase {

  public IndexerIOHardware(int motorID) {
    super(motorID, IndexerConstants.MOTOR_STATOR_LIMIT,
        IndexerConstants.PEAK_FORWARD_DUTY_CYCLE, IndexerConstants.PEAK_REVERSE_DUTY_CYCLE, "indexerIO/");
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

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    var currentLimitsConfigs = talonFXConfigs.CurrentLimits;
    currentLimitsConfigs.StatorCurrentLimit = IndexerConstants.MOTOR_STATOR_LIMIT;
    currentLimitsConfigs.StatorCurrentLimitEnable = true; 

    motor.getConfigurator().apply(talonFXConfigs);
  }
}