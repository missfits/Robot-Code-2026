package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScorerConstants;

public class IndexerIOHardware {

    private final TalonFX m_indexerMotor;
    private final StatusSignal<Angle> m_positionSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;

    // constructor
    public IndexerIOHardware(int motorID) {
      m_indexerMotor = new TalonFX(motorID);
      m_positionSignal = m_indexerMotor.getPosition();
      m_velocitySignal = m_indexerMotor.getVelocity();
      m_voltageSignal = m_indexerMotor.getMotorVoltage();
      m_currentSignal = m_indexerMotor.getStatorCurrent();

      var talonFXConfigurator = m_indexerMotor.getConfigurator();
      var limitConfigs = new CurrentLimitsConfigs();


      limitConfigs.StatorCurrentLimit = ScorerConstants.INDEXER_MOTOR_STATOR_LIMIT;
      limitConfigs.StatorCurrentLimitEnable = true;

      talonFXConfigurator.apply(limitConfigs);
    }

  // getters
  public double getPosition() {
    return Math.toRadians(m_positionSignal.refresh().getValue().in(Revolutions)*ScorerConstants.INDEXER_DEGREES_PER_ROTATION);
  }

  public double getPositionDegrees() {
    return m_positionSignal.refresh().getValue().in(Revolutions)*ScorerConstants.INDEXER_DEGREES_PER_ROTATION;
  }

  public double getVelocity() { //in radians
    return Math.toRadians(m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*ScorerConstants.INDEXER_DEGREES_PER_ROTATION);
  }

  public double getVelocityDegrees() {
    return m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*ScorerConstants.INDEXER_DEGREES_PER_ROTATION;
  }

  public double getVoltage() {
    return m_voltageSignal.refresh().getValue().in(Volts);
  }

  public double getCurrent() {
    return m_currentSignal.refresh().getValue().in(Amps);
  }

  // setters
  public void motorOff() {
    m_indexerMotor.stopMotor();
  }

  public void setVoltage(double value) {
    m_indexerMotor.setControl(new VoltageOut(value));
    SmartDashboard.putNumber("indexer/voltage", value);
  }

  public void setVelocityVoltage(double value) {
    m_indexerMotor.setControl(new VelocityVoltage(value));
    SmartDashboard.putNumber("indexer/velocity voltage", value);
  }

  public void setPosition(double value){
    m_indexerMotor.setPosition(value);
  }

  public void resetPosition() {
        setPosition(0);
    }
}