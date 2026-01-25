// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

    limitConfigs.StatorCurrentLimit = ScorerConstants.INDEXER_MOTOR_ID;
    limitConfigs.StatorCurrentLimitEnable = true;

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

  public void setVoltage(double volts) {
    m_indexerMotor.setControl(new VoltageOut(volts));
  }

  public void setVelocityVoltage(double velocity) {
    m_indexerMotor.setControl(new VelocityVoltage(velocity));
  }  

  public void resetPosition() {
    m_indexerMotor.setPosition(0);
  }

  public void setVelocityVoltage(VelocityVoltage request) {
    m_indexerMotor.setControl(request);
  }
}
