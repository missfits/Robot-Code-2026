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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IndexerConstants;;


public class IndexerIOHardware {
  private final TalonFX m_indexerMotor;
  private final StatusSignal<AngularVelocity> m_velocitySignal;
  private final StatusSignal<Voltage> m_voltageSignal; 
  private final StatusSignal<Current> m_currentSignal;

    // constructor
  public IndexerIOHardware(int motorID) {
    m_indexerMotor = new TalonFX(motorID);
    m_velocitySignal = m_indexerMotor.getVelocity();
    m_voltageSignal = m_indexerMotor.getMotorVoltage();
    m_currentSignal = m_indexerMotor.getStatorCurrent();

    var talonFXConfigurator = m_indexerMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();

    limitConfigs.StatorCurrentLimit = IndexerConstants.MOTOR_STATOR_LIMIT;
    limitConfigs.StatorCurrentLimitEnable = true;

    talonFXConfigurator.apply(limitConfigs);
  }

    // getters
  public double getVelocity() { //in radians
    return Math.toRadians(m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*IndexerConstants.DEGREES_PER_ROTATION);
  }

  public double getCurrent() {
    return m_currentSignal.refresh().getValue().in(Amps);
  }

  public double getVoltage() {
    return m_voltageSignal.refresh().getValue().in(Volts);
  }

    // setters
  public void motorOff() {
    m_indexerMotor.stopMotor();
  }

  public void setVoltage(double volts) {
    m_indexerMotor.setControl(new VoltageOut(volts));
  }

  public void resetPosition() {
    m_indexerMotor.setPosition(0);
  }

  public void setVelocityVoltage(VelocityVoltage request) {
    m_indexerMotor.setControl(request);
  }
}
