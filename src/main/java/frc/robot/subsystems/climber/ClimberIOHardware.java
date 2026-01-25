// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

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

import frc.robot.Constants.ClimberConstants;


public class ClimberIOHardware {
  private final TalonFX m_climberMotor;
  private final StatusSignal<Angle> m_positionSignal;
  private final StatusSignal<AngularVelocity> m_velocitySignal;
  private final StatusSignal<Voltage> m_voltageSignal;
  private final StatusSignal<Current> m_currentSignal;

  // constructor
  public ClimberIOHardware(int motorID) {
    m_climberMotor = new TalonFX(motorID);
    m_positionSignal = m_climberMotor.getPosition();
    m_velocitySignal = m_climberMotor.getVelocity();
    m_voltageSignal = m_climberMotor.getMotorVoltage();
    m_currentSignal = m_climberMotor.getStatorCurrent();


    var talonFXConfigurator = m_climberMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();

    limitConfigs.StatorCurrentLimit = ClimberConstants.MOTOR_STATOR_LIMIT;
    limitConfigs.StatorCurrentLimitEnable = true;

    talonFXConfigurator.apply(limitConfigs);
  }

  // getters
  public double getPosition() {
    return m_positionSignal.refresh().getValue().in(Revolutions)*ClimberConstants.METERS_PER_ROTATION;
  }

  public double getVelocity() {
    return m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*ClimberConstants.METERS_PER_ROTATION;
  }

  public double getVoltage() {
    return m_voltageSignal.refresh().getValue().in(Volts);
  }

  public double getCurrent() {
    return m_currentSignal.refresh().getValue().in(Amps);
  }

  // setters
  public void motorOff() {
    m_climberMotor.stopMotor();
  }

  public void setPosition(double value) {
    m_climberMotor.setPosition(value);
  }

  public void resetPosition() {
    setPosition(0);
  }

  public void setVoltage(double value) {
    m_climberMotor.setControl(new VoltageOut(value));
    SmartDashboard.putNumber("climber/voltage", value);
  }

  public void setVelocityVoltage(double velocity) {
    m_climberMotor.setControl(new VelocityVoltage(velocity));
  }
}