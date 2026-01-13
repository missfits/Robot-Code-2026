// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanism;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.AngularMechanismConstants;


public class AngularMechanismIOHardware {
  private final TalonFX m_motorName = new TalonFX(AngularMechanismConstants.MECHANISM_MOTOR_ID);
  private final StatusSignal<Angle> m_positionSignal = m_motorName.getPosition();
  private final StatusSignal<AngularVelocity> m_velocitySignal = m_motorName.getVelocity();
  private final StatusSignal<Current> m_currentSignal = m_motorName.getStatorCurrent();

  // constructor
  public AngularMechanismIOHardware() {
    var talonFXConfigurator = m_motorName.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();

    limitConfigs.StatorCurrentLimit = AngularMechanismConstants.MOTOR_STATOR_LIMIT;
    limitConfigs.StatorCurrentLimitEnable = true;

    talonFXConfigurator.apply(limitConfigs);
  }

  // getters
  public double getPosition() {
    return Math.toRadians(m_positionSignal.refresh().getValue().in(Revolutions)*AngularMechanismConstants.DEGREES_PER_ROTATION);
  }

  public double getVelocity() {
    return Math.toRadians(m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*AngularMechanismConstants.DEGREES_PER_ROTATION);
  }

  public double getCurrent() {
    return m_currentSignal.refresh().getValue().in(Amps);
  }

  // setters
  public void motorOff() {
    m_motorName.stopMotor();
  }

  public void setPosition(double value) {
    m_motorName.setPosition(value);
  }

  public void resetPosition() {
    setPosition(0);
  }

  public void setVoltage(double value) {
    m_motorName.setControl(new VoltageOut(value));
    SmartDashboard.putNumber("angular mechanism/voltage", value);
  }

  public void setVelocityVoltage(double value) {
    m_motorName.setControl(new VelocityVoltage(value));
    SmartDashboard.putNumber("angular mechanism/velocity voltage", value);
  }
}