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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;


public class RollerIOHardware {
  private final TalonFX m_intakeMotor;

  private final StatusSignal<Angle> m_positionSignal;
  private final StatusSignal<AngularVelocity> m_velocitySignal;
  private final StatusSignal<Voltage> m_voltageSignal;
  private final StatusSignal<Current> m_currentSignal;

  // constructor
  public RollerIOHardware(int motorID) {
    m_intakeMotor = new TalonFX(motorID);
    m_positionSignal = m_intakeMotor.getPosition();
    m_velocitySignal = m_intakeMotor.getVelocity();
    m_voltageSignal = m_intakeMotor.getMotorVoltage();
    m_currentSignal = m_intakeMotor.getStatorCurrent();


    var talonFXConfigurator = m_intakeMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();

    limitConfigs.StatorCurrentLimit = IntakeConstants.ROLLER_MOTOR_STATOR_LIMIT;
    limitConfigs.StatorCurrentLimitEnable = true;

    talonFXConfigurator.apply(limitConfigs);
  }

  // getters
  public double getPosition() {
    return Math.toRadians(m_positionSignal.refresh().getValue().in(Revolutions)*IntakeConstants.ROLLER_DEGREES_PER_ROTATION);
  }

  public double getPositionDegrees() {
    return m_positionSignal.refresh().getValue().in(Revolutions)*IntakeConstants.ROLLER_DEGREES_PER_ROTATION;
  }

  public double getVelocity() { //in radians
    return Math.toRadians(m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*IntakeConstants.ROLLER_DEGREES_PER_ROTATION);
  }

  public double getVelocityDegrees() {
    return m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*IntakeConstants.ROLLER_DEGREES_PER_ROTATION;
  }

  public double getVoltage() {
    return m_voltageSignal.refresh().getValue().in(Volts);
  }

  public double getCurrent() {
    return m_currentSignal.refresh().getValue().in(Amps);
  }

  // setters
  public void motorOff() {
    m_intakeMotor.stopMotor();
  }

  public void setVoltage(double value) {
    m_intakeMotor.setControl(new VoltageOut(value));
    SmartDashboard.putNumber("intake/voltage", value);
  }

  public void setVelocityVoltage(double value) {
    m_intakeMotor.setControl(new VelocityVoltage(value));
    SmartDashboard.putNumber("intake/velocity voltage", value);
  }

  public void setPosition(double value){
    m_intakeMotor.setPosition(value);
  }

  public void resetPosition() {
        setPosition(0);
    }
}