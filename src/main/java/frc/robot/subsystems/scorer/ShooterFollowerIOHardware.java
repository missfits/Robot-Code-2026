// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ScorerConstants;


public class ShooterFollowerIOHardware {
  private final TalonFX m_shooterFollowerMotor;

  private final StatusSignal<Angle> m_positionSignal;
  private final StatusSignal<AngularVelocity> m_velocitySignal;
  private final StatusSignal<Voltage> m_voltageSignal;
  private final StatusSignal<Current> m_currentSignal;

  // constructor
  public ShooterFollowerIOHardware(int motorID) {
    m_shooterFollowerMotor = new TalonFX(motorID);
    m_positionSignal = m_shooterFollowerMotor.getPosition();
    m_velocitySignal = m_shooterFollowerMotor.getVelocity();
    m_voltageSignal = m_shooterFollowerMotor.getMotorVoltage();
    m_currentSignal = m_shooterFollowerMotor.getStatorCurrent();

    var talonFXConfigurator = m_shooterFollowerMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();

    limitConfigs.StatorCurrentLimit = ScorerConstants.FOLLOWER_MOTOR_STATOR_LIMIT;
    limitConfigs.StatorCurrentLimitEnable = true;

    talonFXConfigurator.apply(limitConfigs);
  }

  // configure pid values on motor
  public void resetSlot0Gains() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = ScorerConstants.FOLLOWER_kP;
    slot0Configs.kI = ScorerConstants.FOLLOWER_kI;
    slot0Configs.kD = ScorerConstants.FOLLOWER_kD;

    slot0Configs.kS = ScorerConstants.FOLLOWER_kS;
    slot0Configs.kV = ScorerConstants.FOLLOWER_kV;
    slot0Configs.kA = ScorerConstants.FOLLOWER_kA;

    m_shooterFollowerMotor.getConfigurator().apply(talonFXConfigs);
  }

  // getters
  public double getPosition() {
    return Math.toRadians(m_positionSignal.refresh().getValue().in(Revolutions)*ScorerConstants.FOLLOWER_DEGREES_PER_ROTATION);
  }

  public double getPositionDegrees() {
    return m_positionSignal.refresh().getValue().in(Revolutions)*ScorerConstants.FOLLOWER_DEGREES_PER_ROTATION;
  }

  public double getVelocity() { //in radians
    return Math.toRadians(m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*ScorerConstants.FOLLOWER_DEGREES_PER_ROTATION);
  }

  public double getVelocityDegrees() {
    return m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*ScorerConstants.FOLLOWER_DEGREES_PER_ROTATION;
  }

  public double getVoltage() {
    return m_voltageSignal.refresh().getValue().in(Volts);
  }

  public double getCurrent() {
    return m_currentSignal.refresh().getValue().in(Amps);
  }

  // setters
  public void motorOff() {
    m_shooterFollowerMotor.stopMotor();
  }

  public void setVoltage(double value) {
    m_shooterFollowerMotor.setControl(new VoltageOut(value));
    SmartDashboard.putNumber("shooter follower/voltage", value);
  }

  public void setPosition(double value){
    m_shooterFollowerMotor.setPosition(value);
  }

  public void resetPosition() {
    m_shooterFollowerMotor.setPosition(0);
  }
  
  public void setVelocityVoltage(double velocity) {
    m_shooterFollowerMotor.setControl(new VelocityVoltage(velocity));
    SmartDashboard.putNumber("shooter influencer/velocity voltage", velocity);
  }

  public void setVelocityVoltage(VelocityVoltage request) {
    m_shooterFollowerMotor.setControl(request);
  }
}