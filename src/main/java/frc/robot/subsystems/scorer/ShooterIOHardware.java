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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;


public class ShooterIOHardware {
  private final TalonFX m_shooterMotor = new TalonFX(ShooterConstants.MECHANISM_MOTOR_ID);

  private final StatusSignal<AngularVelocity> m_velocitySignal = m_shooterMotor.getVelocity();
  private final StatusSignal<Current> m_currentSignal = m_shooterMotor.getStatorCurrent();

  // constructor
  public ShooterIOHardware() {
    var talonFXConfigurator = m_shooterMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();

    limitConfigs.StatorCurrentLimit = ShooterConstants.MOTOR_STATOR_LIMIT;
    limitConfigs.StatorCurrentLimitEnable = true;

    talonFXConfigurator.apply(limitConfigs);
  }

  // getters
  public double getVelocity() { //in radians
    return Math.toRadians(m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*ShooterConstants.DEGREES_PER_ROTATION);
  }

  public double getCurrent() {
    return m_currentSignal.refresh().getValue().in(Amps);
  }

  // setters
  public void motorOff() {
    m_shooterMotor.stopMotor();
  }

  public void setVoltage(double value) {
    m_shooterMotor.setControl(new VoltageOut(value));
    SmartDashboard.putNumber("shooter/voltage", value);
  }

  public void setPosition(double value){
    m_shooterMotor.setPosition(value);
  }

  public void resetPosition() {
        setPosition(0);
    }
  
  public void setVelocityVoltage(VelocityVoltage request) {
    m_shooterMotor.setControl(request);
}
}