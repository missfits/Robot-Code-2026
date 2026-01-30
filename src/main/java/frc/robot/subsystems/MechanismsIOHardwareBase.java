package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.*;

public abstract class MechanismsIOHardwareBase {
  protected final TalonFX motor;

  protected final StatusSignal<Angle> positionSignal;
  protected final StatusSignal<AngularVelocity> velocitySignal;
  protected final StatusSignal<Voltage> voltageSignal;
  protected final StatusSignal<Current> currentSignal;

  protected MechanismsIOHardwareBase(int motorID, double statorCurrentLimit) {
    motor = new TalonFX(motorID);

    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    currentSignal = motor.getStatorCurrent();

    var limits = new CurrentLimitsConfigs();
    limits.StatorCurrentLimit = statorCurrentLimit;
    limits.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(limits);
  }

  protected double getRotations() {
    return positionSignal.refresh().getValue().in(Revolutions);
  }

  protected double getRotationsPerSecond() {
    return velocitySignal.refresh().getValue().in(RevolutionsPerSecond);
  }

  public double getVoltage() {
    return voltageSignal.refresh().getValue().in(Volts);
  }

  public double getCurrent() {
    return currentSignal.refresh().getValue().in(Amps);
  }

  public void motorOff() {
    motor.stopMotor();
  }

  public void setPositionRotations(double rotations) {
    motor.setPosition(rotations);
  }

  public void resetPosition() {
    setPositionRotations(0);
  }

  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }

  public void setVelocityVoltage(double velocityRPS) {
    motor.setControl(new VelocityVoltage(velocityRPS));
  }

  public void setVelocityVoltage(VelocityVoltage request) {
    motor.setControl(request);
  }
}