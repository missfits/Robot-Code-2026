package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class MechanismsIOHardwareBase {
  protected final TalonFX motor;
  protected final String logPrefix;

  protected final StatusSignal<Angle> positionSignal;
  protected final StatusSignal<AngularVelocity> velocitySignal;
  protected final StatusSignal<Voltage> voltageSignal;
  protected final StatusSignal<Current> currentSignal;

  private double targetVelocity = 0.0;

  protected MechanismsIOHardwareBase(int motorID, double statorCurrentLimit,
      double peakForwardDutyCycle, double peakReverseDutyCycle, String logPrefix) {
    motor = new TalonFX(motorID);
    this.logPrefix = logPrefix;

    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    currentSignal = motor.getStatorCurrent();

    var limits = new CurrentLimitsConfigs();
    limits.StatorCurrentLimit = statorCurrentLimit;
    limits.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(limits);

    var motorOutput = new MotorOutputConfigs();
    motorOutput.PeakForwardDutyCycle = peakForwardDutyCycle;
    motorOutput.PeakReverseDutyCycle = peakReverseDutyCycle;
    motor.getConfigurator().apply(motorOutput);
  }

  protected double getPositionRevolutions() {
    return positionSignal.refresh().getValue().in(Revolutions);
  }

  public double getVoltage() {
    return voltageSignal.refresh().getValue().in(Volts);
  }

  protected double getMotorVelocityRevolutionsPerSecond() {
    return velocitySignal.refresh().getValue().in(RevolutionsPerSecond);
  }

  public double getCurrent() {
    return currentSignal.refresh().getValue().in(Amps);
  }

  public void motorOff() {
    motor.stopMotor();
  }

  protected void setPositionRevolutions(double revolutions) {
    motor.setPosition(revolutions);
  }

  public void resetPosition() {
    setPositionRevolutions(0);
  }

  public void setVoltage(double volts) {
    SmartDashboard.putNumber(logPrefix + "commandedVoltage", volts);
    motor.setControl(new VoltageOut(volts));
  }

  public void setVelocityVoltage(double velocityRevolutionsPerSecond) {
    targetVelocity = velocityRevolutionsPerSecond;
    SmartDashboard.putNumber(logPrefix + "targetVelocityRevolutionsPerSecond", velocityRevolutionsPerSecond);
    motor.setControl(new VelocityVoltage(velocityRevolutionsPerSecond));
  }

  public void setVelocityVoltage(VelocityVoltage request) {
    motor.setControl(request);
  }

  public void followMotor(MechanismsIOHardwareBase influencerIO, boolean aligned){
    motor.setControl(new Follower(influencerIO.motor.getDeviceID(), 
      aligned ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed));
  }

  public void goToPositionProfiled(MotionMagicVoltage request) {
    motor.setControl(request);
  }

  /**
   * Checks if the mechanism has reached the target velocity within tolerance.
   *
   * @param tolerance the velocity tolerance in revolutions per second
   * @return true if the current velocity is within tolerance of the target velocity
   */
  public boolean atTargetVelocity(double tolerance) {
    double currentVelocity = getMotorVelocityRevolutionsPerSecond();
    return Math.abs(currentVelocity - targetVelocity) <= tolerance;
  }

  /**
   * Creates a trigger that is true when the mechanism has reached the target velocity.
   *
   * @param tolerance the velocity tolerance in revolutions per second
   * @return a Trigger that activates when at target velocity
   */
  public Trigger atTargetVelocityTrigger(double tolerance) {
    return new Trigger(() -> atTargetVelocity(tolerance));
  }
}