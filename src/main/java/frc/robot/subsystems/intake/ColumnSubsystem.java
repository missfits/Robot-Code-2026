package frc.robot.subsystems.intake;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;

public class ColumnSubsystem extends MechanismsSubsystemBase {
  private final ColumnIOHardware m_IO = new ColumnIOHardware(ColumnConstants.MOTOR_ID);

  public ColumnSubsystem() {
    super("column");
    m_IO.resetPosition();
  }

  protected void setVoltage(double volts) {
    m_IO.setVoltage(volts);
  }

  @Override
  protected void runClosedLoopVelocity(double velocity) {
    VelocityVoltage request = new VelocityVoltage(velocity)
    .withEnableFOC(ColumnConstants.ENABLE_FOC)
    .withFeedForward(ColumnConstants.FEED_FORWARD)
    .withSlot(ColumnConstants.SLOT)
    .withOverrideBrakeDurNeutral(ColumnConstants.OVERRIDE_BRAKE_DUR_NEUTRAL);
    m_IO.setVelocityVoltage(request);
  }

  public void resetControllers() {
    m_IO.resetSlot0Gains();
  }

  public void resetPosition() {
    m_IO.resetPosition();
  }

  // Commands
  public Command columnVoltageCommand() {
    return voltageCommand(ColumnConstants.COLUMN_VOLTAGE).withName("run column voltage");
  }

  public Command columnBackVoltageCommand() {
    return voltageCommand(-ColumnConstants.COLUMN_VOLTAGE).withName("run column back voltage");
  }

  public Command columnVelocityCommand() {
    return velocityCommand(ColumnConstants.COLUMN_VELOCITY).withName("run column velocity");
  }

  private boolean isMotorVelocityOverPercentTolerance(double currentVelocity, double targetVelocity) {
    double thresholdVelocity = targetVelocity * ColumnConstants.AT_VELOCITY_DETECTION_PERCENTAGE;
    return targetVelocity >= 0
      ? currentVelocity > thresholdVelocity
      : currentVelocity < thresholdVelocity;
  }

  public Trigger isMotorVelocityOverPercentToleranceTrigger(Supplier<Double> targetVelocitySupplier) {
    return new Trigger(() -> isMotorVelocityOverPercentTolerance(
      m_IO.getMotorVelocityRevolutionsPerSecond(),
      targetVelocitySupplier.get()
    ));
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("column/actualCurrent", m_IO.getCurrent());
    SmartDashboard.putNumber("column/actualPositionDegrees", m_IO.getPositionDegrees());
    SmartDashboard.putNumber("column/actualVelocityRotationsPerSecond", m_IO.getVelocityRotationsPerSecond());
    SmartDashboard.putNumber("column/actualVelocityDegreesPerSecond", m_IO.getVelocityDegreesPerSecond());
    SmartDashboard.putNumber("column/actualVoltage", m_IO.getVoltage());
  }
}