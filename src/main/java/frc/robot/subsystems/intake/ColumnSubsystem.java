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
  private final ColumnIOHardware m_influencerIO = new ColumnIOHardware(ColumnMotorType.INFLUENCER);
  private final ColumnIOHardware m_followerIO = new ColumnIOHardware(ColumnMotorType.FOLLOWER);

  public ColumnSubsystem() {
    super("column");
    m_influencerIO.resetPosition();
    m_followerIO.resetPosition();
    m_followerIO.followMotor(m_influencerIO, false);
  }

  protected void setVoltage(double volts) {
    m_influencerIO.setVoltage(volts);
  }

  @Override
  protected void runClosedLoopVelocity(double velocity) {
    VelocityVoltage request = new VelocityVoltage(velocity)
    .withEnableFOC(ColumnConstants.ENABLE_FOC)
    .withFeedForward(ColumnConstants.FEED_FORWARD)
    .withSlot(ColumnConstants.SLOT)
    .withOverrideBrakeDurNeutral(ColumnConstants.OVERRIDE_BRAKE_DUR_NEUTRAL);
    m_influencerIO.setVelocityVoltage(request);
  }

  public void resetControllers() {
    m_influencerIO.resetSlot0Gains();
    m_followerIO.resetSlot0Gains();
  }

  public void resetPosition() {
    m_influencerIO.resetPosition();
    m_followerIO.resetPosition();
  }

  // Commands
  public Command columnVoltageCommand() {
    return voltageCommand(ColumnConstants.COLUMN_VOLTAGE).withName("columnVoltageCommand");
  }

  public Command columnBackVoltageCommand() {
    return voltageCommand(-ColumnConstants.COLUMN_VOLTAGE).withName("columnBackVoltageCommand");
  }

  public Command columnVelocityCommand() {
    return velocityCommand(ColumnConstants.COLUMN_VELOCITY).withName("columnVelocityCommand");
  }

  private boolean isMotorVelocityOverPercentTolerance(double currentVelocity, double targetVelocity) {
    double thresholdVelocity = targetVelocity * ColumnConstants.AT_VELOCITY_DETECTION_PERCENTAGE;
    return targetVelocity >= 0
      ? currentVelocity > thresholdVelocity
      : currentVelocity < thresholdVelocity;
  }

  public Trigger isMotorVelocityOverPercentToleranceTrigger(Supplier<Double> targetVelocitySupplier) {
    return new Trigger(() -> isMotorVelocityOverPercentTolerance(
      m_influencerIO.getMotorVelocityRevolutionsPerSecond(),
      targetVelocitySupplier.get()
    ));
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("column/influencer/actualStatorCurrent", m_influencerIO.getStatorCurrent());
    SmartDashboard.putNumber("column/influencer/actualSupplyCurrent", m_influencerIO.getSupplyCurrent());
    SmartDashboard.putNumber("column/influencer/actualPositionDegrees", m_influencerIO.getPositionDegrees());
    SmartDashboard.putNumber("column/influencer/actualVelocityRotationsPerSecond", m_influencerIO.getVelocityRotationsPerSecond());
    SmartDashboard.putNumber("column/influencer/actualVelocityDegreesPerSecond", m_influencerIO.getVelocityDegreesPerSecond());
    SmartDashboard.putNumber("column/influencer/actualVoltage", m_influencerIO.getVoltage());

    SmartDashboard.putNumber("column/follower/actualStatorCurrent", m_followerIO.getStatorCurrent());
    SmartDashboard.putNumber("column/follower/actualSupplyCurrent", m_followerIO.getSupplyCurrent());
    SmartDashboard.putNumber("column/follower/actualPositionDegrees", m_followerIO.getPositionDegrees());
    SmartDashboard.putNumber("column/follower/actualVelocityRotationsPerSecond", m_followerIO.getVelocityRotationsPerSecond());
    SmartDashboard.putNumber("column/follower/actualVelocityDegreesPerSecond", m_followerIO.getVelocityDegreesPerSecond());
    SmartDashboard.putNumber("column/follower/actualVoltage", m_followerIO.getVoltage());
  }
}