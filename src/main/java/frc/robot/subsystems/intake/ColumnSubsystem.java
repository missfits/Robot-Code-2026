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
      m_influencerIO.getMotorVelocityRevolutionsPerSecond(),
      targetVelocitySupplier.get()
    ));
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("column influencer IO/live current", m_influencerIO.getCurrent());
    SmartDashboard.putNumber("column influencer IO/live position", m_influencerIO.getPositionDegrees());
    SmartDashboard.putNumber("column influencer IO/live velocity", m_influencerIO.getVelocityDegreesPerSecond());
    SmartDashboard.putNumber("column influencer IO/live voltage", m_influencerIO.getVoltage());

    SmartDashboard.putNumber("column follower IO/live current", m_followerIO.getCurrent());
    SmartDashboard.putNumber("column follower IO/live position", m_followerIO.getPositionDegrees());
    SmartDashboard.putNumber("column follower IO/live velocity", m_followerIO.getVelocityDegreesPerSecond());
    SmartDashboard.putNumber("column follower IO/live voltage", m_followerIO.getVoltage());
  }
}