package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;


public class IndexerSubsystem extends MechanismsSubsystemBase {
  private final IndexerIOHardware m_IO = new IndexerIOHardware(IndexerConstants.MOTOR_ID);

  public IndexerSubsystem() {
    super("indexer");
    m_IO.resetPosition();
    m_IO.setInverted(true);
  }

  protected void setVoltage(double volts) {
    m_IO.setVoltage(volts);
  }

  @Override
  protected void runClosedLoopVelocity(double velocity) {
    VelocityVoltage request = new VelocityVoltage(velocity)
    .withEnableFOC(IndexerConstants.ENABLE_FOC)
    .withFeedForward(IndexerConstants.FEED_FORWARD)
    .withSlot(IndexerConstants.SLOT)
    .withOverrideBrakeDurNeutral(IndexerConstants.OVERRIDE_BRAKE_DUR_NEUTRAL);
    m_IO.setVelocityVoltage(request);
  }

  public void resetControllers() {
    m_IO.resetSlot0Gains();
  }

  public void resetPosition() {
    m_IO.resetPosition();
  }

  // Commands
  public Command indexerVoltageCommand() {
    return voltageCommand(IndexerConstants.INDEXER_VOLTAGE).withName("run indexer voltage");
  }

  public Command indexerBackVoltageCommand() {
    return voltageCommand(-IndexerConstants.INDEXER_VOLTAGE).withName("run indexer back voltage");
  }

  public Command indexerVelocityCommand() {
    return velocityCommand(IndexerConstants.INDEXER_VELOCITY).withName("run indexer velocity");
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("indexer/current", m_IO.getCurrent());
  }
}