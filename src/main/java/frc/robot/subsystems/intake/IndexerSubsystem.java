package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;


public class IndexerSubsystem extends MechanismsSubsystemBase {
  private final IndexerIOHardware m_IO = new IndexerIOHardware(ScorerConstants.INDEXER_MOTOR_ID);

  public IndexerSubsystem() {
    super("indexer");
    m_IO.resetPosition();
  }

  protected void setVoltage(double volts) {
    m_IO.setVoltage(volts);
  }

  @Override
  protected void runClosedLoopVelocity(double velocity) {
    VelocityVoltage request = new VelocityVoltage(velocity)
    .withEnableFOC(ScorerConstants.INDEXER_ENABLE_FOC)
    .withFeedForward(ScorerConstants.INDEXER_FEED_FORWARD)
    .withSlot(ScorerConstants.INDEXER_SLOT)
    .withOverrideBrakeDurNeutral(ScorerConstants.INDEXER_OVERRIDE_BRAKE_DUR_NEUTRAL);
    m_IO.setVelocityVoltage(request);
  }

  public void resetPosition() {
    m_IO.resetPosition();
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("indexer/current", m_IO.getCurrent());
  }

  public Command runIndexerFactory() {
    return this.run(() -> runClosedLoopVelocity(ScorerConstants.INDEXER_MOTOR_VELOCITY));
  }
}