package frc.robot.subsystems.scorer;

import edu.wpi.first.wpilibj2.command.Command;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;


public class IndexerSubsystem extends MechanismsSubsystemBase {
  private final IndexerIOHardware m_IO = new IndexerIOHardware(ScorerConstants.INDEXER_MOTOR_ID);

  public IndexerSubsystem() {
    super("indexer", "indexer");
    m_IO.resetPosition();
  }

  protected void setVoltage(double volts) {
    m_IO.setVoltage(volts);
  }

  protected void applyVelocityVoltage(double velocity) {
    m_IO.setVelocityVoltage(velocity);
  }
  
  public Command runIndexerPID(double velocity) {
    return this.run(() -> {
        m_IO.setVelocityVoltage(velocity);
        SmartDashboard.putNumber("indexer/input velocity", velocity);
    });
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("indexer/current", m_IO.getCurrent());
  }
}