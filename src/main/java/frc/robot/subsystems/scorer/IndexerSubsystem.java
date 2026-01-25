package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;


public class IndexerSubsystem extends MechanismsSubsystemBase {
  private final IndexerIOHardware m_IO = new IndexerIOHardware(IndexerConstants.MECHANISM_MOTOR_ID);

  public IndexerSubsystem() {
    super("indexer", "indexer");
    m_IO.resetPosition();
  }

  protected void setVoltage(double volts) {
    m_IO.setVoltage(volts);
  }

  protected void applyVelocityVoltage(VelocityVoltage request) {
    m_IO.setVelocityVoltage(request);
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