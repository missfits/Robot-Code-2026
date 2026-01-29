package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;


public class ClimberSubsystem extends MechanismsSubsystemBase {
  private final ClimberIOHardware m_IO = new ClimberIOHardware(ClimberConstants.CLIMBER_MOTOR_ID);

  public ClimberSubsystem() {
    super("climber", "climber");
    m_IO.resetPosition();
  }

  protected void setVoltage(double volts) {
    m_IO.setVoltage(volts);
  }

  protected void applyVelocityVoltage(double velocity) {
    m_IO.setVelocityVoltage(velocity);
  }

  protected void resetPosition() {
    m_IO.resetPosition();
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("climber/current", m_IO.getCurrent());
  }
}