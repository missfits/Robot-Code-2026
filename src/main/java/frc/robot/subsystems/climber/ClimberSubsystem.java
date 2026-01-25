package frc.robot.subsystems.climber;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;


public class ClimberSubsystem extends MechanismsSubsystemBase {
  private final ClimberIOHardware m_IO = new ClimberIOHardware(ClimberConstants.MECHANISM_MOTOR_ID);

  public ClimberSubsystem() {
    super("climber", "climber");
    m_IO.resetPosition();
  }

  protected void setVoltage(double volts) {
    m_IO.setVoltage(volts);
  }

  protected void applyVelocityVoltage(VelocityVoltage request) {
    m_IO.setVelocityVoltage(request);
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("pivot/current", m_IO.getCurrent());
  }
}