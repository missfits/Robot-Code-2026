package frc.robot.subsystems.intake;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;


public class RollerSubsystem extends MechanismsSubsystemBase {
  private final RollerIOHardware m_IO = new RollerIOHardware(RollerConstants.MECHANISM_MOTOR_ID);

  public RollerSubsystem() {
    super("roller", "roller");
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
    SmartDashboard.putNumber("roller/current", m_IO.getCurrent());
  }
}