package frc.robot.subsystems.scorer;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;


public class ShooterSubsystem extends MechanismsSubsystemBase {
  private final ShooterIOHardware m_IO = new ShooterIOHardware(ShooterConstants.MECHANISM_MOTOR_ID);

  public ShooterSubsystem() {
    super("shooter", "shooter");
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
    SmartDashboard.putNumber("shooter/current", m_IO.getCurrent());
  }
}