package frc.robot.subsystems.intake;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class PivotSubsystem extends SubsystemBase {
  private final PivotIOHardware m_IO = new PivotIOHardware(IntakeConstants.PIVOT_MOTOR_ID);

  public PivotSubsystem() {
    super("pivot", "pivot");
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