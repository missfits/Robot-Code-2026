package frc.robot.subsystems.mechanism;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIOHardware m_IO = new ClimberIOHardware();

  public ClimberSubsystem() {
    resetPosition();
  }

  public void resetPosition() {
    m_IO.resetPosition();
  }

  public Command runMechanismOff() {
    return new RunCommand(
      () -> {
        m_IO.setVoltage(0);
        SmartDashboard.putBoolean("climber/off", true);
      },
      this
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("climber/subsystem", this);
    SmartDashboard.putNumber("climber/position", m_IO.getPosition());
    SmartDashboard.putNumber("climber/current", m_IO.getCurrent());
  }

}