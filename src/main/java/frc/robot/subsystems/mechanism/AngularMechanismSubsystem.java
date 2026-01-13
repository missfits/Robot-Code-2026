package frc.robot.subsystems.mechanism;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AngularMechanismSubsystem extends SubsystemBase {
  private final AngularMechanismIOHardware m_IO = new AngularMechanismIOHardware();

  public AngularMechanismSubsystem() {
    resetPosition();
  }

  public void resetPosition() {
    m_IO.resetPosition();
  }

  public Command runMechanismOff() {
    return new RunCommand(
      () -> {
        m_IO.setVoltage(0);
        SmartDashboard.putBoolean("angular mechanism/off", true);
      },
      this
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("angular mechanism/subsystem", this);
    SmartDashboard.putNumber("angular mechanism/position", m_IO.getPosition());
    SmartDashboard.putNumber("angular mechanism/current", m_IO.getCurrent());
  }

}