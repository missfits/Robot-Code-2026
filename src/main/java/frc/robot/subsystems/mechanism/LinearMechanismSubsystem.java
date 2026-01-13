package frc.robot.subsystems.mechanism;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearMechanismSubsystem extends SubsystemBase {
  private final LinearMechanismIOHardware m_IO = new LinearMechanismIOHardware();

  public LinearMechanismSubsystem() {
    resetPosition();
  }

  public void resetPosition() {
    m_IO.resetPosition();
  }

  public Command runMechanismOff() {
    return new RunCommand(
      () -> {
        m_IO.setVoltage(0);
        SmartDashboard.putBoolean("linear mechanism/off", true);
      },
      this
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("linear mechanism/subsystem", this);
    SmartDashboard.putNumber("linear mechanism/position", m_IO.getPosition());
    SmartDashboard.putNumber("linear mechanism/current", m_IO.getCurrent());
  }

}