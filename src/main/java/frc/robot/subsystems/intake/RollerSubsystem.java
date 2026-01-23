package frc.robot.subsystems.intake;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
  private final RollerIOHardware m_IO = new RollerIOHardware();

  public RollerSubsystem() {
    m_IO.resetPosition();
  }

  public Command runRoller(double speed) {
        return new FunctionalCommand(
            // set voltage in init also
            () -> {m_IO.setVoltage(speed); SmartDashboard.putString("roller/currentlyRunningCommand", "runRoller");},
            () -> {m_IO.setVoltage(speed); SmartDashboard.putString("roller/currentlyRunningCommand", "runRoller");},
            (interrupted) -> {},
            () -> false,
            this
        ).withName("runRoller");
    }

  public Command runRollerOff() {
    return new RunCommand(
      () -> {
        m_IO.setVoltage(0);
        SmartDashboard.putBoolean("roller/off", true);
      },
      this
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("roller/subsystem", this);
    SmartDashboard.putNumber("roller/current", m_IO.getCurrent());
  }

}