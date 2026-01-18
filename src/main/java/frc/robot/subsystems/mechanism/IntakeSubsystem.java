package frc.robot.subsystems.mechanism;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIOHardware m_IO = new IntakeIOHardware();

  public IntakeSubsystem() {
    m_IO.resetPosition();
  }

  public Command runIntake(double speed) {
        return new FunctionalCommand(
            // set voltage in init also
            () -> {m_IO.setVoltage(speed); SmartDashboard.putString("intake/currentlyRunningCommand", "runIntake");},
            () -> {m_IO.setVoltage(speed); SmartDashboard.putString("intake/currentlyRunningCommand", "runIntake");},
            (interrupted) -> {},
            () -> false,
            this
        ).withName("runIntake");
    }

  public Command runIntakeOff() {
    return new RunCommand(
      () -> {
        m_IO.setVoltage(0);
        SmartDashboard.putBoolean("intake/off", true);
      },
      this
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("intake/subsystem", this);
    SmartDashboard.putNumber("intake/current", m_IO.getCurrent());
  }

}