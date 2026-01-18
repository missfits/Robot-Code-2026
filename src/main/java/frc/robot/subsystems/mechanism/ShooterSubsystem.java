package frc.robot.subsystems.mechanism;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIOHardware m_IO = new ShooterIOHardware();

  public ShooterSubsystem() {
        m_IO.resetPosition();
  }

  public Command runShooter(double speed) {
        return new FunctionalCommand(
            // set voltage in init also
            () -> {m_IO.setVoltage(speed); SmartDashboard.putString("shooter/currentlyRunningCommand", "runCollar");},
            () -> {m_IO.setVoltage(speed); SmartDashboard.putString("shooter/currentlyRunningCommand", "runCollar");},
            (interrupted) -> {},
            () -> false,
            this
        ).withName("runShooter");
    }

  public Command runIntakeOff() {
    return new RunCommand(
      () -> {
        m_IO.setVoltage(0);
        SmartDashboard.putBoolean("shooter/off", true);
      },
      this
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("shooter/subsystem", this);
    SmartDashboard.putNumber("shooter/current", m_IO.getCurrent());
  }

}