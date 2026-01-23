package frc.robot.subsystems.intake;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;


public class PivotSubsystem extends SubsystemBase {
  private final PivotIOHardware m_IO = new PivotIOHardware(PivotConstants.MECHANISM_MOTOR_ID);

  public PivotSubsystem() {
    m_IO.resetPosition();
  }
}