package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class RollerSubsystem extends SubsystemBase {
  private final RollerIOHardware m_IO = new RollerIOHardware(IntakeConstants.ROLLER_MOTOR_ID);

  public RollerSubsystem() {
    m_IO.resetPosition();
  }

  public Command runRoller(double velocity) {
    return this.run(() -> {
        m_IO.setVoltage(velocity);
        SmartDashboard.putNumber("roller/input velocity", velocity);
    });
  }

  public Command runRollerPID(double velocity) {
    return this.run(() -> {
        m_IO.setVelocityVoltage(velocity);
        SmartDashboard.putNumber("roller/input velocity", velocity);
    });
  }

  public Command runRollerOff() {
    return new RunCommand(() -> {
        m_IO.setVoltage(0);
      },
      this
    );
  }

  public void resetControllers() {
    m_IO.resetSlot0Gains();
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("roller/subsystem", this);
    SmartDashboard.putNumber("roller/current", m_IO.getCurrent());
  }

}