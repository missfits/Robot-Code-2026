package frc.robot.subsystems.intake;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIOHardware m_IO = new IntakeIOHardware(IntakeConstants.MECHANISM_MOTOR_ID);

  public IntakeSubsystem() {
    m_IO.resetPosition();
  }

  public Command runIntake(double velocity) {
    return this.run(() -> {
        m_IO.setVoltage(velocity);
        SmartDashboard.putNumber("intake/input velocity", velocity);
    });
  }

  public Command runIntakePID(double velocity) {
    return this.run(() -> {
        m_IO.setVelocityVoltage(velocity);
        SmartDashboard.putNumber("intake/input velocity", velocity);
    });
  }

  public Command runIntakeOff() {
    return new RunCommand(() -> {
        m_IO.setVoltage(0);
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