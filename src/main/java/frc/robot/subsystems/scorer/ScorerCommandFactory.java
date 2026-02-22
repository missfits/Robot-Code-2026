package frc.robot.subsystems.scorer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LaserCANSensorBase;

public class ScorerCommandFactory {
  private ShooterSubsystem m_subsystem;
  private LaserCANSensorBase m_feederSensor;

  public ScorerCommandFactory(ShooterSubsystem shooter, LaserCANSensorBase feederSensor) {
    m_subsystem = shooter;
    m_feederSensor = feederSensor;
  }

  public Command runShooter() {
    return m_subsystem.voltageCommand(
      ShooterConstants.OUTTAKE_MOTOR_VOLTAGE
    ).withName("run shooter");
  }

  public Command runShooterBack() {
    return m_subsystem.voltageCommand(
      ShooterConstants.BACK_MOTOR_VOLTAGE
    ).withName("run shooter back");
  }

  public Command runShooterWithTimeout() {
    return m_subsystem.voltageCommand(
      ShooterConstants.OUTTAKE_MOTOR_VOLTAGE
    ).withTimeout(ShooterConstants.RUN_SHOOTER_TIME).withName("run shooter timeout");
  }

  public Command runShooterSmartDashboard() {
    return m_subsystem.VoltageVelocityPIDCommand(
      () -> SmartDashboard.getNumber("shooter influencer IO/velocity", ShooterConstants.OUTTAKE_MOTOR_VELOCITY)
    ).withName("run shooter smart dashboard");
  }

  public Command shooterOff() {
    return m_subsystem.offCommand().withName("shooter off");
  }

  public void setDefaultCommand() {
    m_subsystem.setDefaultCommand(m_subsystem.offCommand());
  }
}
