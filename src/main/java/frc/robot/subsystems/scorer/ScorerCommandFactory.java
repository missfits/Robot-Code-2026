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

  public Command shooterVoltageCommand() {
    return m_subsystem.voltageCommand(
      ShooterConstants.OUTTAKE_MOTOR_VOLTAGE
    ).withName("run shooter");
  }
  
  public Command shooterVelocityCommand(double velocity) {
    return m_subsystem.voltageVelocityPIDCommand(velocity).withName("run shooter velocity " + velocity);
  }
  

  public Command shooterBackVoltageCommand() {
    return m_subsystem.voltageCommand(
      ShooterConstants.BACK_MOTOR_VOLTAGE
    ).withName("run shooter back");
  }

  public Command shooterWithTimeoutVoltageCommand() {
    return m_subsystem.voltageCommand(
      ShooterConstants.OUTTAKE_MOTOR_VOLTAGE
    ).withTimeout(ShooterConstants.RUN_SHOOTER_TIME).withName("run shooter timeout");
  }

  
    public Command shooterSmartDashboardVelocityCommand(String name, double defaultVelocity) {
    return m_subsystem.velocityCommand(
      () -> SmartDashboard.getNumber("shooter test speeds/" + name, defaultVelocity)
    ).withName("run shooter smart dashboard " + name);
  }

  public Command shooterOffCommand() {
    return m_subsystem.offCommand().withName("shooter off");
  }

  public void setDefaultCommand() {
    m_subsystem.setDefaultCommand(m_subsystem.offCommand());
  }
}
