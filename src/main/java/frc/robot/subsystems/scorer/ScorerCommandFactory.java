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

  public Command runShooter(double velocity) {
    return m_subsystem.runMechanism(velocity).withName("run shooter velocity " + velocity);
  }

  public Command runShooterBack() {
    return m_subsystem.runMechanism(
      ShooterConstants.BACK_MOTOR_VOLTAGE
    ).withName("run shooter back");
  }

  public Command runShooterWithTimeout() {
    return m_subsystem.runMechanism(
      ShooterConstants.OUTTAKE_MOTOR_VOLTAGE
    ).withTimeout(ShooterConstants.RUN_SHOOTER_TIME).withName("run shooter timeout");
  }

  public Command runShooterSmartDashboard(String name, double defaultVelocity) {
    return m_subsystem.runMechanismPID(
      () -> SmartDashboard.getNumber("shooter test speeds/" + name, defaultVelocity)
    ).withName("run shooter smart dashboard " + name);
  }

  public Command shooterOff() {
    return m_subsystem.runMechanismOff().withName("shooter off");
  }

  public void setDefaultCommand() {
    m_subsystem.setDefaultCommand(m_subsystem.runMechanismOff());
  }
}
