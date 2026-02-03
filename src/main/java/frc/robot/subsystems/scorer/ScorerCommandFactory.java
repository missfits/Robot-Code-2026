package frc.robot.subsystems.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.LaserCANSensorBase;

public class ScorerCommandFactory {
  private ShooterSubsystem m_subsystem;
  private LaserCANSensorBase m_feederSensor;

  public ScorerCommandFactory(ShooterSubsystem shooter, LaserCANSensorBase feederSensor) {
    m_subsystem = shooter;
    m_feederSensor = feederSensor;
  }

  public Command runShooter() {
    return m_subsystem.runMechanismPID(
      ScorerConstants.INFLUENCER_OUTTAKE_MOTOR_VELOCITY,
      ScorerConstants.FOLLOWER_OUTTAKE_MOTOR_VELOCITY
    ).withName("run shooter");
  }

  public Command runShooterBack() {
    return m_subsystem.runMechanismPID(
      ScorerConstants.INFLUENCER_SHOOTER_BACK_VELOCITY,
      ScorerConstants.FOLLOWER_SHOOTER_BACK_VELOCITY
    ).withName("run shooter back");
  }

  public Command runShooterWithTimeout() {
    return m_subsystem.runMechanismPID(
      ScorerConstants.INFLUENCER_OUTTAKE_MOTOR_VELOCITY,
      ScorerConstants.FOLLOWER_OUTTAKE_MOTOR_VELOCITY
    ).withTimeout(ScorerConstants.RUN_SHOOTER_TIME).withName("run shooter timeout");
  }

  public Command runShooterPID() { 
    return m_subsystem.runMechanismPID(
      ScorerConstants.INFLUENCER_OUTTAKE_MOTOR_VELOCITY,
      ScorerConstants.FOLLOWER_OUTTAKE_MOTOR_VELOCITY
    ).withName("run shooter PID");
  }

  public Command shooterOff() {
    return m_subsystem.runMechanismOff().withName("shooter off");
  }
}
