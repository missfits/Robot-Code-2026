package frc.robot.subsystems.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScorerConstants;

public class ScorerCommandFactory {
  private ShooterSubsystem m_subsystem;

  public ScorerCommandFactory(ShooterSubsystem shooter) {
    m_subsystem = shooter;
  }

  public Command runShooter() {
    return m_subsystem.runShooter(
      ScorerConstants.INFLUENCER_OUTTAKE_MOTOR_VELOCITY,
      ScorerConstants.FOLLOWER_OUTTAKE_MOTOR_VELOCITY
    ).withName("run shooter");
  }

  public Command runShooterBack() {
    return m_subsystem.runShooter(
      ScorerConstants.INFLUENCER_SHOOTER_BACK_VELOCITY,
      ScorerConstants.FOLLOWER_SHOOTER_BACK_VELOCITY
    ).withName("run shooter back");
  }

  public Command runShooterWithTimeout() {
    return m_subsystem.runShooter(
      ScorerConstants.INFLUENCER_OUTTAKE_MOTOR_VELOCITY,
      ScorerConstants.FOLLOWER_OUTTAKE_MOTOR_VELOCITY
    ).withTimeout(ScorerConstants.RUN_SHOOTER_TIME).withName("run shooter timeout");
  }

  public Command shooterOff() {
    return m_subsystem.runShooterOff().withName("shooter off");
  }
}
