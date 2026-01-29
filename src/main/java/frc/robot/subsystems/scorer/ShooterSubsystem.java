package frc.robot.subsystems.scorer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;

public class ShooterSubsystem extends MechanismsSubsystemBase {
  private final ShooterInfluencerIOHardware m_influencerIO = new ShooterInfluencerIOHardware(ScorerConstants.INFLUENCER_MOTOR_ID);
  private final ShooterFollowerIOHardware m_followerIO = new ShooterFollowerIOHardware(ScorerConstants.FOLLOWER_MOTOR_ID);

  public ShooterSubsystem() {
    super("shooter", "shooter");
    m_influencerIO.resetPosition();
    m_followerIO.resetPosition();
  }

  protected void setVoltage(double volts) {
    m_influencerIO.setVoltage(volts);
    m_followerIO.setVoltage(volts);
  }

  protected void applyVelocityVoltage(double velocity) {
    m_influencerIO.setVelocityVoltage(velocity);
    m_followerIO.setVelocityVoltage(velocity);
  }

  public Command runMechanism(double influencerVelocity, double followerVelocity) {
    return loggedCommand("runShooter",
        this.run(() -> {
            setVoltage(influencerVelocity);
            setVoltage(followerVelocity);
        }));
  }
  
  public Command runShooterPID(double influencerVelocity, double followerVelocity) {
    return this.run(() -> {
        m_influencerIO.setVelocityVoltage(influencerVelocity);
        m_followerIO.setVelocityVoltage(followerVelocity);

        SmartDashboard.putNumber("shooter/influencer_velocity", influencerVelocity);
        SmartDashboard.putNumber("shooter/follower_velocity", followerVelocity);
    });
  }

  public Command runShooterOff() {
    return new RunCommand(() -> {
        m_influencerIO.setVoltage(0);
        m_followerIO.setVoltage(0);
      },
      this
    );
  }

  public void resetControllers() {
    m_influencerIO.resetSlot0Gains();
    m_followerIO.resetSlot0Gains();
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("shooter follower/current", m_followerIO.getCurrent());
    SmartDashboard.putNumber("shooter influencer/current", m_influencerIO.getCurrent());
  }
}