package frc.robot.subsystems.scorer;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ScorerConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterInfluencerIOHardware m_influencerIO = new ShooterInfluencerIOHardware(ScorerConstants.INFLUENCER_MOTOR_ID);
  private final ShooterFollowerIOHardware m_followerIO = new ShooterFollowerIOHardware(ScorerConstants.FOLLOWER_MOTOR_ID);

  public ShooterSubsystem() {
    m_influencerIO.resetPosition();
    m_followerIO.resetPosition();
  }

  public Command runShooter(double influencerVelocity, double followerVelocity) {
    return this.run(() -> {
        m_influencerIO.setVoltage(influencerVelocity);
        m_followerIO.setVoltage(followerVelocity);
        SmartDashboard.putNumber("shooter/influencer input velocity", influencerVelocity);
        SmartDashboard.putNumber("shooter/follower input velocity", followerVelocity);
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
    SmartDashboard.putData("shooter/subsystem", this);
    SmartDashboard.putNumber("shooter/influencer current", m_influencerIO.getCurrent());
    SmartDashboard.putNumber("shooter/follower current", m_followerIO.getCurrent());

    Command current = this.getCurrentCommand();
    SmartDashboard.putString(
      "shooter/currentlyRunningOuterCommand",
      current != null ? current.getName() : "None"
    );
  }
}