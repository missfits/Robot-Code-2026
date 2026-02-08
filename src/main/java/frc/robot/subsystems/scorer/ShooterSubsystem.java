package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;

public class ShooterSubsystem extends MechanismsSubsystemBase {
  private final ShooterInfluencerIOHardware m_influencerIO = new ShooterInfluencerIOHardware(ScorerConstants.INFLUENCER_MOTOR_ID);
  private final ShooterFollowerIOHardware m_followerIO = new ShooterFollowerIOHardware(ScorerConstants.FOLLOWER_MOTOR_ID);

  public ShooterSubsystem() {
    super("shooter");
    resetPosition();
    m_followerIO.followMotor(m_influencerIO, false);
  }

  protected void setVoltage(double volts) {
    m_influencerIO.setVoltage(volts);
  }

  @Override
  protected void runClosedLoopVelocity(double influencerVelocity) {

    VelocityVoltage influencerRequest = new VelocityVoltage(influencerVelocity)
    .withEnableFOC(ScorerConstants.INFLUENCER_ENABLE_FOC)
    .withFeedForward(ScorerConstants.INFLUENCER_FEED_FORWARD)
    .withSlot(ScorerConstants.INFLUENCER_SLOT)
    .withOverrideBrakeDurNeutral(ScorerConstants.INFLUENCER_OVERRIDE_BRAKE_DUR_NEUTRAL);
    m_influencerIO.setVelocityVoltage(influencerRequest);
  }

  public Command runShooterOff() {
    return new RunCommand(() -> {
        m_influencerIO.setVoltage(0);
      },
      this
    );
  }

  public void resetControllers() {
    m_influencerIO.resetSlot0Gains();
  }

  public void resetPosition() {
    m_influencerIO.resetPosition();
    m_followerIO.resetPosition();
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("shooter follower/current", m_followerIO.getCurrent());
    SmartDashboard.putNumber("shooter influencer/current", m_influencerIO.getCurrent());
  }
}