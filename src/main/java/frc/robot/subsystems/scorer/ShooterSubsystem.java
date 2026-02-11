package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;

public class ShooterSubsystem extends MechanismsSubsystemBase {
  private final ShooterInfluencerIOHardware m_influencerIO = new ShooterInfluencerIOHardware(ShooterConstants.INFLUENCER_MOTOR_ID);
  private final ShooterFollowerIOHardware m_followerIO = new ShooterFollowerIOHardware(ShooterConstants.FOLLOWER_MOTOR_ID);

  public ShooterSubsystem() {
    super("shooter");
    resetPosition();
    m_followerIO.followMotor(m_influencerIO, false);
  }

  protected void setVoltage(double volts) {
    m_influencerIO.setVoltage(volts);
  }

  @Override
  protected void runClosedLoopVelocity(double velocity) {

    VelocityVoltage influencerRequest = new VelocityVoltage(velocity)
    .withEnableFOC(ShooterConstants.INFLUENCER_ENABLE_FOC)
    .withFeedForward(ShooterConstants.INFLUENCER_FEED_FORWARD)
    .withSlot(ShooterConstants.INFLUENCER_SLOT)
    .withOverrideBrakeDurNeutral(ShooterConstants.INFLUENCER_OVERRIDE_BRAKE_DUR_NEUTRAL);

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
    m_followerIO.resetSlot0Gains();
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