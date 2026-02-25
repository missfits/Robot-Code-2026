package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;
import frc.robot.utils.ShooterLookupTable;
import java.util.function.Supplier;

public class ShooterSubsystem extends MechanismsSubsystemBase {
  private final ShooterIOHardware m_influencerIO = new ShooterIOHardware(ShooterMotorType.INFLUENCER);
  private final ShooterIOHardware m_followerIO = new ShooterIOHardware(ShooterMotorType.FOLLOWER);

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

  // Commands
  public Command shooterVelocityCommand(double velocity) {
    return velocityCommand(velocity).withName("run shooter velocity " + velocity);
  }

  public Command shooterVelocityCommand(Supplier<Double> velocitySupplier) {
    return velocityCommand(velocitySupplier.get()).withName("run shooter velocity from supplier");
  }

  public Command shooterVoltageCommand() {
    return voltageCommand(
      ShooterConstants.OUTTAKE_MOTOR_VOLTAGE
    ).withName("run shooter voltage");
  }
  
  public Command shooterBackVoltageCommand() {
    return voltageCommand(
      ShooterConstants.BACK_MOTOR_VOLTAGE
    ).withName("run shooter back voltage");
  }

  public Command shooterWithTimeoutVoltageCommand() {
    return voltageCommand(
      ShooterConstants.OUTTAKE_MOTOR_VOLTAGE
    ).withTimeout(ShooterConstants.RUN_SHOOTER_TIME).withName("run shooter timeout");
  }

  // Triggers
  private boolean atTargetVelocity() {
    return m_influencerIO.atTargetVelocityTrigger(ShooterConstants.VELOCITY_TOLERANCE).getAsBoolean();
  }

  public Trigger atTargetVelocityTrigger() {
    return new Trigger(() -> atTargetVelocity());
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("shooter follower/current", m_followerIO.getCurrent());
    SmartDashboard.putNumber("shooter influencer/current", m_influencerIO.getCurrent());

    SmartDashboard.putNumber("shooter influencer/voltage", m_influencerIO.getVoltage());
    SmartDashboard.putNumber("shooter influencer/velocityDPS", m_influencerIO.getVelocityDegreesPerSecond());
    SmartDashboard.putNumber("shooter influencer/velocityRadiansPS", m_influencerIO.getVelocityRadiansPerSecond());
    SmartDashboard.putNumber("shooter influencer/velocityRevolutionsPS", m_influencerIO.getVelocityRevolutionsPerSecond());


  }
}