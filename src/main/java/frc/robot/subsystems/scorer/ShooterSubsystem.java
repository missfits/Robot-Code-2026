package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;
import frc.robot.utils.ShooterLookupTable;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShooterSubsystem extends MechanismsSubsystemBase {
  private final ShooterIOHardware m_influencerIO = new ShooterIOHardware(ShooterMotorType.INFLUENCER);
  private final ShooterIOHardware m_followerIO = new ShooterIOHardware(ShooterMotorType.FOLLOWER);
  private final ShooterIOHardware m_thirdIO = new ShooterIOHardware(ShooterMotorType.THIRD);

  public ShooterSubsystem() {
    super("shooter");
    resetPosition();
    m_followerIO.followMotor(m_influencerIO, false);
    m_thirdIO.followMotor(m_influencerIO, false);
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
    m_thirdIO.resetSlot0Gains();
  }

  public void resetPosition() {
    m_influencerIO.resetPosition();
    m_followerIO.resetPosition();
    m_thirdIO.resetPosition();
  }

  // Commands
  public Command shooterVelocityCommand(double velocity) {
    return velocityCommand(velocity).withName("run shooter velocity " + velocity);
  }

  public Command shooterVelocityCommand(Supplier<Double> velocitySupplier) {
    return run(() -> { m_influencerIO.setVelocityVoltage(velocitySupplier.get());});
    //return velocityCommand(velocitySupplier.get()).withName("run shooter velocity from supplier");
  }

  public Command shooterVoltageCommand() {
    return voltageCommand(
      ShooterConstants.SHOOTER_VOLTAGE
    ).withName("run shooter voltage");
  }
  
  public Command shooterBackVoltageCommand() {
    return voltageCommand(
      ShooterConstants.SHOOTER_BACK_VOLTAGE
    ).withName("run shooter back voltage");
  }

  public Command shooterWithTimeoutVoltageCommand() {
    return voltageCommand(
      ShooterConstants.SHOOTER_VOLTAGE
    ).withTimeout(ShooterConstants.RUN_SHOOTER_TIME).withName("run shooter timeout");
  }

  public Command shooterAutoCommand() {
    return velocityCommand(ShooterConstants.SHOOTER_VELOCITY)
      .withTimeout(ShooterConstants.RUN_SHOOTER_TIME).withName("run shooter auto");
  }

  public Trigger isCurrentSpiking() {
    return new Trigger(() -> m_influencerIO.getCurrent() > ShooterConstants.CURRENT_SPIKE_THRESHOLD);
  }

  // Triggers
  public Trigger atTargetVelocityTrigger(Supplier<Double> targetVelocitySupplier) {
    return m_influencerIO.atTargetVelocityTrigger(ShooterConstants.VELOCITY_TOLERANCE, targetVelocitySupplier);
  }

  public Trigger atTargetVelocityTrigger(double targetVelocity) {
    return atTargetVelocityTrigger(() -> targetVelocity);
  }

  private boolean isVelocityWithinPercentTolerance(double currentVelocity, double targetVelocity) {
    double thresholdVelocity = targetVelocity * ShooterConstants.FUEL_SHOT_DETECTION_PERCENTAGE;
    return targetVelocity >= 0
      ? currentVelocity > thresholdVelocity
      : currentVelocity < thresholdVelocity;
  }

  public Trigger isMotorVelocityWithinPercentTolerance(Supplier<Double> targetVelocitySupplier) {
    return new Trigger(() -> isVelocityWithinPercentTolerance(
      m_influencerIO.getMotorVelocityRevolutionsPerSecond(),
      targetVelocitySupplier.get()
    ));
  }
  public Trigger isFuelShot(Supplier<Double> targetVelocitySupplier) {
    return isCurrentSpiking().and(isMotorVelocityWithinPercentTolerance(targetVelocitySupplier));
   }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("shooter follower IO/live current", m_followerIO.getCurrent());
    SmartDashboard.putNumber("shooter follower IO/live position", m_followerIO.getPositionDegrees());
    SmartDashboard.putNumber("shooter follower IO/live velocity", m_followerIO.getVelocityDegreesPerSecond());
    SmartDashboard.putNumber("shooter follower IO/live voltage", m_followerIO.getVoltage());

    SmartDashboard.putNumber("shooter influencer IO/live current", m_influencerIO.getCurrent());
    SmartDashboard.putNumber("shooter influencer IO/live position", m_influencerIO.getPositionDegrees());
    SmartDashboard.putNumber("shooter influencer IO/live velocity", m_influencerIO.getVelocityRotationsPerSecond());
    SmartDashboard.putNumber("shooter influencer IO/live voltage", m_influencerIO.getVoltage());

    SmartDashboard.putNumber("shooter third IO/live current", m_thirdIO.getCurrent());
    SmartDashboard.putNumber("shooter third IO/live position", m_thirdIO.getPositionDegrees());
    SmartDashboard.putNumber("shooter third IO/live velocity", m_thirdIO.getVelocityDegreesPerSecond());
    SmartDashboard.putNumber("shooter third IO/live voltage", m_thirdIO.getVoltage());

    SmartDashboard.putBoolean("shooter/is current spiking", isCurrentSpiking().getAsBoolean());
  }
}