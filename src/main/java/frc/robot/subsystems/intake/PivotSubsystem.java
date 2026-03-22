package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;


public class PivotSubsystem extends MechanismsSubsystemBase {
  private final PivotIOHardware m_IO = new PivotIOHardware(PivotConstants.MOTOR_ID);

  public PivotSubsystem() {
    super("pivot");
    m_IO.resetPosition();
    m_IO.setInverted(true);
  }

  protected void setVoltage(double volts) {
    m_IO.setVoltage(volts);
  }

  @Override
  protected void runClosedLoopVelocity(double velocity) {
    VelocityVoltage request = new VelocityVoltage(velocity)
    .withEnableFOC(PivotConstants.ENABLE_FOC)
    .withFeedForward(PivotConstants.FEED_FORWARD)
    .withSlot(PivotConstants.SLOT)
    .withOverrideBrakeDurNeutral(PivotConstants.OVERRIDE_BRAKE_DUR_NEUTRAL);
    m_IO.setVelocityVoltage(request);
  }

  public void resetPosition() {
    m_IO.resetPosition();
  }

  public void resetToDeployPosition() {
    m_IO.setPositionDegrees(PivotConstants.RESET_DEPLOY_POSITION_DEGREES);
  }

  public void resetPosition(double positionDegrees) {
    m_IO.setPositionDegrees(positionDegrees);
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("pivot/actualCurrent", m_IO.getCurrent());
    SmartDashboard.putNumber("pivot/actualPositionDegrees", m_IO.getPositionDegrees());
    SmartDashboard.putNumber("pivot/actualVelocityRotationsPerSecond", m_IO.getVelocityRotationsPerSecond());
    SmartDashboard.putNumber("pivot/actualVelocityDegreesPerSecond", m_IO.getVelocityDegreesPerSecond());
    SmartDashboard.putNumber("pivot/actualVoltage", m_IO.getVoltage());
  }

  public void resetControllers() {
    m_IO.resetSlot0Gains();
  }

  // Commands 
  public Command voltageDeployPivotCommand() {
    return voltageCommand(PivotConstants.DEPLOY_VOLTAGE).withName("voltageDeployPivotCommand");
  }

  public Command voltageStorePivotCommand() {
    return voltageCommand(PivotConstants.STORE_VOLTAGE).withName("voltageStorePivotCommand");
  }

  public Command velocityDeployPivotCommand() {
    return velocityCommand(PivotConstants.DEPLOY_VELOCITY).withName("velocityDeployPivotCommand");
  }

  public Command velocityStorePivotCommand() {
    return velocityCommand(PivotConstants.STORE_VELOCITY).withName("velocityStorePivotCommand");
  }

  public Command deployPivotCommand() {
    return motionMagicVoltageCommand(() -> PivotConstants.DEPLOY_POSITION_DEGREES)
      .withName("deployPivotCommand");
  }

  public Command storePivotCommand() {
    return motionMagicVoltageCommand(() -> PivotConstants.STORE_POSITION_DEGREES)
      .withName("storePivotCommand");
  }

  public Command displaceFuelCommand() {
    return Commands.sequence(
      motionMagicVoltageCommand(() -> PivotConstants.DISPLACE_FUEL_POSITION_DEGREES).withTimeout(PivotConstants.DISPLACE_FUEL_UP_TIMEOUT), 
      motionMagicVoltageCommand(() -> PivotConstants.DEPLOY_POSITION_DEGREES).withTimeout(PivotConstants.DISPLACE_FUEL_DOWN_TIMEOUT)
    ).withName("displaceFuelCommand");
  }

  public Command repeatingDisplaceFuelCommand() {
    return Commands.repeatingSequence(
      displaceFuelCommand(),
      Commands.waitSeconds(PivotConstants.DISPLACE_FUEL_DELAY)
    ).withName("repeatingDisplaceFuelCommand");
  }

  
  private Command motionMagicVoltageCommand(DoubleSupplier positionSupplier) {
    return this.run(() ->  {
      SmartDashboard.putNumber("pivot/targetPositionDegrees", positionSupplier.getAsDouble());
      MotionMagicVoltage request = new MotionMagicVoltage(m_IO.degreesToMotorRevolutions(positionSupplier.getAsDouble())).withUpdateFreqHz(30);
      m_IO.goToPositionProfiled(request);
    });
  }

  public Command zeroPivotCommand() {
      return this.run(() -> setVoltage(PivotConstants.ZERO_PIVOT_VOLTAGE))
        .until(() -> m_IO.getCurrent() > PivotConstants.CURRENT_THRESHOLD)
        .andThen(new InstantCommand(() -> this.resetPosition(PivotConstants.RESET_DEPLOY_POSITION_DEGREES)))
        .withName("zeroPivotCommand");
  }


  public Command autoZeroPivotCommand() {
      return this.run(() -> setVoltage(PivotConstants.AUTO_ZERO_PIVOT_VOLTAGE))
        .until(() -> m_IO.getCurrent() > PivotConstants.AUTO_CURRENT_THRESHOLD)
        .andThen(new InstantCommand(() -> this.resetPosition(PivotConstants.AUTO_RESET_DEPLOY_POSITION_DEGREES)))
        .withName("autoZeroPivotCommand");
  }
}