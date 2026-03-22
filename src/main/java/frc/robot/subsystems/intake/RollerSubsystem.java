package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;

public class RollerSubsystem extends MechanismsSubsystemBase {
  private final RollerIOHardware m_IO = new RollerIOHardware(RollerConstants.MOTOR_ID);

  public RollerSubsystem() {
    super("roller");
    m_IO.resetPosition();
    m_IO.setInverted(true);
  }

  protected void setVoltage(double volts) {
    m_IO.setVoltage(volts);
  }

  @Override
  protected void runClosedLoopVelocity(double velocity) {
    VelocityVoltage request = new VelocityVoltage(velocity)
    .withEnableFOC(RollerConstants.ENABLE_FOC)
    .withFeedForward(RollerConstants.FEED_FORWARD)
    .withSlot(RollerConstants.SLOT)
    .withOverrideBrakeDurNeutral(RollerConstants.OVERRIDE_BRAKE_DUR_NEUTRAL);
    m_IO.setVelocityVoltage(request);
  }

  public void resetControllers() {
    m_IO.resetSlot0Gains();
  }

  public void resetPosition() {
    m_IO.resetPosition();
  }

  // Commands
  public Command rollerVoltageCommand() {
    return voltageCommand(RollerConstants.ROLLER_VOLTAGE).withName("rollerVoltageCommand");
  }

  public Command rollerBackVoltageCommand() {
    return voltageCommand(RollerConstants.ROLLER_BACK_VOLTAGE).withName("rollerBackVoltageCommand");
  }

  public Command rollerWithTimeoutVoltageCommand() {
    return voltageCommand(RollerConstants.ROLLER_VOLTAGE)
    .withTimeout(RollerConstants.RUN_INTAKE_TIME)
    .withName("rollerWithTimeoutVoltageCommand");
  }

  public Command rollerVelocityCommand() { 
    return velocityCommand(RollerConstants.ROLLER_VELOCITY).withName("rollerVelocityCommand");
  }

  public Command rollerBackVelocityCommand() { 
    return velocityCommand(-RollerConstants.ROLLER_VELOCITY).withName("rollerBackVelocityCommand");
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("roller/actualCurrent", m_IO.getCurrent());
    SmartDashboard.putNumber("roller/actualPositionDegrees", m_IO.getPositionDegrees());
    SmartDashboard.putNumber("roller/actualVelocityRotationsPerSecond", m_IO.getVelocityRotationsPerSecond());
    SmartDashboard.putNumber("roller/actualVelocityDegreesPerSecond", m_IO.getVelocityDegreesPerSecond());
    SmartDashboard.putNumber("roller/actualVoltage", m_IO.getVoltage());
  }
}