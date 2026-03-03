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
    return voltageCommand(RollerConstants.ROLLER_VOLTAGE).withName("run roller voltage");
  }

  public Command rollerBackVoltageCommand() {
    return voltageCommand(RollerConstants.ROLLER_BACK_VOLTAGE).withName("run roller back voltage");
  }

  public Command rollerWithTimeoutVoltageCommand() {
    return voltageCommand(RollerConstants.ROLLER_VOLTAGE)
    .withTimeout(RollerConstants.RUN_INTAKE_TIME)
    .withName("run intake voltage timeout");
  }

  public Command rollerVelocityCommand() { 
    return velocityCommand(RollerConstants.ROLLER_VELOCITY);
  }

  public Command rollerBackVelocityCommand() { 
    return velocityCommand(-RollerConstants.ROLLER_VELOCITY);
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("roller IO/live current", m_IO.getCurrent());
    SmartDashboard.putNumber("roller IO/live position", m_IO.getPositionDegrees());
    SmartDashboard.putNumber("roller IO/live velocity", m_IO.getVelocityDegreesPerSecond());
    SmartDashboard.putNumber("roller IO/live voltage", m_IO.getVoltage());
  }
}