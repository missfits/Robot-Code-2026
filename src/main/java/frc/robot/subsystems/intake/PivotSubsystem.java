package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;


public class PivotSubsystem extends MechanismsSubsystemBase {
  private final PivotIOHardware m_IO = new PivotIOHardware(PivotConstants.MOTOR_ID);

  public PivotSubsystem() {
    super("pivot");
    m_IO.resetPosition();
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
    m_IO.setPositionDegrees(PivotConstants.DEPLOY_POSITION_DEGREES);
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("pivot IO/live current", m_IO.getCurrent());
    SmartDashboard.putNumber("pivot IO/live position", m_IO.getPositionDegrees());
    SmartDashboard.putNumber("pivot IO/live velocity", m_IO.getVelocityDegreesPerSecond());
    SmartDashboard.putNumber("pivot IO/live voltage", m_IO.getVoltage());
  }

  public void resetControllers() {
    m_IO.resetSlot0Gains();
  }

  // Commands 
  public Command voltageDeployPivotCommand() {
    return voltageCommand(PivotConstants.DEPLOY_VOLTAGE).withName("deploy pivot voltage");
  }

  public Command voltageStorePivotCommand() {
    return voltageCommand(PivotConstants.STORE_VOLTAGE).withName("store pivot voltage");
  }

  public Command velocityDeployPivotCommand() {
    return velocityCommand(PivotConstants.DEPLOY_VELOCITY).withName("deploy pivot velocity");
  }

  public Command velocityStorePivotCommand() {
    return velocityCommand(PivotConstants.STORE_VELOCITY).withName("store pivot velocity");
  }

  public Command deployPivotCommand() {
    return motionMagicVoltageCommand (() -> PivotConstants.DEPLOY_POSITION_DEGREES);
  }

  public Command storePivotCommand() {
    return motionMagicVoltageCommand (() -> PivotConstants.STORE_POSITION_DEGREES);
  }

  public Command displaceFuelCommand() {
    return Commands.sequence(
      motionMagicVoltageCommand(() -> PivotConstants.DISPLACE_FUEL_POSITION_DEGREES).withTimeout(PivotConstants.DISPLACE_FUEL_UP_TIMEOUT), 
      motionMagicVoltageCommand(() -> PivotConstants.DEPLOY_POSITION_DEGREES).withTimeout(PivotConstants.DISPLACE_FUEL_DOWN_TIMEOUT)
    );
  }

  
  private Command motionMagicVoltageCommand(DoubleSupplier positionSupplier) {
    return this.run(() ->  {
      MotionMagicVoltage request = new MotionMagicVoltage(m_IO.degreesToMotorRevolutions(positionSupplier.getAsDouble())).withUpdateFreqHz(30);
      m_IO.goToPositionProfiled(request);
    });
  }
}