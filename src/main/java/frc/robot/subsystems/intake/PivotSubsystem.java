package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;


public class PivotSubsystem extends MechanismsSubsystemBase {
  private final PivotIOHardware m_IO = new PivotIOHardware(IntakePivotConstants.MOTOR_ID);

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
    .withEnableFOC(IntakePivotConstants.ENABLE_FOC)
    .withFeedForward(IntakePivotConstants.FEED_FORWARD)
    .withSlot(IntakePivotConstants.SLOT)
    .withOverrideBrakeDurNeutral(IntakePivotConstants.OVERRIDE_BRAKE_DUR_NEUTRAL);
    m_IO.setVelocityVoltage(request);
  }

  public void resetPosition() {
    m_IO.resetPosition();
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("pivot/current", m_IO.getCurrent());
  }

  public Command deployIntakeFactory() {
    return this.run(() ->  {
      MotionMagicVoltage request = new MotionMagicVoltage(m_IO.degreesToMotorRevolutions(IntakePivotConstants.DEPLOY_POSITION_DEGREES));
      m_IO.goToPositionProfiled(request);
    });
  }

  public Command storeIntakeFactory() {
    return this.run(() ->  {
      MotionMagicVoltage request = new MotionMagicVoltage(m_IO.degreesToMotorRevolutions(IntakePivotConstants.STORE_POSITION_DEGREES));
      m_IO.goToPositionProfiled(request);
    });
  }
}