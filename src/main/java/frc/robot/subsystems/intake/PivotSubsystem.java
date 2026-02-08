package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;


public class PivotSubsystem extends MechanismsSubsystemBase {
  private final PivotIOHardware m_IO = new PivotIOHardware(IntakeConstants.PIVOT_MOTOR_ID);

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
    .withEnableFOC(IntakeConstants.PIVOT_ENABLE_FOC)
    .withFeedForward(IntakeConstants.PIVOT_FEED_FORWARD)
    .withSlot(IntakeConstants.PIVOT_SLOT)
    .withOverrideBrakeDurNeutral(IntakeConstants.PIVOT_OVERRIDE_BRAKE_DUR_NEUTRAL);
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
      MotionMagicVoltage request = new MotionMagicVoltage(m_IO.degreesToMotorRevolutions(IntakeConstants.PIVOT_DEPLOY_POSITION_DEGREES));
      m_IO.goToPositionProfiled(request);
    });
  }

  public Command storeIntakeFactory() {
    return this.run(() ->  {
      MotionMagicVoltage request = new MotionMagicVoltage(m_IO.degreesToMotorRevolutions(IntakeConstants.PIVOT_STORE_POSITION_DEGREES));
      m_IO.goToPositionProfiled(request);
    });
  }
}