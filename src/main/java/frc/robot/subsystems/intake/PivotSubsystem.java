package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

  public Command runPivotManual(double volts) {
    return run(() -> {
        double clampedVolts = MathUtil.clamp(volts, -PivotConstants.PIVOT_MAX_MANUAL_VOLTS, PivotConstants.PIVOT_MAX_MANUAL_VOLTS);
        double position = m_IO.getPositionDegrees();

        if ((position <= PivotConstants.STORE_POSITION_DEGREES && clampedVolts < 0) ||
            (position >= PivotConstants.DEPLOY_POSITION_DEGREES && clampedVolts > 0))
            m_IO.setVoltage(0.0);
        else
            m_IO.setVoltage(clampedVolts);
    });
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
      MotionMagicVoltage request = new MotionMagicVoltage(m_IO.degreesToMotorRevolutions(PivotConstants.DEPLOY_POSITION_DEGREES));
      m_IO.goToPositionProfiled(request);
    });
  }

  public Command storeIntakeFactory() {
    return this.run(() ->  {
      MotionMagicVoltage request = new MotionMagicVoltage(m_IO.degreesToMotorRevolutions(PivotConstants.STORE_POSITION_DEGREES));
      m_IO.goToPositionProfiled(request);
    });
  }

  public void resetControllers() {
    m_IO.resetSlot0Gains();
  }
}