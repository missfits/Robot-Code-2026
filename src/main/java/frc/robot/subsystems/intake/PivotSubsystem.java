package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  protected void applyVelocityVoltage(double velocity) {
    VelocityVoltage request = new VelocityVoltage(velocity)
    .withEnableFOC(IntakeConstants.PIVOT_ENABLE_FOC)
    .withFeedForward(IntakeConstants.PIVOT_FEED_FORWARD)
    .withSlot(IntakeConstants.PIVOT_SLOT)
    .withOverrideBrakeDurNeutral(IntakeConstants.PIVOT_OVERRIDE_BRAKE_DUR_NEUTRAL);
    m_IO.setVelocityVoltage(request);
  }
  
  public Command runPivotPID(double velocity) {
    return this.run(() -> {
        m_IO.setVelocityVoltage(velocity);
        SmartDashboard.putNumber("pivot/input velocity", velocity);
    });
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("pivot/current", m_IO.getCurrent());
  }
}