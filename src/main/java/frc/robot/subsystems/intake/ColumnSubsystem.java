package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ColumnWheelsConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;

public class ColumnSubsystem extends MechanismsSubsystemBase {
  private final ColumnIOHardware m_IO = new ColumnIOHardware(ColumnWheelsConstants.MOTOR_ID);

  public ColumnSubsystem() {
    super("column");
    m_IO.resetPosition();
  }

  protected void setVoltage(double volts) {
    m_IO.setVoltage(volts);
  }

  @Override
  protected void runClosedLoopVelocity(double velocity) {
    VelocityVoltage request = new VelocityVoltage(velocity)
    .withEnableFOC(ColumnWheelsConstants.ENABLE_FOC)
    .withFeedForward(ColumnWheelsConstants.FEED_FORWARD)
    .withSlot(ColumnWheelsConstants.SLOT)
    .withOverrideBrakeDurNeutral(ColumnWheelsConstants.OVERRIDE_BRAKE_DUR_NEUTRAL);
    m_IO.setVelocityVoltage(request);
  }

  public void resetControllers() {
    m_IO.resetSlot0Gains();
  }

  public void resetPosition() {
    m_IO.resetPosition();
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("column/current", m_IO.getCurrent());
  }

  public Command runRollerFactory() {
    return this.run(() -> runClosedLoopVelocity(ColumnWheelsConstants.INTAKE_VELOCITY));
  }
}