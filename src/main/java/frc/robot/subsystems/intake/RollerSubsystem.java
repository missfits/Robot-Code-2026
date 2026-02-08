package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;

public class RollerSubsystem extends MechanismsSubsystemBase {
  private final RollerIOHardware m_IO = new RollerIOHardware(IntakeConstants.ROLLER_MOTOR_ID);

  public RollerSubsystem() {
    super("roller");
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

  public void resetControllers() {
    m_IO.resetSlot0Gains();
  }

  public void resetPosition() {
    m_IO.resetPosition();
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("roller/current", m_IO.getCurrent());
  }

  public Command runRollerFactory() {
    return this.run(() -> runClosedLoopVelocity(IntakeConstants.ROLLER_INTAKE_VELOCITY));
  }
}