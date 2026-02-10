package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;

public class RollerSubsystem extends MechanismsSubsystemBase {
  private final RollerIOHardware m_IO = new RollerIOHardware(IntakeRollerConstants.MOTOR_ID);

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
    .withEnableFOC(IntakeRollerConstants.ENABLE_FOC)
    .withFeedForward(IntakeRollerConstants.FEED_FORWARD)
    .withSlot(IntakeRollerConstants.SLOT)
    .withOverrideBrakeDurNeutral(IntakeRollerConstants.OVERRIDE_BRAKE_DUR_NEUTRAL);
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
    return this.run(() -> runClosedLoopVelocity(IntakeRollerConstants.INTAKE_VELOCITY));
  }
}