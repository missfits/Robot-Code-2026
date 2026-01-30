package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.MechanismsSubsystemBase;


public class ClimberSubsystem extends MechanismsSubsystemBase {
  private final ClimberIOHardware m_IO = new ClimberIOHardware(ClimberConstants.CLIMBER_MOTOR_ID);

  public ClimberSubsystem() {
    super("climber");
    m_IO.resetPosition();
  }

  protected void setVoltage(double volts) {
    m_IO.setVoltage(volts);
  }

  @Override
  protected void runClosedLoopVelocity(double velocity) {
    VelocityVoltage request = new VelocityVoltage(velocity)
    .withEnableFOC(ClimberConstants.CLIMBER_ENABLE_FOC)
    .withFeedForward(ClimberConstants.CLIMBER_FEED_FORWARD)
    .withSlot(ClimberConstants.CLIMBER_SLOT)
    .withOverrideBrakeDurNeutral(ClimberConstants.CLIMBER_OVERRIDE_BRAKE_DUR_NEUTRAL);
    m_IO.setVelocityVoltage(request);
  }

  public void resetPosition() {
    m_IO.resetPosition();
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("climber/current", m_IO.getCurrent());
  }
}