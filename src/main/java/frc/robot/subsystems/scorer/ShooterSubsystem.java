package frc.robot.subsystems.scorer;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIOHardware m_IO = new ShooterIOHardware(ShooterConstants.MECHANISM_MOTOR_ID);

  public ShooterSubsystem() {
        m_IO.resetPosition();
  }

  public Command runShooter(double speed) {
        return new FunctionalCommand(
            // set voltage in init also
            () -> {m_IO.setVoltage(speed); SmartDashboard.putString("shooter/currentlyRunningCommand", "runShooter");},
            () -> {m_IO.setVoltage(speed); SmartDashboard.putString("shooter/currentlyRunningCommand", "runShooter");},
            (interrupted) -> {},
            () -> false,
            this
        ).withName("runShooter");
    }

  public Command VelocityVoltage(double velocity, boolean enableFOC, double feedForward, int slot, boolean overrideBrakeDurNeutral) {
    VelocityVoltage request = new VelocityVoltage(velocity)
        .withEnableFOC(enableFOC)
        .withFeedForward(feedForward)
        .withSlot(slot)
        .withOverrideBrakeDurNeutral(overrideBrakeDurNeutral);

    return this.run(() -> {
        m_IO.setVelocityVoltage(request);
        SmartDashboard.putNumber("shooter/velocity voltage", velocity);
    });
}

  public Command runShooterOff() {
    return new RunCommand(
      () -> {
        m_IO.setVoltage(0);
        SmartDashboard.putBoolean("shooter/off", true);
      },
      this
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("shooter/subsystem", this);
    SmartDashboard.putNumber("shooter/current", m_IO.getCurrent());
  }

}