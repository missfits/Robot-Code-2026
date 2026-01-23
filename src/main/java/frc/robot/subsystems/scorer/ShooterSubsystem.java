package frc.robot.subsystems.scorer;
import com.ctre.phoenix6.controls.VelocityVoltage;

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
    return loggedCommand(
        "runShooter",
        this.run(() -> m_IO.setVoltage(speed))
    );
  }
  public Command velocityVoltage(double velocity, boolean enableFOC, double feedForward, int slot, boolean overrideBrakeDurNeutral) {
    VelocityVoltage request = new VelocityVoltage(velocity)
        .withEnableFOC(enableFOC)
        .withFeedForward(feedForward)
        .withSlot(slot)
        .withOverrideBrakeDurNeutral(overrideBrakeDurNeutral);

    return loggedCommand(
      "velocityVoltage",
      this.run(() -> m_IO.setVelocityVoltage(request))
  );
}

  public Command runShooterOff() {
    return loggedCommand(
      "runShooterOff",
      new RunCommand(() -> m_IO.setVoltage(0), this)
      );
  }

  private Command loggedCommand(String name, Command command) {
  return command.withName(name).beforeStarting(() ->
          SmartDashboard.putString("shooter/lastRunningInnerCommand", name)
      );
  }


  @Override
  public void periodic() {
    SmartDashboard.putData("shooter/subsystem", this);
    SmartDashboard.putNumber("shooter/current", m_IO.getCurrent());

    Command current = this.getCurrentCommand();
    SmartDashboard.putString(
      "shooter/currentlyRunningOuterCommand",
      current != null ? current.getName() : "None"
  );
}
}