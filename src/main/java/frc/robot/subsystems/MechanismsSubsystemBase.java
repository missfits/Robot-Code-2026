package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class MechanismsSubsystemBase extends SubsystemBase {
  private final String mechanismDashboardName;

  protected MechanismsSubsystemBase(String name, String mechanismDashboardName){
    setName(name);
    this.mechanismDashboardName = mechanismDashboardName;
  }

  protected Command loggedCommand(String name, Command command) {
    return command.withName(name).beforeStarting(() 
    -> SmartDashboard.putString(mechanismDashboardName + "/lastRunningInnerCommand", name)
    );
  }

  protected abstract void setVoltage(double volts);

  protected Command runMechanism(double speed){
    return loggedCommand("run"+mechanismDashboardName, this.run(() -> setVoltage(speed)));
  }

  protected Command runMechanismOff() {
    return loggedCommand("run"+mechanismDashboardName+"off", new RunCommand(() -> setVoltage(0), this));
  }

  protected abstract void applyVelocityVoltage(VelocityVoltage request);

  protected Command velocityVoltage(double velocity, boolean enableFOC, double feedForward, int slot, boolean overrideBrakeDurNeutral) {
    VelocityVoltage request = new VelocityVoltage(velocity)
    .withEnableFOC(enableFOC)
    .withFeedForward(feedForward)   
    .withSlot(slot)
    .withOverrideBrakeDurNeutral(overrideBrakeDurNeutral);
    return loggedCommand("velocityVoltage", this.run(() -> applyVelocityVoltage(request)));
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(mechanismDashboardName + "/subsystem", this);
    Command current = getCurrentCommand();
    SmartDashboard.putString(mechanismDashboardName + "/currentlyRunningOuterCommand",
    current != null ? current.getName() : "None");
  }
}