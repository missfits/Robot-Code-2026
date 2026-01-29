package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class MechanismsSubsystemBase extends SubsystemBase {
  private final String mechanismDashboardName;

  /** 
   * @param name: the WPILib subsystem name; it's used by the command scheduler, command naming, etc.
   *
   * @param mechanismDashboardName: a string that's used exclusively for logging; allows for 
   * consistent logging paths across different mechanisms
   */
  protected MechanismsSubsystemBase(String name, String mechanismDashboardName){
    setName(name);
    this.mechanismDashboardName = mechanismDashboardName;
  }

  //logs the LAST INNER COMMAND that was scheduled on this subsystem, and runs ONCE when the command starts
  protected Command loggedCommand(String name, Command command) {
    return command.withName(name).beforeStarting(() 
    -> SmartDashboard.putString(mechanismDashboardName + "/lastRunningInnerCommand", name)
    );
  }

  protected abstract void setVoltage(double volts);

  public Command runMechanism(double volts) {
    return loggedCommand("run" + mechanismDashboardName, this.run(() -> setVoltage(volts)));
  }

  public Command runMechanismWithTimeout(double volts, double time) {
    return loggedCommand("run" + mechanismDashboardName + "for " + time + "sec", this.run(() -> {setVoltage(volts);})
    .withTimeout(time));
  }

  public Command runMechanismOff() {
    return loggedCommand("run" + mechanismDashboardName + "off", new RunCommand(() -> setVoltage(0), this));
  }

  protected abstract void applyVelocityVoltage(double velocity);

  protected Command velocityVoltage(double velocity) {
    return loggedCommand("velocityVoltage", this.run(() -> applyVelocityVoltage(velocity)));
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(mechanismDashboardName + "/subsystem", this);
    Command current = getCurrentCommand();

    //logs the OUTER command currently scheduled on this subsystem, and is logged every loop of periodic
    SmartDashboard.putString(mechanismDashboardName + "/currentlyRunningOuterCommand",
    current != null ? current.getName() : "None");
  }
}