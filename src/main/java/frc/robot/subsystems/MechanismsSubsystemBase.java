package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class MechanismsSubsystemBase extends SubsystemBase {
  private final String mechanismName;

  protected MechanismsSubsystemBase(String mechanismName){
    setName(mechanismName);
    this.mechanismName = mechanismName;
  }

  //logs the LAST INNER COMMAND that was scheduled on this subsystem, and runs ONCE when the command starts
  protected Command loggedCommand(String name, Command command) {
    return command.withName(name).beforeStarting(() 
    -> SmartDashboard.putString(mechanismName + "/lastRunningInnerCommand", name)
    );
  }

  protected abstract void setVoltage(double volts);

  public Command runMechanism(double volts) {
    return loggedCommand("run" + mechanismName, this.run(() -> setVoltage(volts)));
  }

  public Command runMechanismWithTimeout(double volts, double time) {
    return loggedCommand("run" + mechanismName + "for " + time + "sec", this.run(() -> {setVoltage(volts);})
    .withTimeout(time));
  }

  public Command runMechanismOff() {
    return loggedCommand("run" + mechanismName + "off", new RunCommand(() -> setVoltage(0), this));
  }

  protected void runClosedLoopVelocity(double velocity) {
    throw new UnsupportedOperationException(getName() + " does not support single-motor velocity control");
  }

  public Command runMechanismPID(double velocity) {
    return loggedCommand("run" + mechanismName + "PID", this.run(() -> runClosedLoopVelocity(velocity)));
  }

  //need a second runClosedLoopVelocity command for subsystems with two motors
  protected void runClosedLoopVelocity(double velocityOne, double velocityTwo) {
    throw new UnsupportedOperationException(getName() + " does not support dual-motor velocity control");
  }

  //need a second runMechanismPID command for subsystems with two motors
  public Command runMechanismPID(double velocityOne, double velocityTwo) {
    return loggedCommand("run" + mechanismName + "PID", this.run(() -> runClosedLoopVelocity(velocityOne, velocityTwo)));
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(mechanismName + "/subsystem", this);
    Command current = getCurrentCommand();

    //logs the OUTER command currently scheduled on this subsystem, and is logged every loop of periodic
    SmartDashboard.putString(mechanismName + "/currentlyRunningOuterCommand",
    current != null ? current.getName() : "None");
  }
}