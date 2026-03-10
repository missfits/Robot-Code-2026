package frc.robot.subsystems;

import java.util.function.Supplier;

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
    return command.beforeStarting(() 
    -> SmartDashboard.putString(mechanismName + "/lastRunningInnerCommand", name)
    ).withName(name);
  }
  /*
   * --------------------------------------------------------------
   * Command creation wrappers
   * 
   * Created logged commands to make it easier to not keep repeating code?
   */

  public Command run(Runnable action, String name){
    Command superClassCommand = super.run(action);
    return this.loggedCommand(name, superClassCommand);
  }

  public Command startEnd(Runnable start, Runnable end, String name){
    Command superClassCommand = super.startEnd(start, end);
    return this.loggedCommand(name, superClassCommand);
  }

  public Command runEnd(Runnable run, Runnable end, String name){
    Command superClassCommand = super.runEnd(run, end);
    return this.loggedCommand(name, superClassCommand);
  }
 
  public Command startRun(Runnable start, Runnable run, String name){
    Command superClassCommand = super.startRun(start,run);
    return this.loggedCommand(name, superClassCommand);
  }

  public Command defer(Supplier <Command> supplier, String name){
    Command superClassCommand = super.defer(supplier);
    return this.loggedCommand(name, superClassCommand);
  }

  /*
   * -----------------------------------------------------------------
   */

  protected abstract void setVoltage(double volts);

  // voltage
  public Command voltageCommand(double volts) {
    return this.run(() -> setVoltage(volts), mechanismName + "voltageCommand");
  }

  public Command voltageCommandWithTimeout(double volts, double time) {
    return this.run(() -> {setVoltage(volts);}, mechanismName + "voltageCommand with " + time + " second timeout").withTimeout(time);
  }

  public Command voltageCommand(Supplier<Double> voltageSupplier) {
    return this.run(() -> setVoltage(voltageSupplier.get()), mechanismName + "voltageCommand with supplier");
  }

  public Command voltageCommandWithTimeout(Supplier<Double> voltageSupplier, double time) {
    return this.run(() -> {setVoltage(voltageSupplier.get());}, mechanismName + "voltageCommand with supplier and " + time + " second timeout").withTimeout(time);
  }

  // velocity
  public Command velocityCommand(double velocity) {
    return this.run(() -> runClosedLoopVelocity(velocity), mechanismName + "velocityCommand");
  } 

  public Command velocityCommand(double velocityOne, double velocityTwo) {
    return this.run(()-> runClosedLoopVelocity(velocityOne, velocityTwo), mechanismName + "velocityCommand");
  }

  public Command velocityCommand(Supplier<Double> velocitySupplier) {
    return this.run(() -> runClosedLoopVelocity(velocitySupplier.get()), mechanismName + "velocityCommand with supplier");
  }

  public Command velocityCommand(Supplier<Double> velocitySupplierOne, Supplier<Double> velocitySupplierTwo) {
    return this.run(() -> runClosedLoopVelocity(velocitySupplierOne.get(), velocitySupplierTwo.get()), mechanismName + "velocityCommand with two suppliers");
  }

  protected void runClosedLoopVelocity(double velocity) {
    throw new UnsupportedOperationException(getName() + " does not support single-motor velocity control");
  }

  protected void runClosedLoopVelocity(double velocityOne, double velocityTwo) {
    throw new UnsupportedOperationException(getName() + " does not support dual-motor velocity control");
  }

  // off
  public Command offCommand() {
    return loggedCommand(mechanismName + "off", new RunCommand(() -> setVoltage(0), this));
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