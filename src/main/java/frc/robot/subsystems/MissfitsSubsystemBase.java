package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class MissfitsSubsystemBase extends SubsystemBase {
    private final String mechanismDashboardName;

    protected MissfitsSubsystemBase(String name, String mechanismDashboardName){
        setName(name);
        this.mechanismDashboardName = mechanismDashboardName;
    }

    protected Command loggedCommand(String name, Command command) {
        return command.withName(name).beforeStarting(() 
        -> SmartDashboard.putString(mechanismDashboardName + "/lastRunningInnerCommand", name)
        );
    }

    @Override
    public void periodic() {
    SmartDashboard.putData(mechanismDashboardName + "/subsystem", this);

    Command current = getCurrentCommand();
    SmartDashboard.putString(
        mechanismDashboardName + "/currentlyRunningOuterCommand",
        current != null ? current.getName() : "None"
    );
  }
}