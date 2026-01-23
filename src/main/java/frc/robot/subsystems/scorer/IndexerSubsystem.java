<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/scorer/IndexerSubsystem.java
package frc.robot.subsystems.scorer;
import com.ctre.phoenix6.controls.VelocityVoltage;

========
package frc.robot.subsystems.mechanism;
>>>>>>>> 93d25e4ce2f6d3f1b224eaf70377d462fe1c0bde:src/main/java/frc/robot/subsystems/mechanism/IntakeSubsystem.java
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/scorer/IndexerSubsystem.java

import frc.robot.Constants.ScorerConstants;
========
import frc.robot.Constants.IntakeConstants;
>>>>>>>> 93d25e4ce2f6d3f1b224eaf70377d462fe1c0bde:src/main/java/frc/robot/subsystems/mechanism/IntakeSubsystem.java


public class IndexerSubsystem extends SubsystemBase {
  private final IndexerIOHardware m_IO = new IndexerIOHardware(ScorerConstants.INDEXER_MOTOR_ID);

  public IndexerSubsystem() {
    m_IO.resetPosition();
  }

<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/scorer/IndexerSubsystem.java
  public Command runIndexer(double velocity) {
    return this.run(() -> {
        m_IO.setVoltage(velocity);
        SmartDashboard.putNumber("intake/input velocity", velocity);
    });
  }

  public Command runIndexerPID(double velocity) {
    return this.run(() -> {
        m_IO.setVelocityVoltage(velocity);
        SmartDashboard.putNumber("intake/input velocity", velocity);
    });
  }

  public Command runIndexerOff() {
    return new RunCommand(() -> {
========
  public Command runIntake(double speed) {
        return new FunctionalCommand(
            // set voltage in init also
            () -> {m_IO.setVoltage(speed); SmartDashboard.putString("intake/currentlyRunningCommand", "runIntake");},
            () -> {m_IO.setVoltage(speed); SmartDashboard.putString("intake/currentlyRunningCommand", "runIntake");},
            (interrupted) -> {},
            () -> false,
            this
        ).withName("runIntake");
    }

  public Command runIntakeOff() {
    return new RunCommand(
      () -> {
>>>>>>>> 93d25e4ce2f6d3f1b224eaf70377d462fe1c0bde:src/main/java/frc/robot/subsystems/mechanism/IntakeSubsystem.java
        m_IO.setVoltage(0);
        SmartDashboard.putBoolean("intake/off", true);
      },
      this
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("indexer/subsystem", this);
    SmartDashboard.putNumber("indexer/current", m_IO.getCurrent());
  }
}