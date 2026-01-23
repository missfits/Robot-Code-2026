package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;

public class IntakeCommandFactory {
  private RollerSubsystem m_roller;

  public IntakeCommandFactory(RollerSubsystem roller) {
    m_roller = roller;
  }

  public Command runIntake() {
    return m_roller.runRoller(IntakeConstants.OUTTAKE_MOTOR_VELOCITY)
      .withName("run intake");
  }

  public Command runIntakeBack() {
    return m_roller.runRoller(IntakeConstants.INTAKE_BACK_VELOCITY)
      .withName("run intake back");
  }

  public Command runIntakeWithTimeout() {
    return m_roller.runRoller(IntakeConstants.OUTTAKE_MOTOR_VELOCITY)
      .withTimeout(IntakeConstants.RUN_INTAKE_TIME).withName("run intake timeout");
  }

  public Command runIntakePID() { 
    return m_roller.runRollerPID(IntakeConstants.OUTTAKE_MOTOR_VELOCITY)
      .withName("run intake PID");
  }

  public Command intakeOff() {
    return m_roller.runRollerOff().withName("intake off");
  }
}
