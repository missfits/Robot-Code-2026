package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.subsystems.LaserCANSensorBase;

public class IntakeCommandFactory {
  private RollerSubsystem m_roller;
  private LaserCANSensorBase m_intakeSensor;

  public IntakeCommandFactory(RollerSubsystem roller, LaserCANSensorBase intakeSensor) {
    m_roller = roller;
    m_intakeSensor = intakeSensor;
  }

  public Command runIntake() {
    return m_roller.runMechanism(IntakeRollerConstants.INTAKE_VELOCITY).withName("run intake");
  }

  public Command runIntakeBack() {
    return m_roller.runMechanism(IntakeRollerConstants.INTAKE_BACK_VELOCITY).withName("run intake");
  }

  public Command runIntakeWithTimeout() {
    return m_roller.runMechanism(IntakeRollerConstants.INTAKE_VELOCITY)
    .withTimeout(IntakeConstants.RUN_INTAKE_TIME)
    .withName("run intake timeout");
  }

  public Command runIntakePID() {
    return m_roller.runMechanismPID(IntakeRollerConstants.INTAKE_VELOCITY)
      .withName("run intake PID");
  }

  public Command intakeOff() {
    return m_roller.runMechanismOff().withName("intake off");
  }
}
