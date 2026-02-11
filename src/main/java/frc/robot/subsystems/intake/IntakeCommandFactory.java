package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.LaserCANSensorBase;

public class IntakeCommandFactory {
  private RollerSubsystem m_roller;
  private LaserCANSensorBase m_intakeSensor;

  public IntakeCommandFactory(RollerSubsystem roller, LaserCANSensorBase intakeSensor) {
    m_roller = roller;
    m_intakeSensor = intakeSensor;
  }

  public Command runIntake() {
    return m_roller.runMechanism(RollerConstants.INTAKE_VELOCITY).withName("run intake");
  }

  public Command runIntakeBack() {
    return m_roller.runMechanism(RollerConstants.INTAKE_BACK_VELOCITY).withName("run intake");
  }

  public Command runIntakeWithTimeout() {
    return m_roller.runMechanism(RollerConstants.INTAKE_VELOCITY)
    .withTimeout(RollerConstants.RUN_INTAKE_TIME)
    .withName("run intake timeout");
  }

  public Command runIntakePID() {
    return m_roller.runMechanismPID(RollerConstants.INTAKE_VELOCITY)
      .withName("run intake PID");
  }

  public Command intakeOff() {
    return m_roller.runMechanismOff().withName("intake off");
  }
}
