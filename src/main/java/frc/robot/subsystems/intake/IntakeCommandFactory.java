package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;

public class IntakeCommandFactory {
    private IntakeSubsystem m_subsystem;

    public IntakeCommandFactory(IntakeSubsystem intake) {
        m_subsystem = intake;
    }

    public Command runIntake() {
        return m_subsystem.runIntake(IntakeConstants.OUTTAKE_MOTOR_VELOCITY);
    }

    public Command runIntakeBack() {
        return m_subsystem.runIntake(IntakeConstants.INTAKE_BACK_VELOCITY);
    }

    public Command runIntakeWithTimeout() {
        return m_subsystem.runIntake(IntakeConstants.OUTTAKE_MOTOR_VELOCITY)
            .withTimeout(IntakeConstants.RUN_INTAKE_TIME);
    }

    public Command intakeOff() {
        return m_subsystem.runIntakeOff();
    }
}
