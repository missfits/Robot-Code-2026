package frc.robot.subsystems.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;

public class ShooterCommandFactory {
    private ShooterSubsystem m_subsystem;

    public ShooterCommandFactory(ShooterSubsystem shooter) {
        m_subsystem = shooter;
    }

    public Command runShooter() {
        return m_subsystem.runShooter(
            ShooterConstants.INFLUENCER_OUTTAKE_MOTOR_VELOCITY,
            ShooterConstants.FOLLOWER_OUTTAKE_MOTOR_VELOCITY
        );
    }

    public Command runShooterBack() {
        return m_subsystem.runShooter(
            ShooterConstants.INFLUENCER_SHOOTER_BACK_VELOCITY,
            ShooterConstants.FOLLOWER_SHOOTER_BACK_VELOCITY
        );
    }

    public Command runShooterWithTimeout() {
        return m_subsystem.runShooter(
            ShooterConstants.INFLUENCER_OUTTAKE_MOTOR_VELOCITY,
            ShooterConstants.FOLLOWER_OUTTAKE_MOTOR_VELOCITY
        ).withTimeout(ShooterConstants.RUN_SHOOTER_TIME);
    }

    public Command shooterOff() {
        return m_subsystem.runShooterOff();
    }
}
