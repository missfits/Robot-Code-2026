package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotContainer.JoystickVals;

public class DrivetrainCommandFactory {
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle m_driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.Velocity).withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    private final SwerveRequest.PointWheelsAt m_point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

    private final CommandSwerveDrivetrain m_drivetrain;

    public DrivetrainCommandFactory(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    // ----- DEFAULT DRIVE -----
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    public Command defaultDrive(JoystickVals transVals, JoystickVals rotVals, boolean slowmode) {
        JoystickVals shapedTrans = Controls.inputShape(transVals.x(), transVals.y(), true, slowmode);
        JoystickVals shapedRot = Controls.inputShape(rotVals.x(), rotVals.y(), false, slowmode);

        SmartDashboard.putNumber("controller/translation x", -shapedTrans.y());
        SmartDashboard.putNumber("controller/translation y", -shapedTrans.x());
        SmartDashboard.putNumber("controller/rotation x", -shapedRot.x());
        SmartDashboard.putNumber("controller/rotation y", -shapedRot.y());

        return m_drivetrain.getCommandFromRequest(() ->
            m_drive.withVelocityX(-shapedTrans.y() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive forward with negative Y (forward)
                .withVelocityY(-shapedTrans.x() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive left with negative X (left)
                .withRotationalRate(-shapedRot.x() * DrivetrainConstants.MAX_ROTATION_SPEED) // Drive counterclockwise with negative X (left)
        );
    }

    // ----- SNAP TO ANGLE -----
    public Command snapToAngle(CommandXboxController joystick, double angle){
        SmartDashboard.putNumber("drivetrain/snap to angle", angle);
        JoystickVals shapedValues = Controls.inputShape(joystick.getLeftX(), joystick.getLeftY(), true, false);
        return m_drivetrain.getCommandFromRequest(() ->
        m_driveFacingAngle.withVelocityX(-shapedValues.y() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive forward with negative Y (forward)
            .withVelocityY(-shapedValues.x() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive left with negative X (left)
            .withTargetDirection(Rotation2d.fromDegrees(angle))
        );
    }

    public void setHeadingController(){
        m_driveFacingAngle.HeadingController = new PhoenixPIDController(DrivetrainConstants.ROTATION_KP, DrivetrainConstants.ROTATION_KI, DrivetrainConstants.ROTATION_KD);
        m_driveFacingAngle.HeadingController.enableContinuousInput(0, 2*Math.PI);
    }

    public Command resetRotation() {
        return new InstantCommand(() -> m_drivetrain.setRotation(0));
    }

    // ----- POINT WHEELS IN X -----
    public Command pointWheelsinX() {
        return m_drivetrain.getCommandFromRequest(() -> m_brake);
    }

    // ----- POINT -----
    public Command pointWheelsAt(JoystickVals vals) {
        return m_drivetrain.getCommandFromRequest(() -> 
            m_point.withModuleDirection(new Rotation2d(-vals.y(), -vals.x())));
    }
}
