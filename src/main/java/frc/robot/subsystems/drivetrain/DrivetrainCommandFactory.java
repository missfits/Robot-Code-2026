package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    public Command defaultDrive(CommandXboxController controller, BooleanSupplier slowmodeSupplier) {

        return m_drivetrain.getCommandFromRequest(() -> {

            JoystickVals shapedTrans = Controls.inputShape(controller.getLeftX(), controller.getLeftY(), true, slowmodeSupplier.getAsBoolean());
            JoystickVals shapedRot = Controls.inputShape(controller.getRightX(), controller.getRightY(), false, slowmodeSupplier.getAsBoolean());

            SmartDashboard.putNumber("controller/translation x", -shapedTrans.y());
            SmartDashboard.putNumber("controller/translation y", -shapedTrans.x());
            SmartDashboard.putNumber("controller/rotation x", -shapedRot.x());
            SmartDashboard.putNumber("controller/rotation y", -shapedRot.y());

            return m_drive.withVelocityX(-shapedTrans.y() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive forward with negative Y (forward)
                .withVelocityY(-shapedTrans.x() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive left with negative X (left)
                .withRotationalRate(-shapedRot.x() * DrivetrainConstants.MAX_ROTATION_SPEED); // Drive counterclockwise with negative X (left)
            }
            );
    }

    // ----- SNAP TO ANGLE -----
    public Command snapToAngle(CommandXboxController joystick, double angle) {
        return m_drivetrain.getCommandFromRequest(() -> {
            SmartDashboard.putNumber("drivetrain/snap to angle", angle);
            JoystickVals shapedValues = Controls.inputShape(joystick.getLeftX(), joystick.getLeftY(), true, false);
            
            return m_driveFacingAngle.withVelocityX(-shapedValues.y() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive forward with negative Y (forward)
            .withVelocityY(-shapedValues.x() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive left with negative X (left)
            .withTargetDirection(Rotation2d.fromDegrees(angle));
        });
    }

    // ---- SNAP TO TARGET -----
    public Command snapToTarget(CommandXboxController joystick, Supplier<Pose2d> poseSupplier) {
        return m_drivetrain.getCommandFromRequest(() -> {

        JoystickVals shapedValues = Controls.inputShape(joystick.getLeftX(), joystick.getLeftY(), true, false);

        Pose2d robotPose = m_drivetrain.getState().Pose; // current robot pose
        Pose2d targetPose = poseSupplier.get(); // target pose

        // Get the translation from robot to target
        Translation2d translationToTarget = targetPose.getTranslation().minus(robotPose.getTranslation());

        // Get the angle to the target
        Rotation2d angleToTarget = translationToTarget.getAngle();

        SmartDashboard.putNumber("drivetrain/snap to target/target x", targetPose.getX());
        SmartDashboard.putNumber("drivetrain/snap to target/target y", targetPose.getY());
        SmartDashboard.putNumber("drivetrain/snap to target/angle", angleToTarget.getRadians());


        return m_driveFacingAngle.withVelocityX(-shapedValues.y() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive forward with negative Y (forward)
            .withVelocityY(-shapedValues.x() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive left with negative X (left)
            .withTargetDirection(angleToTarget);
        });
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
