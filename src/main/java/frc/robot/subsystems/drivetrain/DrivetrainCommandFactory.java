package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer.JoystickVals;
import frc.robot.utils.AllianceFlipUtil;

public class DrivetrainCommandFactory {
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle m_driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.Velocity).withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    private final SwerveRequest.PointWheelsAt m_point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

    private final CommandSwerveDrivetrain m_drivetrain;

    private BooleanSupplier slowmodeSupplier = () -> false;

    private Rotation2d targetAngle = new Rotation2d();

    public DrivetrainCommandFactory(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;

        setHeadingController();
    }

    public void setSlowmodeButton(Trigger slowmodeButton) {
        slowmodeSupplier = () -> slowmodeButton.getAsBoolean();
    }

    // ----- DEFAULT DRIVE -----
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    public Command defaultDrive(Supplier<JoystickVals> translationSupplier, Supplier<JoystickVals> rotationSupplier) {

        return m_drivetrain.getCommandFromRequest(() -> {

            JoystickVals translation = translationSupplier.get();
            JoystickVals rotation = rotationSupplier.get();
            boolean slowmode = slowmodeSupplier.getAsBoolean();

            JoystickVals shapedTrans = Controls.inputShape(translation, true, slowmode);
            JoystickVals shapedRot = Controls.inputShape(rotation, false, slowmode);

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
    // Drives the robot while automatically rotating to face a specified angle in degrees
    public Command snapToAngle(Supplier<JoystickVals> translationSupplier, double angle) {
        return snapToAngle(translationSupplier, () -> Rotation2d.fromDegrees(angle));
    }

    public Command snapToAngle(Supplier<JoystickVals> translationSupplier, Rotation2d angle) {
        return snapToAngle(translationSupplier, () -> angle);
    }

    // Drives the robot while automatically rotating to face a specified rotation2d
    public Command snapToAngle(Supplier<JoystickVals> translationSupplier, Supplier<Rotation2d> angleSupplier) {
        return m_drivetrain.getCommandFromRequest(() -> {
            targetAngle = angleSupplier.get();
            SmartDashboard.putNumber("drivetrain/snapToAngle/targetAngle", targetAngle.getDegrees());
            JoystickVals translation = translationSupplier.get();
            boolean slowmode = slowmodeSupplier.getAsBoolean();

            JoystickVals shapedValues = Controls.inputShape(translation, true, slowmode);

            return m_driveFacingAngle.withVelocityX(-shapedValues.y() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive forward with negative Y (forward)
            .withVelocityY(-shapedValues.x() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive left with negative X (left)
            .withTargetDirection(targetAngle);
        }).withName("snapToAngle");
    }

    /**
     * Calculates the angle from a reference pose to a target pose
     * @param referencePose The reference pose (typically robot pose)
     * @param targetPose The target pose
     * @return The angle from reference to target, or the current heading if already at the target
     */
    private static Rotation2d calculateAngleToTarget(Pose2d referencePose, Pose2d targetPose) {
        Translation2d translationToTarget = targetPose.getTranslation().minus(referencePose.getTranslation());

        // If we're already at the target (zero distance), keep the current heading
        if (translationToTarget.getNorm() < DrivetrainConstants.SNAP_TO_TARGET_DISTANCE_THRESHOLD) {
            return referencePose.getRotation();
        }

        return translationToTarget.getAngle();
    }

    // ---- SNAP TO TARGET -----
    // Drives the robot while automatically rotating to face a target pose
    public Command snapToTarget(Supplier<JoystickVals> translationSupplier, Supplier<Pose2d> targetPoseSupplier) {
        return m_drivetrain.getCommandFromRequest(() -> {

            JoystickVals translation = translationSupplier.get();
            boolean slowmode = slowmodeSupplier.getAsBoolean();

            JoystickVals shapedValues = Controls.inputShape(translation, true, slowmode);

            Pose2d targetPose = targetPoseSupplier.get(); // target pose

            Rotation2d angleToTarget = calculateAngleToTarget(m_drivetrain.getState().Pose, targetPose);
            targetAngle = angleToTarget;

            SmartDashboard.putNumber("drivetrain/snap to target/target x", targetPose.getX());
            SmartDashboard.putNumber("drivetrain/snap to target/target y", targetPose.getY());
            SmartDashboard.putNumber("drivetrain/snap to target/angle", angleToTarget.getRadians());

            return m_driveFacingAngle.withVelocityX(-shapedValues.y() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive forward with negative Y (forward)
                .withVelocityY(-shapedValues.x() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive left with negative X (left)
                .withTargetDirection(angleToTarget);
        }).withName("snapToTarget");
    }

    public Command snapToBump(Supplier<JoystickVals> translationSupplier) {
        return snapToAngle(translationSupplier, () -> getBumpAngle(m_drivetrain.getState().Pose)).withName("snapForBump");
    }

    /**
     * Determines the field-relative heading the robot should use to face the bump.
     * @param robotPose The robot's current field pose
     * @return The field-relative heading the robot should face
     */
    private Rotation2d getBumpAngle(Pose2d robotPose) {
        Pose2d blueAlliancePose = AllianceFlipUtil.apply(robotPose); // Normalize to the blue-alliance perspective to check with blue hub
        Rotation2d blueAllianceHeading;
        if (blueAlliancePose.getX() < FieldConstants.LinesVertical.hubCenter) { // If in alliance zone
            blueAllianceHeading = Rotation2d.fromDegrees(0); // Face away from the driver station.
        } else {
            blueAllianceHeading = Rotation2d.fromDegrees(180); // Face toward the driver station.
        }
        return AllianceFlipUtil.apply(blueAllianceHeading); // Convert heading back for the current alliance
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
    public Command pointWheelsAt(Supplier<JoystickVals> joystickSupplier) {
        return m_drivetrain.getCommandFromRequest(() -> {
            JoystickVals vals = joystickSupplier.get();
            return m_point.withModuleDirection(new Rotation2d(-vals.y(), -vals.x()));
        });
    }

    private boolean atAngle(Supplier<Rotation2d> angleSupplier) {
        return Math.abs(m_drivetrain.getState().Pose.getRotation().minus(angleSupplier.get()).getRadians()) < DrivetrainConstants.ANGLE_TOLERANCE;
    }

    public Trigger atTargetAngleTrigger() {
        return new Trigger(() -> atAngle(() -> targetAngle));
    }

    public Trigger atAngleTrigger(Supplier<Rotation2d> angleSupplier) {
        return new Trigger(() -> atAngle(angleSupplier));
    }

    // ----- SYSID -----
    public Command sysIdQuasistaticTranslationForward() {
        return m_drivetrain.sysIdQuasistaticTranslation(Direction.kForward);
    }
    
    public Command sysIdQuasistaticTranslationReverse() {
        return m_drivetrain.sysIdQuasistaticTranslation(Direction.kReverse);
    }

    public Command sysIdDynamicTranslationForward() {
        return m_drivetrain.sysIdDynamicTranslation(Direction.kForward);
    }

    public Command sysIdDynamicTranslationReverse() {
        return m_drivetrain.sysIdDynamicTranslation(Direction.kReverse);
    }

    public Command sysIdQuasistaticRotationForward() {
        return m_drivetrain.sysIdQuasistaticRotation(Direction.kForward);
    }

    public Command sysIdQuasistaticRotationReverse() {
        return m_drivetrain.sysIdQuasistaticRotation(Direction.kReverse);
    }

    public Command sysIdDynamicRotationForward() {
        return m_drivetrain.sysIdDynamicRotation(Direction.kForward);
    }

    public Command sysIdDynamicRotationReverse() {
        return m_drivetrain.sysIdDynamicRotation(Direction.kReverse);
    }
}
