package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.FieldConstants;

public class HubCalculations {

    public static Supplier<Translation2d> hubTranslationSupplier = () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());

    public static Rotation2d angleToHub(Pose2d robotPose) {
        Translation2d hub = hubTranslationSupplier.get();

        Translation2d translationToTarget = hub.minus(robotPose.getTranslation());

        // If we're already at the target (zero distance), keep the current heading
        if (translationToTarget.getNorm() < DrivetrainConstants.SNAP_TO_TARGET_DISTANCE_THRESHOLD) {
            return robotPose.getRotation();
        }

        return hub.minus(robotPose.getTranslation()).getAngle();
    }

    public static double distanceToHub(Pose2d robotPose) {
        Translation2d hub = hubTranslationSupplier.get();
        return hub.getDistance(robotPose.getTranslation());
    }

    /**
     * Calculates the required drivetrain angle for shooting on the fly with a FIXED shooter.
     *
     * Uses the lookup table to get the appropriate exit velocity for the distance,
     * then calculates the horizontal component and applies vector subtraction.
     *
     * @param robotPose Current robot pose
     * @param robotSpeeds Current robot speeds (field-relative)
     * @return Rotation2d drivetrain angle, or null if distance is out of range
     */
    public static Rotation2d calculateShootOnTheFlyAngle(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        return calculateShootOnTheFlyAngleToTarget(robotPose, robotSpeeds, hubTranslationSupplier);
    }

    /**
     * Generalized shoot-on-the-fly angle calculation for any target location.
     *
     * @param robotPose Current robot pose
     * @param robotSpeeds Current robot speeds (field-relative)
     * @param targetLocationSupplier Supplier for target location on the field
     * @return Rotation2d drivetrain angle, or null if distance is out of range
     */
    private static Rotation2d calculateShootOnTheFlyAngleToTarget(Pose2d robotPose, ChassisSpeeds robotSpeeds, Supplier<Translation2d> targetLocationSupplier) {
        // 1. LATENCY COMPENSATION - Project robot position forward
        Translation2d futurePos = robotPose.getTranslation().plus(
            new Translation2d(
                robotSpeeds.vxMetersPerSecond * TeleopConstants.LATENCY_COMPENSATION,
                robotSpeeds.vyMetersPerSecond * TeleopConstants.LATENCY_COMPENSATION));

        // 2. GET TARGET VECTOR
        Translation2d targetLocation = targetLocationSupplier.get();
        Translation2d targetVec = targetLocation.minus(futurePos);
        double distance = targetVec.getNorm();

        // 3. GET EXIT VELOCITY FROM LOOKUP TABLE
        java.util.Optional<Double> rpmOptional = ShooterLookupTable.getVelocityForDistance(distance);
        if (!rpmOptional.isPresent()) {
            return null; // Distance out of range
        }

        // Convert RPS to m/s exit velocity
        double exitVelocityMPS = rpmOptional.get() * ShooterConstants.SHOOTER_RPS_TO_MPS * ShooterConstants.SHOOTER_SLIP_FACTOR;

        // 4. CALCULATE HORIZONTAL VELOCITY from exit velocity and fixed shooter angle
        // For a fixed shooter: horizontal velocity = exit velocity * cos(angle)
        double shooterAngleRad = Math.toRadians(ShooterConstants.SHOOTER_ANGLE_DEGREES);
        double idealHorizontalSpeed = exitVelocityMPS * Math.cos(shooterAngleRad);

        // 5. VECTOR SUBTRACTION: V_shot = V_target - V_robot
        Translation2d robotVelVec = new Translation2d(
            robotSpeeds.vxMetersPerSecond,
            robotSpeeds.vyMetersPerSecond);

        return targetVec.div(distance).times(idealHorizontalSpeed).minus(robotVelVec).getAngle();
    }

    /**
     * Gets the shuttle target corners with offset (blue alliance perspective).
     * Returns two corners on the alliance side (low X value).
     *
     * @return Array of two Translation2d representing [left corner, right corner]
     */
    private static Translation2d[] getShuttleCorners() {
        // Alliance corners are on our side (low X value)
        // Left corner: low X, high Y
        Translation2d leftCorner = new Translation2d(
            TeleopConstants.X_CORNER_OFFSET,
            FieldConstants.fieldWidth - TeleopConstants.Y_CORNER_OFFSET
        );

        // Right corner: low X, low Y
        Translation2d rightCorner = new Translation2d(
            TeleopConstants.X_CORNER_OFFSET,
            TeleopConstants.Y_CORNER_OFFSET
        );

        return new Translation2d[] { leftCorner, rightCorner };
    }

    /**
     * Finds the closest shuttle corner to the robot.
     *
     * @param robotPose Current robot pose
     * @return Translation2d of the closest corner (alliance-flipped)
     */
    public static Translation2d getClosestShuttleCorner(Pose2d robotPose) {
        Translation2d[] corners = getShuttleCorners();

        // Apply alliance flip to corners
        Translation2d leftCorner = AllianceFlipUtil.apply(corners[0]);
        Translation2d rightCorner = AllianceFlipUtil.apply(corners[1]);

        // Find closest corner
        double distToLeft = robotPose.getTranslation().getDistance(leftCorner);
        double distToRight = robotPose.getTranslation().getDistance(rightCorner);

        return distToLeft < distToRight ? leftCorner : rightCorner;
    }

    /**
     * Calculates the distance to the closest shuttle corner.
     *
     * @param robotPose Current robot pose
     * @return Distance in meters to the closest shuttle corner
     */
    public static double distanceToShuttleCorner(Pose2d robotPose) {
        Translation2d closestCorner = getClosestShuttleCorner(robotPose);
        return robotPose.getTranslation().getDistance(closestCorner);
    }

    /**
     * Calculates the required drivetrain angle for shuttling with SOTM compensation.
     * Aims at the closest corner of the field.
     *
     * @param robotPose Current robot pose
     * @param robotSpeeds Current robot speeds (field-relative)
     * @return Rotation2d drivetrain angle, or null if distance is out of range
     */
    public static Rotation2d calculateShuttleAngle(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        Supplier<Translation2d> shuttleTargetSupplier = () -> getClosestShuttleCorner(robotPose);
        return calculateShootOnTheFlyAngleToTarget(robotPose, robotSpeeds, shuttleTargetSupplier);
    }
}
