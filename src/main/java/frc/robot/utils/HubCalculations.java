package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.FieldConstants;

public class HubCalculations {

    public static Translation2d hubTranslation = AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());

    public static Rotation2d angleToHub(Pose2d robotPose) {
        Translation2d hub = hubTranslation;

        Translation2d translationToTarget = hub.minus(robotPose.getTranslation());

        // If we're already at the target (zero distance), keep the current heading
        if (translationToTarget.getNorm() < DrivetrainConstants.SNAP_TO_TARGET_DISTANCE_THRESHOLD) {
            return robotPose.getRotation();
        }
        
        return hub.minus(robotPose.getTranslation()).getAngle();
    }

    public static double distanceToHub(Pose2d robotPose) {
        Translation2d hub = hubTranslation;
        return hub.getDistance(robotPose.getTranslation());
    }
}
