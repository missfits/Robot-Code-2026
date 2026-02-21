package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DrivetrainConstants;

public class HubCalculations {
    
    public static Translation2d blueAllianceHub = new Translation2d(Units.inchesToMeters(651.2-(158.6+(47./2))), 
                                                                   Units.inchesToMeters(317.7/2));
    public static Translation2d redAllianceHub = new Translation2d(Units.inchesToMeters(158.6+(47./2)), 
                                                                   Units.inchesToMeters(317.7/2));


    public static Rotation2d angleToHub(Pose2d robotPose, boolean isBlueAlliance) {
        Translation2d hub = isBlueAlliance ? blueAllianceHub : redAllianceHub;

        Translation2d translationToTarget = hub.minus(robotPose.getTranslation());

        // If we're already at the target (zero distance), keep the current heading
        if (translationToTarget.getNorm() < DrivetrainConstants.SNAP_TO_TARGET_DISTANCE_THRESHOLD) {
            return robotPose.getRotation();
        }
        
        return hub.minus(robotPose.getTranslation()).getAngle();
    }

    public static double distanceToHub(Pose2d robotPose, boolean isBlueAlliance) {
        Translation2d hub = isBlueAlliance ? blueAllianceHub : redAllianceHub;
        return hub.getDistance(robotPose.getTranslation());
    }
}
