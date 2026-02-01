package frc.robot.subsystems.vision.filtering;

import java.util.LinkedList;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LocalVisionFilter;
import frc.robot.subsystems.vision.LocalizationCamera;
import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;

public class LocalCameraPoseConsistencyDistanceToFusedPoseFilter implements LocalVisionFilter {
  
  private CommandSwerveDrivetrain drivetrain;
  
  public LocalCameraPoseConsistencyDistanceToFusedPoseFilter(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  /*
   * GOAL: check if the last three readings are smooth + consistent OR if the distance to the fused pose is valid.
   * 
   * NOTE: Must be OR, not AND, to help mitigate slow correction after bumping into walls
   */
  @Override
  public boolean isValid(CameraReading reading, LocalizationCamera cam) {
    return isLocalCameraPoseConsistent(reading, cam) || isDistanceToFusedPoseValid(reading, cam);
  }
  
  public boolean isDistanceToFusedPoseValid(CameraReading reading, LocalizationCamera cam) {
    // Get the estimated pose from the camera reading
    EstimatedRobotPose robotPose = reading.robotPose();
    Pose2d estPose2d = robotPose.estimatedPose.toPose2d();

    // check if new estimated pose and previous pose are less than 2 meters apart (fused poseEst)
    double distance = estPose2d.getTranslation().getDistance(drivetrain.getState().Pose.getTranslation());

    return distance < VisionConstants.MAX_VISION_POSE_DISTANCE;
  }


  /*
   * Filter to check if the last 3 camera readings from a given camera
   *   are consistent with each other.
   * NOTE: most recent reading is included in last 3 readings.
   * 
   * GOAL: check if the last three readings are smooth + consistent.
   */
  public boolean isLocalCameraPoseConsistent(CameraReading reading, LocalizationCamera cam) {

    LinkedList<CameraReading> lastReadings = cam.getLastCameraReadings();

    // If we don't have enough readings, return true to skip this filter.
    //  NOTE: returning false will cause the entire filter pipeline to stop.
    if (lastReadings.size() < VisionConstants.NUM_LAST_EST_POSES) {
      return true;
    }

    double totalDistance = 0;
    double totalTime = 0;

    // Calculate the average speed by computing (avg distance) / (avg time)
    for (int i = 0; i < lastReadings.size() - 1; i++) {
      // add distance between ith pose and i+1th pose
      Pose2d pose1 = lastReadings.get(i).robotPose().estimatedPose.toPose2d();
      Pose2d pose2 = lastReadings.get(i + 1).robotPose().estimatedPose.toPose2d();
      
      totalDistance += Math.abs(pose1.minus(pose2).getTranslation().getNorm());
      totalTime += Math.abs(lastReadings.get(i).timestampSeconds() - lastReadings.get(i+1).timestampSeconds());
    }

    // divide by number of intervals (n-1)
    double avgDist = totalDistance / (lastReadings.size() - 1);
    double avgTime = totalTime / (lastReadings.size() - 1);
    if (avgTime == 0){
      return false;
    }

    double avgSpeed = avgDist / avgTime;

    return avgSpeed < VisionConstants.MAX_AVG_SPEED_BETWEEN_LAST_EST_POSES;
  }
}
