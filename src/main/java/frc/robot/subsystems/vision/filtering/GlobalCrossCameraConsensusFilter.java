package frc.robot.subsystems.vision.filtering;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.GlobalVisionFilter;
import frc.robot.subsystems.vision.LocalizationCamera;
import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;

public class GlobalCrossCameraConsensusFilter implements GlobalVisionFilter{
  
  /*
   * Checks if the given camera reading is within a certain distance
   *  of the average of all camera readings.
   * 
   * GOAL: Find outlier readings.
   */
  public GlobalCrossCameraConsensusFilter() {}

  @Override
  public boolean isValid(CameraReading reading, List<CameraReading> allReadings) {
    // If we don't have enough readings, return true to skip this filter.
    //  NOTE: returning false will cause the entire filter pipeline to stop.
    if (allReadings.size() < VisionConstants.MIN_NUM_CAMERA_READINGS) {
      return true;
    }

    // Calculate the average estimated Pose2d of across all readings w/ (sum || sumY) / numposes
    double sumX = 0, sumY = 0;
    int numPoses = 0;
    for (CameraReading read : allReadings) {
      Pose2d pose = read.robotPose().estimatedPose.toPose2d();
      sumX += pose.getX();
      sumY += pose.getY();
      numPoses++;
    }

    // NOTE: Don't need to check for division by zero because MIN_NUM_CAMERA_READINGS > 0.
    Pose2d avgPose = new Pose2d(sumX / numPoses, sumY / numPoses, new Rotation2d());
    Pose2d readingPose = reading.robotPose().estimatedPose.toPose2d();

    return readingPose.getTranslation().getDistance(avgPose.getTranslation()) < VisionConstants.MAX_VISION_READING_DISTANCE;
  }    
}
