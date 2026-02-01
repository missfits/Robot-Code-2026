package frc.robot.subsystems.vision.filtering;

import java.util.Iterator;
import java.util.LinkedList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.LocalVisionFilter;
import frc.robot.subsystems.vision.LocalizationCamera;
import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;

public class LocalCameraPoseConsistencyFilter implements LocalVisionFilter{
    
  /*
   * Filter to check if the last 3 camera readings from a given camera
   *   are consistent with each other.
   * NOTE: most recent reading is NOT included in the last 3 readings. We
   *   want to check most recent reading AGAINST last 3 readings.
   * 
   * GOAL: check if the last three readings are smooth + consistent.
   */
  public LocalCameraPoseConsistencyFilter() {}

  @Override  
  public boolean isValid(CameraReading reading, LocalizationCamera cam) {

    LinkedList<CameraReading> lastReadings = cam.getLastCameraReadings();

    // If we don't have enough readings, return true to skip this filter.
    //  NOTE: returning false will cause the entire filter pipeline to stop.
    if (lastReadings.size() < VisionConstants.NUM_LAST_EST_POSES) {
      return true;
    }

    double totalDistance = 0;
    double totalTime = 0;

    // Calculate the average speed by computing (avg distance) / (avg time)
    // Uses an iterator for optimized LinkedList runtime (O(n) instead of O(n²))
    Iterator<CameraReading> it = lastReadings.iterator();
    CameraReading prev = it.next();

    while (it.hasNext()) {
      CameraReading curr = it.next();
      Pose2d pose1 = prev.robotPose().estimatedPose.toPose2d();
      Pose2d pose2 = curr.robotPose().estimatedPose.toPose2d();
      
      totalDistance += Math.abs(pose1.minus(pose2).getTranslation().getNorm());
      totalTime += Math.abs(prev.timestampSeconds() - curr.timestampSeconds());
      prev = curr;
    }

    // Add comparison: last history reading → new reading; last reading =  prev from iterator
    // (recall lastReadings does NOT include most recent reading)
    Pose2d lastPose = prev.robotPose().estimatedPose.toPose2d();
    Pose2d newPose = reading.robotPose().estimatedPose.toPose2d();
    
    totalDistance += Math.abs(lastPose.minus(newPose).getTranslation().getNorm());
    totalTime += Math.abs(prev.timestampSeconds() - reading.timestampSeconds());

    // Division by zero check!
    if (totalTime == 0){
      return false;
    }

    double avgSpeed = totalDistance / totalTime;

    return avgSpeed < VisionConstants.MAX_AVG_SPEED_BETWEEN_LAST_EST_POSES;
  }
}
