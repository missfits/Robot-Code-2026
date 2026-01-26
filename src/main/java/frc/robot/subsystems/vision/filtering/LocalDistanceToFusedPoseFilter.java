package frc.robot.subsystems.vision.filtering;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LocalVisionFilter;
import frc.robot.subsystems.vision.LocalizationCamera;
import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;

public class LocalDistanceToFusedPoseFilter implements LocalVisionFilter{
    
  private final CommandSwerveDrivetrain drivetrain;

  /*
   * Filter to check if the distance between the estimaed pose of a given camera
   *    and the current robot pose (fused drivetrain pose) is less than max distance constant.
   */
  public LocalDistanceToFusedPoseFilter(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain; 
  }

  @Override
  // NOTE: Variable cam does not get used in this filter.
  public boolean isValid(CameraReading reading, LocalizationCamera cam) {
    // Get the estimated pose from the camera reading
    EstimatedRobotPose robotPose = reading.robotPose();
    Pose2d estPose2d = robotPose.estimatedPose.toPose2d();

    // check if new estimated pose and previous pose are less than 2 meters apart (fused poseEst)
    double distance = estPose2d.getTranslation().getDistance(drivetrain.getState().Pose.getTranslation());

    if (distance > VisionConstants.MAX_VISION_POSE_DISTANCE) {
      return false;
    }
    return true;
  }
}
