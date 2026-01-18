package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import java.util.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants.VisionConstants;

public class LocalizationCamera {

  private final PhotonCamera m_camera;
  private final String m_cameraName;

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private final Field2d m_estPoseField = new Field2d(); // field pose estimator
  private PhotonPoseEstimator poseEstimator;

  private LinkedList<CameraReading> m_lastReadings = new LinkedList<>();

  private Optional<CameraReading> m_currentReading = Optional.empty();

  public static record CameraReading(Optional<EstimatedRobotPose> robotPose, Matrix<N3, N1> stdDevs, double timestampSeconds, Integer numTargets) {}

  public LocalizationCamera(String cameraName, Transform3d robotToCam) {
    m_cameraName = cameraName;
    m_camera = new PhotonCamera(m_cameraName);
    poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    SmartDashboard.putBoolean("isConnected/" + m_cameraName, m_camera.isConnected());
  }

  public String getCameraName() {
    return m_cameraName;
  }

  public Optional<CameraReading> getCameraReading() {
    return m_currentReading;
  }

  public Field2d getEstField(){
    return m_estPoseField;
  }

  // Updates the field simulation in elastic
  public void updateField(Pose2d newPos){
    m_estPoseField.setRobotPose(newPos);

    SmartDashboard.putData("est pose field/" + m_cameraName + "/", m_estPoseField);
  }

  
  public void updateCameraReading() {
    var results = m_camera.getAllUnreadResults(); // raw camera data

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1); // latest camera reading

      var poseEstimatorOutput = poseEstimator.update(result);
      
      // if present + not jumpy when compared to last 3 camera readings,
      // update instance var m_currentReading and add to m_lastReadings
      if (poseEstimatorOutput.isPresent()) {
        // update std devs (will account for multi + single tag)
        m_currentReading = Optional.of(new CameraReading(poseEstimatorOutput, updateEstimationStdDevs(poseEstimatorOutput, result.getTargets()), result.getTimestampSeconds(), result.getTargets().size()));

        // return empty if single tag has high pose ambiguity
        if (m_currentReading.get().numTargets() == 1 && result.getBestTarget().getPoseAmbiguity() > VisionConstants.MAX_POSE_AMBIGUITY) {
          m_currentReading = Optional.empty();
          return;
        }

        m_lastReadings.add(m_currentReading.get());

        if (m_lastReadings.size() > VisionConstants.NUM_LAST_EST_POSES) {
          m_lastReadings.removeFirst();
        }

      } else {
        // if newest reading empty or jumpy, set m_currentReading to be empty
        m_currentReading = Optional.empty(); 
      }

      SmartDashboard.putString("vision/" + m_cameraName + "/targetState", result.hasTargets() ? "targetsFound" : "noTargets");
      SmartDashboard.putString("vision/" + m_cameraName + "/is-multiTag", result.getTargets().size() > 1 ? "multitagReading" : "singleTagReading");
    }
  }

  // LOGIC UNCHANGED
  // Standard deviation measures how "spread out" / accurate a vision reading is
  private Matrix<N3, N1> updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      SmartDashboard.putNumber("vision/" + m_cameraName + "/standardDeviation-average-distance", Double.MAX_VALUE);
      SmartDashboard.putString("vision/" + m_cameraName + "/standardDeviation-state", "empty");
      return VisionConstants.kSingleTagStdDevs;
    } else {
      // Pose present. Start running Heuristic
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        SmartDashboard.putString("vision/" + m_cameraName + "/standardDeviation-state", "no tags visible");
        return VisionConstants.kSingleTagStdDevs;
      } else if (numTags == 1 && avgDist > VisionConstants.VISION_DISTANCE_DISCARD) {
        SmartDashboard.putString("vision/" + m_cameraName + "/standardDeviation-state", "target too far");
        return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      } else {
        var unscaledStdDevs = numTags > 1 ? VisionConstants.kMultiTagStdDevs : VisionConstants.kSingleTagStdDevs;
        avgDist /= numTags;
        // increase std devs based on (average) distance
        SmartDashboard.putString("vision/" + m_cameraName + "/standardDeviation-state", "good :)");
        return unscaledStdDevs.times(1 + (avgDist * avgDist / VisionConstants.STD_DEV_SCALER));
      }
    }
  }

  // LOGIC UNCHANGED
  // checks if the pose is jumpy based on avg speed since by calculating based on speed, 
  // the camera fps doesn't matter as the speed between readings will still be the same. this is based on last 3 readings
  public boolean isEstPoseJumpy() {
    if (m_lastReadings.size() < VisionConstants.NUM_LAST_EST_POSES) {
      return true;
    }

    double totalDistance = 0;
    double totalTime = 0;

    for (int i = 0; i < m_lastReadings.size() - 1; i++) {
      // add distance between ith pose and i+1th pose
      totalDistance += Math.abs(m_lastReadings.get(i).robotPose.get().estimatedPose.toPose2d().minus(m_lastReadings.get(i + 1).robotPose.get().estimatedPose.toPose2d()).getTranslation().getNorm());
      totalTime += Math.abs(m_lastReadings.get(i).timestampSeconds - m_lastReadings.get(i+1).timestampSeconds);
    }

    // divide by number of intervals (n-1)
    double avgDist = totalDistance / (m_lastReadings.size() - 1);
    double avgTime = totalTime / (m_lastReadings.size() - 1);
    if (avgTime == 0){
      return true;
    }
    double avgSpeed = avgDist/avgTime;

    SmartDashboard.putNumber("vision/" + m_cameraName + "/avgDistBetweenLastEstPoses", avgDist);
    SmartDashboard.putNumber("vision/" + m_cameraName + "/avgSpeedBetweenLastEstPoses", avgSpeed);
    SmartDashboard.putNumber("vision/" + m_cameraName + "/avgTimeBetweenLastEstPoses", avgTime);

    return avgSpeed > VisionConstants.MAX_AVG_SPEED_BETWEEN_LAST_EST_POSES;
  }
}

