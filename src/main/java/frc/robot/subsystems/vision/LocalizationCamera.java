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

  private boolean targetFound; // true if the translation2d was updated last periodic() call
  private EstimatedRobotPose estimatedRobotPose;
  private boolean isNewResult = false;

  private Matrix<N3, N1> curStdDevs = VisionConstants.kSingleTagStdDevs;

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private final Field2d m_estPoseField = new Field2d(); // field pose estimator
  private PhotonPoseEstimator poseEstimator;

  private PhotonTrackedTarget m_apriltagTarget;
  private LinkedList<EstimatedRobotPose> m_lastEstPoses = new LinkedList<>();

  public LocalizationCamera(String cameraName, Transform3d robotToCam) {
    m_cameraName = cameraName;
    m_camera = new PhotonCamera(m_cameraName);
    poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    SmartDashboard.putBoolean("isConnected/" + m_cameraName, m_camera.isConnected());
  }

  public String getCameraName() {
    return m_cameraName;
  }

  public PhotonTrackedTarget getTarget() {
    return m_apriltagTarget;
  }

  public Double getTargetPoseAmbiguity() {
    return m_apriltagTarget.getPoseAmbiguity();
  }

  public boolean getTargetFound() {
    return targetFound;
  }

  public EstimatedRobotPose getRobotPose(){
    return estimatedRobotPose;
  }

  public Field2d getEstField(){
    return m_estPoseField;
  }

  public boolean getIsNewResult(){
    return isNewResult;
  }

  public Matrix<N3, N1> getCurrentStdDevs(){
    return curStdDevs;
  }

  // Updates the field simulation in elastic
  public void updateField(Pose2d newPos){
    m_estPoseField.setRobotPose(newPos);

    SmartDashboard.putData("est pose field/" + m_cameraName + "/", m_estPoseField);
  }

  
  // processes all targets
  // uses area, standard deviation checks (outlier results?) and "sane-ness" (are readings > pre-determined maximum constants?) 
  public void findTarget() {

      ArrayList<Integer> apriltagIDs = new ArrayList<>();
      ArrayList<Double> targetPoseAmbiguity = new ArrayList<>();
      List<PhotonPipelineResult> results;

      results = m_camera.getAllUnreadResults();

        if (!results.isEmpty()){
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            isNewResult = true;

            if (result.hasTargets()) {
                double biggestTargetArea = 0;

                for (PhotonTrackedTarget sampleTarget : result.getTargets()){
                    apriltagIDs.add(sampleTarget.getFiducialId());
                    targetPoseAmbiguity.add(sampleTarget.getPoseAmbiguity());
                    //loops through every sample target in results.getTargets()
                    //if the sample target's area is bigger than the biggestTargetArea, then the sample target
                    // is set to the target, and the biggest Target Area is set to the sample's target area
                    // we want the april tag with the biggest area since that means it is the closest
                    if (sampleTarget.getArea() > biggestTargetArea){
                        biggestTargetArea = sampleTarget.getArea();
                        m_apriltagTarget = sampleTarget;
                    }
                }

                SmartDashboard.putNumber("vision/" + m_cameraName + "Alternate Target X", m_apriltagTarget.getBestCameraToTarget().getX());
                SmartDashboard.putNumber("vision/" + m_cameraName + "Alternate Target Y", m_apriltagTarget.getBestCameraToTarget().getY());

                if (m_apriltagTarget.getPoseAmbiguity() > VisionConstants.MAX_POSE_AMBIGUITY) {
                    SmartDashboard.putString("vision/" + m_cameraName + "/targetState", "targetDiscardedAmbiguity");
                    targetFound = false;
                  } else {
                    SmartDashboard.putString("vision/" + m_cameraName + "/targetState", "targetFound");
                    targetFound = true;
                  }
                  
            } else {
                SmartDashboard.putString("vision/" + m_cameraName + "/targetState", "noTarget");
                targetFound = false;
            }

            if (targetFound) {
              // update the estimated robot pose if the pose estimator outputs something
              Optional<EstimatedRobotPose> poseEstimatorOutput = poseEstimator.update(result);
      
              // update standard deviation based on dist 
              this.updateEstimationStdDevs(poseEstimatorOutput, result.getTargets());
                            
              if (poseEstimatorOutput.isPresent() && VisionUtils.poseIsSane(poseEstimatorOutput.get().estimatedPose)) {
                // Temporarily add new pose to history to check if it makes the sequence jumpy
                m_lastEstPoses.add(poseEstimatorOutput.get());
                
                if (m_lastEstPoses.size() > VisionConstants.NUM_LAST_EST_POSES) {
                  m_lastEstPoses.removeFirst();
                }

                // Check if the pose estimate is jumpy (indicates bad data or camera shift)
                if (isEstPoseJumpy()) {
                  // reject pose by removing from history m_lastEstPoses
                  m_lastEstPoses.removeLast();
                  estimatedRobotPose = null; // resets estimatedRobotPose to null if jumpy; stops processing in updatePoseEst
                  SmartDashboard.putString("vision/" + m_cameraName + "/targetState", "targetDiscardedJumpy");
                } else {
                  // accept pose by changing instance var estimatedRobotPose
                  estimatedRobotPose = poseEstimatorOutput.get();
                  SmartDashboard.putString("vision/" + m_cameraName + "/targetState", "targetFound");
                }

                SmartDashboard.putBoolean("vision/" + m_cameraName + "/zIsSane", VisionUtils.zIsSane(poseEstimatorOutput.get().estimatedPose));
                SmartDashboard.putBoolean("vision/" + m_cameraName + "/rollIsSane", VisionUtils.rollIsSane(poseEstimatorOutput.get().estimatedPose));
                SmartDashboard.putBoolean("vision/" + m_cameraName + "/pitchIsSane", VisionUtils.pitchIsSane(poseEstimatorOutput.get().estimatedPose));
              }
            }
            else{
              isNewResult = false;
            }
        }
        SmartDashboard.putBoolean("vision/" + m_cameraName + "/isConnected", m_camera.isConnected());
        SmartDashboard.putBoolean("vision/" + m_cameraName + "/isNewResult", getIsNewResult());

        SmartDashboard.putNumberArray("vision/" + m_cameraName + "/Targets Seen", apriltagIDs.stream().mapToDouble(Integer::doubleValue).toArray());
        SmartDashboard.putNumberArray("vision/" + m_cameraName + "/Target Pose Ambiguities", targetPoseAmbiguity.stream().mapToDouble(Double::doubleValue).toArray());
        SmartDashboard.putBoolean("vision/" + m_cameraName + "/Target Found", targetFound);

        SmartDashboard.putBoolean("vision/" + m_cameraName + "/isEstPoseJumpy", isEstPoseJumpy());
        SmartDashboard.putNumberArray("vision/" + m_cameraName + "/standardDeviations", curStdDevs.getData());

        SmartDashboard.putBoolean("isConnected/" + m_cameraName, m_camera.isConnected());
    }

  // Standard deviation measures how "spread out" / accurate a vision reading is
  private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = VisionConstants.kSingleTagStdDevs;
      SmartDashboard.putNumber("vision/" + m_cameraName + "/standardDeviation-average-distance", Double.MAX_VALUE);
      SmartDashboard.putString("vision/" + m_cameraName + "/standardDeviation-state", "empty");
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
        curStdDevs = VisionConstants.kSingleTagStdDevs;
        SmartDashboard.putString("vision/" + m_cameraName + "/standardDeviation-state", "no tags visible");
      } else if (numTags == 1 && avgDist > VisionConstants.VISION_DISTANCE_DISCARD) {
        curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        SmartDashboard.putString("vision/" + m_cameraName + "/standardDeviation-state", "target too far");
      } else {
        var unscaledStdDevs = numTags > 1 ? VisionConstants.kMultiTagStdDevs : VisionConstants.kSingleTagStdDevs;

        avgDist /= numTags;
        // increase std devs based on (average) distance
        curStdDevs = unscaledStdDevs.times(1 + (avgDist * avgDist / VisionConstants.STD_DEV_SCALER));
        SmartDashboard.putString("vision/" + m_cameraName + "/standardDeviation-state", "good :)");
      }
      SmartDashboard.putNumber("vision/" + m_cameraName + "/standardDeviation-average-distance", avgDist);
    }
  }

  // checks if the pose is jumpy based on avg speed since by calculating based on speed, 
  // the camera fps doesn't matter as the speed between readings will still be the same. this is based on last 3 readings
  public boolean isEstPoseJumpy() {
    if (m_lastEstPoses.size() < VisionConstants.NUM_LAST_EST_POSES) {
      return true;
    }

    double totalDistance = 0;
    double totalTime = 0;

    for (int i = 0; i < m_lastEstPoses.size() - 1; i++) {
      // add distance between ith pose and i+1th pose
      totalDistance += Math.abs(m_lastEstPoses.get(i).estimatedPose.toPose2d().minus(m_lastEstPoses.get(i + 1).estimatedPose.toPose2d()).getTranslation().getNorm());
      totalTime += Math.abs(m_lastEstPoses.get(i).timestampSeconds - m_lastEstPoses.get(i+1).timestampSeconds);
    }

    // divide by number of intervals (n-1)
    double avgDist = totalDistance / (m_lastEstPoses.size() - 1);
    double avgTime = totalTime / (m_lastEstPoses.size() - 1);
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

