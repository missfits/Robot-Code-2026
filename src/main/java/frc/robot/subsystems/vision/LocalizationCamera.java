package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import java.util.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants.VisionConstants;

public class LocalizationCamera {

  private final PhotonCamera m_camera;
  private final String m_cameraName;
  private final String m_logString;

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private final Field2d m_estPoseField = new Field2d(); // field pose estimator
  private PhotonPoseEstimator poseEstimator;

  private LocalVisionFilterPipeline m_filterPipeline; // filtering pipeline for each camera

  private LinkedList<CameraReading> m_lastReadings = new LinkedList<>();

  private Optional<CameraReading> m_currentReading = Optional.empty();

  private final StructPublisher<Pose2d> pose2dPublisher;
  private final StructPublisher<Pose3d> pose3dPublisher;

  // every camera periodically creates a new CameraReading containing robot pose, std dev, timestamp, and number of targets seen.
  public static record CameraReading(String cameraName, EstimatedRobotPose robotPose, Matrix<N3, N1> stdDevs, double timestampSeconds, int numTargets) {}

  public LocalizationCamera(String cameraName, Transform3d robotToCam) {
    m_cameraName = cameraName;
    m_camera = new PhotonCamera(m_cameraName);
    m_logString = "vision/" + m_cameraName;
    poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    pose2dPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("SmartDashboard/" + m_logString + "/estimatedRobotPose2D", Pose2d.struct).publish();

    pose3dPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("SmartDashboard/" + m_logString + "/estimatedRobotPose3D", Pose3d.struct).publish();

    SmartDashboard.putBoolean("isConnected/" + m_cameraName, m_camera.isConnected());
  }

  public String getCameraName() {
    return m_cameraName;
  }

  public Optional<CameraReading> getCameraReading() {
    return m_currentReading;
  }

  public LinkedList<CameraReading> getLastCameraReadings() {
    return m_lastReadings;
  }

  public Field2d getEstField(){
    return m_estPoseField;
  }

  // --- filtering methods ---
  public void setFilterPipeline(LocalVisionFilterPipeline filterPipeline) {
    m_filterPipeline = filterPipeline;
  }

  // Updates the field simulation in elastic
  public void updateField(Pose2d newPos){
    m_estPoseField.setRobotPose(newPos);

    SmartDashboard.putData(m_logString + "/est pose field/", m_estPoseField);
  }

  
  public void updateCameraReading() {
    Optional<CameraReading> newReading = calculateNewCameraReading();

    if (newReading.isPresent()) {
      m_currentReading = newReading;
      m_lastReadings.add(m_currentReading.get());

      if (m_lastReadings.size() > VisionConstants.NUM_LAST_EST_POSES) {
        m_lastReadings.removeFirst();
      }

      // logging all cameraReading data to SmartDashboard
      SmartDashboard.putBoolean(m_logString + "/reading-is-Present", true);

      SmartDashboard.putNumber(m_logString + "/reading-num-targets-seen", newReading.get().numTargets());
      SmartDashboard.putNumberArray(m_logString + "/reading-standard-devs", newReading.get().stdDevs().getData());
      SmartDashboard.putNumber(m_logString + "/reading-timestamp", newReading.get().timestampSeconds());
      SmartDashboard.putString(m_logString + "/reading-is-multiTag", newReading.get().numTargets() > 1 ? "multitagReading" : "singleTagReading");

    } else {
      SmartDashboard.putBoolean(m_logString + "/reading-is-Present", false);
    }
    // no matter what, want to publish isConnected to NetworkTables
    SmartDashboard.putBoolean("isConnected/" + m_cameraName, m_camera.isConnected());

    // publish estimated robot Pose2d to NetworkTables
    // uses map for efficient unwrapping of Optional<CameraReading>
    pose2dPublisher.set(newReading.map(reading -> reading.robotPose().estimatedPose.toPose2d()).orElse(null));
    pose3dPublisher.set(newReading.map(reading -> reading.robotPose().estimatedPose).orElse(null));
  }

  private Optional<CameraReading> calculateNewCameraReading() {
    var results = m_camera.getAllUnreadResults(); // raw camera data

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1); // latest camera reading

      var poseEstimatorOutput = poseEstimator.update(result);
      
      // if present + not jumpy when compared to last 3 camera readings,
      // update instance var m_currentReading and add to m_lastReadings
      if (poseEstimatorOutput.isPresent()) {
        // update std devs (will account for multi + single tag)
        var stdDevs = calculateEstimationStdDevs(poseEstimatorOutput.get(), result.getTargets());
        var newReading = new CameraReading(m_cameraName, poseEstimatorOutput.get(), stdDevs, result.getTimestampSeconds(), result.getTargets().size());

        // return empty if single tag has high pose ambiguity
        if (newReading.numTargets() == 1 && result.getBestTarget().getPoseAmbiguity() > VisionConstants.MAX_POSE_AMBIGUITY) {
          SmartDashboard.putString(m_logString + "/filtering/" + "poseAmbiguity", "sad");
          return Optional.empty();
        }
        
        SmartDashboard.putString(m_logString + "/filtering/" + "poseAmbiguity", "happy");
        return Optional.of(newReading);
      }
    }
    return Optional.empty();
  }

  // Standard deviation measures how "spread out" / accurate a vision reading is
  private Matrix<N3, N1> calculateEstimationStdDevs(EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {
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
          .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
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

  /*
   * returns true if the average speed between the last 3 camera readings (FROM ONE CAMREA)
   * is less than the max average speed.
   * goal is to check if the last three readings are smooth + consistent. 
   */
  public boolean areRecentCameraPosesConsistent() {
    if (m_lastReadings.size() < VisionConstants.NUM_LAST_EST_POSES) {
      SmartDashboard.putString(m_logString + "/filtering/" + "areRecentCameraPosesConsistent", "skipped");
      return false;
    }

    double totalDistance = 0;
    double totalTime = 0;

    for (int i = 0; i < m_lastReadings.size() - 1; i++) {
      // add distance between ith pose and i+1th pose
      Pose2d pose1 = m_lastReadings.get(i).robotPose.estimatedPose.toPose2d();
      Pose2d pose2 = m_lastReadings.get(i + 1).robotPose.estimatedPose.toPose2d();
      
      totalDistance += Math.abs(pose1.minus(pose2).getTranslation().getNorm());
      totalTime += Math.abs(m_lastReadings.get(i).timestampSeconds - m_lastReadings.get(i+1).timestampSeconds);
    }

    // divide by number of intervals (n-1)
    double avgDist = totalDistance / (m_lastReadings.size() - 1);
    double avgTime = totalTime / (m_lastReadings.size() - 1);
    if (avgTime == 0){
      SmartDashboard.putString(m_logString + "/filtering/" + "areRecentCameraPosesConsistent", "discard");
      return false;
    }
    double avgSpeed = avgDist/avgTime;

    SmartDashboard.putNumber("vision/" + m_cameraName + "/avgDistBetweenLastEstPoses", avgDist);
    SmartDashboard.putNumber("vision/" + m_cameraName + "/avgSpeedBetweenLastEstPoses", avgSpeed);
    SmartDashboard.putNumber("vision/" + m_cameraName + "/avgTimeBetweenLastEstPoses", avgTime);

    SmartDashboard.putString(m_logString + "/filtering/" + "areRecentCameraPosesConsistent", avgSpeed < VisionConstants.MAX_AVG_SPEED_BETWEEN_LAST_EST_POSES ? "good" : "bad");

    return avgSpeed < VisionConstants.MAX_AVG_SPEED_BETWEEN_LAST_EST_POSES;
  }
}

