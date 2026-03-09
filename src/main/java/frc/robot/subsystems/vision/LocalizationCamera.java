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

  private LocalVisionFilterPipeline m_filterPipeline = new LocalVisionFilterPipeline(); // filtering pipeline for each camera, initalizes as empty pipeline

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


    // Initialize Field2d widget for this camera (NOT THE SAME AS FUSEDPOSE)
    SmartDashboard.putData(m_logString + "/est pose field/" + m_cameraName, m_estPoseField);

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

  // Toggles "Raw Video Mode" for single camera
  // NOTE: PhotonVision method for raw video mode is setDriverMode
  // setDriverMode(true) = raw video feed, setDriverMode(false) = normal AprilTag processing
  public void setRawVideoMode(boolean rawVideoMode) {
    m_camera.setDriverMode(rawVideoMode);

    // log state of SINGLE CAMERA "RAW VIDEO MODE" to SmartDashboard
    SmartDashboard.putBoolean(m_logString + "/driverMode", rawVideoMode);
  }

  // --- filtering methods ---
  public void setFilterPipeline(LocalVisionFilterPipeline filterPipeline) {
    m_filterPipeline = filterPipeline;
  }

  // Updates the field simulation in elastic
  public void updateField(Pose2d newPos){
    m_estPoseField.setRobotPose(newPos);
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

    /*
     * NOTE: minor inefficiency by updating the toggles for each camera even though they share
     *       the same pipeline instance. BUT this ensures pipeline isn't leaked outside of
     *       this class.
     */
    if (m_filterPipeline.getNumFilters() > 0) {
      // Get most recent filter toggle states from SmartDashboard
      m_filterPipeline.updateSmartDashboardToggles();
    }

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1); // latest camera reading

      var poseEstimatorOutput = poseEstimator.update(result);
      
      // If present, create new CameraReading and run through pose ambiguity + local filter pipeline.
      if (poseEstimatorOutput.isPresent()) {

        // update std devs (will account for multi + single tag)
        var stdDevs = calculateEstimationStdDevs(poseEstimatorOutput.get(), result.getTargets());
        var newReading = new CameraReading(m_cameraName, poseEstimatorOutput.get(), stdDevs, result.getTimestampSeconds(), result.getTargets().size());

        // return empty if single tag has high pose ambiguity
        // NOTE: doesn't make sense as a LocalVisionFilter bc pose ambiguity is attached to target, not camera reading
        if (newReading.numTargets() == 1 && result.getBestTarget().getPoseAmbiguity() > VisionConstants.MAX_POSE_AMBIGUITY) {
          SmartDashboard.putString(m_logString + "/filtering/" + "poseAmbiguity", "sad");
          return Optional.empty();
        }

        // Run all filters on newReading, return empty reading if any fail
        if (!m_filterPipeline.runAll(newReading, this)) {
          return Optional.empty();
        }

        // Passed all filters --> update field simulation for single camera
        updateField(newReading.robotPose().estimatedPose.toPose2d());
        
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
}

