package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import java.util.*;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import frc.robot.Constants.VisionConstants;

public class LocalizationCamera {

  private final PhotonCamera m_camera;
  private final String m_cameraName;
  private final String m_logString;

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private final Field2d m_estPoseField = new Field2d(); // field pose estimator
  private PhotonPoseEstimator poseEstimator;

  private final Supplier<SwerveDriveState> m_robotSwerveStateSupplier;

  // used for acceleration-based scaling (for front cameras)
  private ChassisSpeeds m_previousSpeed = new ChassisSpeeds(); // intializes to empty
  private double m_previousTimestamp = 0.0;

  private LocalVisionFilterPipeline m_filterPipeline = new LocalVisionFilterPipeline(); // filtering pipeline for each camera, initalizes as empty pipeline

  private LinkedList<CameraReading> m_lastReadings = new LinkedList<>();

  private Optional<CameraReading> m_currentReading = Optional.empty();

  private final StructPublisher<Pose2d> pose2dPublisher;
  private final StructPublisher<Pose3d> pose3dPublisher;

  private final boolean m_isFrontCamera; // boolean: if camera is in front or back 

  // every camera periodically creates a new CameraReading containing robot pose, std dev, timestamp, and number of targets seen.
  public static record CameraReading(String cameraName, EstimatedRobotPose robotPose, Matrix<N3, N1> stdDevs, double timestampSeconds, int numTargets) {}

  public LocalizationCamera(String cameraName, Transform3d robotToCam, Supplier<SwerveDriveState> robotSwerveStateSupplier) {
    m_cameraName = cameraName;
    m_camera = new PhotonCamera(m_cameraName);
    m_logString = "vision/" + m_cameraName;
    m_robotSwerveStateSupplier = robotSwerveStateSupplier;

    poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    pose2dPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("SmartDashboard/" + m_logString + "/estimatedRobotPose2d", Pose2d.struct).publish();

    pose3dPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("SmartDashboard/" + m_logString + "/estimatedRobotPose3d", Pose3d.struct).publish();


    if (m_cameraName.indexOf("front") >= 0) {
      m_isFrontCamera = true;
    }
    else {
      m_isFrontCamera = false;
    }

    // Initialize Field2d widget for this camera (NOT THE SAME AS FUSEDPOSE)
    SmartDashboard.putData(m_logString + "/estimatedPoseField", m_estPoseField);

    SmartDashboard.putBoolean(m_logString + "/isConnected", m_camera.isConnected());
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
    SmartDashboard.putBoolean(m_logString + "/rawVideoModeEnabled", rawVideoMode);
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
      SmartDashboard.putBoolean(m_logString + "/hasReading", true);

      SmartDashboard.putNumber(m_logString + "/numTargetsSeen", newReading.get().numTargets());
      SmartDashboard.putNumberArray(m_logString + "/standardDeviations", newReading.get().stdDevs().getData());
      SmartDashboard.putNumber(m_logString + "/timestampSeconds", newReading.get().timestampSeconds());
      SmartDashboard.putString(m_logString + "/readingType", newReading.get().numTargets() > 1 ? "multitagReading" : "singleTagReading");
      SmartDashboard.putNumber(m_logString + "/estimatedHeadingDegrees",
          newReading.get().robotPose().estimatedPose.toPose2d().getRotation().getDegrees());
      SmartDashboard.putNumber(m_logString + "/estimatedHeadingRadians",
          newReading.get().robotPose().estimatedPose.toPose2d().getRotation().getRadians());

    } else {
      SmartDashboard.putBoolean(m_logString + "/hasReading", false);
    }
    // no matter what, want to publish isConnected to NetworkTables
    SmartDashboard.putBoolean(m_logString + "/isConnected", m_camera.isConnected());

    // publish estimated robot Pose2d to NetworkTables
    // uses map for efficient unwrapping of Optional<CameraReading>
    pose2dPublisher.set(newReading.map(reading -> reading.robotPose().estimatedPose.toPose2d()).orElse(null));
    pose3dPublisher.set(newReading.map(reading -> reading.robotPose().estimatedPose).orElse(null));
  }

  private PhotonPoseEstimator copyPoseEstimator() {
    PhotonPoseEstimator poseEstimatorCopy = new PhotonPoseEstimator(
        poseEstimator.getFieldTags(),
        poseEstimator.getPrimaryStrategy(),
        poseEstimator.getRobotToCameraTransform());

    poseEstimatorCopy.setTagModel(poseEstimator.getTagModel());

    Pose3d referencePose = poseEstimator.getReferencePose();
    if (referencePose != null) {
      poseEstimatorCopy.setReferencePose(referencePose);
    }

    m_currentReading.ifPresent(reading -> poseEstimatorCopy.setLastPose(reading.robotPose().estimatedPose));

    return poseEstimatorCopy;
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

      // updating + filtering runs on a copy pose estimator so filtered don't pollute the estimator 
      PhotonPoseEstimator candidatePoseEstimator = copyPoseEstimator();
      var poseEstimatorOutput = candidatePoseEstimator.update(result);
      
      // If present, create new CameraReading and run through pose ambiguity + local filter pipeline.
      if (poseEstimatorOutput.isPresent()) {
        EstimatedRobotPose estimatedPose = poseEstimatorOutput.get();
        String poseAmbiguityStatus = "happy";

        // NOTE: doesn't make sense as a LocalVisionFilter bc pose ambiguity is attached to target, not camera reading
        if (result.getTargets().size() == 1 && result.getBestTarget().getPoseAmbiguity() > VisionConstants.MAX_POSE_AMBIGUITY) {
          // When PhotonVision reports high ambiguity, compare both candidates against
          // the robot's current heading and keep the one that is more consistent.
          var resolvedPose = resolveHighAmbiguityPose(candidatePoseEstimator, result.getBestTarget(), estimatedPose);
          if (resolvedPose.isEmpty()) {
            SmartDashboard.putString(m_logString + "/filtering/poseAmbiguityStatus", "rejectedHighAmbiguity"); // alternate pose DNE
            return Optional.empty();
          }

          estimatedPose = resolvedPose.get();
          poseAmbiguityStatus = "resolvedWithDrivetrainHeading";
        }

        // update std devs (will account for multi + single tag)
        var stdDevs = calculateEstimationStdDevs(estimatedPose, result.getTargets());
        var newReading = new CameraReading(m_cameraName, estimatedPose, stdDevs, result.getTimestampSeconds(), result.getTargets().size());

        // Run all filters on newReading, return empty reading if any fail
        if (!m_filterPipeline.runAll(newReading, this)) {
          return Optional.empty();
        }

        // Passed all filters --> update field simulation for single camera + update poseEstimator
        updateField(newReading.robotPose().estimatedPose.toPose2d());
        poseEstimator = candidatePoseEstimator;

        SmartDashboard.putString(m_logString + "/filtering/poseAmbiguityStatus", poseAmbiguityStatus);
        return Optional.of(newReading);
      }
    }
    return Optional.empty();
  }

  // resolves high pose ambiguity state for single tag pose by choosing the pose that is closest to the current robot heading
  private Optional<EstimatedRobotPose> resolveHighAmbiguityPose(
      PhotonPoseEstimator candidatePoseEstimator,
      PhotonTrackedTarget target,
      EstimatedRobotPose estimatedPose) {
    // Reconstruct both robot-pose candidates from the tag's best/alternate camera-to-tag transforms.
    var bestPose = estimateSingleTagRobotPose(candidatePoseEstimator, target, target.getBestCameraToTarget());
    var alternatePose = estimateSingleTagRobotPose(candidatePoseEstimator, target, target.getAlternateCameraToTarget());

    if (bestPose.isEmpty() || alternatePose.isEmpty()) {
      return Optional.empty();
    }

    // Get current robot heading from drivetrain
    SwerveDriveState robotState = m_robotSwerveStateSupplier.get();
    if (robotState == null || robotState.Pose == null) { // NOTE: can't resolve heading if heading is null
      return Optional.empty();
    }

    Rotation2d currentHeading = robotState.Pose.getRotation();
    Pose3d chosenPose = bestPose.get();
    // Prefer the candidate whose field-relative heading is closer to the drivetrain heading.
    if (headingDistance(alternatePose.get().toPose2d().getRotation(), currentHeading)
        < headingDistance(bestPose.get().toPose2d().getRotation(), currentHeading)) {
      chosenPose = alternatePose.get();
    }

    // Keep the copied estimator's internal history aligned with the pose we actually chose.
    candidatePoseEstimator.setLastPose(chosenPose);

    return Optional.of(new EstimatedRobotPose(
        chosenPose,
        estimatedPose.timestampSeconds,
        estimatedPose.targetsUsed,
        estimatedPose.strategy));
  }

  private Optional<Pose3d> estimateSingleTagRobotPose(
      PhotonPoseEstimator estimator,
      PhotonTrackedTarget target,
      Transform3d cameraToTarget) {
    var tagPose = estimator.getFieldTags().getTagPose(target.getFiducialId());
    if (tagPose.isEmpty()) {
      return Optional.empty();
    }

    // tagPose -> cameraPose -> robotPose
    return Optional.of(tagPose.get()
        .transformBy(cameraToTarget.inverse())
        .transformBy(estimator.getRobotToCameraTransform().inverse()));
  }

  private double headingDistance(Rotation2d headingA, Rotation2d headingB) {
    return Math.abs(headingA.minus(headingB).getRadians());
  }

  // Standard deviation measures how "spread out" / accurate a vision reading is
  private Matrix<N3, N1> calculateEstimationStdDevs(EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {
    // Pose present. Start running Heuristic
    int numTags = 0;
    double avgDist = 0;
    double matrixScalar = 1.0; // defaults to 1.0!

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

    // calculate robot acceleration for further scaling
    SwerveDriveState robotState = m_robotSwerveStateSupplier.get();
    if (m_isFrontCamera && robotState != null && robotState.Speeds != null) {
      ChassisSpeeds currentSpeeds = robotState.Speeds;
      double currentTimestamp = robotState.Timestamp;
      double dt = currentTimestamp - m_previousTimestamp;

      // calculate acceleration
      double dx = currentSpeeds.vxMetersPerSecond - m_previousSpeed.vxMetersPerSecond;
      double dy = currentSpeeds.vyMetersPerSecond - m_previousSpeed.vyMetersPerSecond;

      if (dt > 0.0001) {
        double linearAcceleration = Math.hypot(dx, dy) / dt;

        if (Math.abs(linearAcceleration) > VisionConstants.MAX_ROBOT_ACCELERATION) {
          matrixScalar = VisionConstants.ACCELERATION_SCALAR;
        }
        // log stuff
        SmartDashboard.putNumber(m_logString + "/linearAcceleration", linearAcceleration);
      }
      else {
        SmartDashboard.putNumber(m_logString + "/linearAcceleration", 0.0);
      }

      // update previous speed and timestamp
      // NOTE: must copy values, not reference, to avoid aliasing the drivetrain's ChassisSpeeds object
      m_previousSpeed = new ChassisSpeeds(
          currentSpeeds.vxMetersPerSecond,
          currentSpeeds.vyMetersPerSecond,
          currentSpeeds.omegaRadiansPerSecond
      );
      m_previousTimestamp = currentTimestamp;

    }
    SmartDashboard.putNumber(m_logString + "/matrixScalar", matrixScalar);
    SmartDashboard.putBoolean(m_logString + "/isAccScalingActive", matrixScalar > 1.0);

    if (numTags == 0) {
      // No tags visible. Default to single-tag std devs
      SmartDashboard.putString(m_logString + "/standardDeviationState", "no tags visible");
      return VisionConstants.kSingleTagStdDevs.times(matrixScalar);
    } else if (numTags == 1 && avgDist > VisionConstants.VISION_DISTANCE_DISCARD) {
      SmartDashboard.putString(m_logString + "/standardDeviationState", "target too far");
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      var unscaledStdDevs = numTags > 1 ? VisionConstants.kMultiTagStdDevs : VisionConstants.kSingleTagStdDevs;
      avgDist /= numTags;
      // increase std devs based on (average) distance
      SmartDashboard.putString(m_logString + "/standardDeviationState", "good :)");
      return unscaledStdDevs.times(1 + (avgDist * avgDist / VisionConstants.STD_DEV_SCALAR)).times(matrixScalar);
    }
  }
}

