// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.Queue;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;
import frc.robot.subsystems.vision.VisionUtils;



public class VisionSubsystem extends SubsystemBase {
  private ArrayList<LocalizationCamera> cameras = new ArrayList<>();
  private List<LocalizationCamera> camerasWithReadings = new ArrayList<>();
  private List<LocalizationCamera> camerasWithValidPose = new ArrayList<>();
  private Optional<CameraReading> m_lastReading; // last vision measurement added to drivetrain

  /** Creates a new Vision Subsystem. */
  public VisionSubsystem() {
    cameras.add(new LocalizationCamera(VisionConstants.CAMERA1_NAME, VisionConstants.ROBOT_TO_CAM1_3D));
    cameras.add(new LocalizationCamera(VisionConstants.CAMERA2_NAME, VisionConstants.ROBOT_TO_CAM2_3D));
  }

  public List<LocalizationCamera> getCameraReadings(){
    return camerasWithValidPose;
  }

  @Override
  public void periodic() {
    for (LocalizationCamera cam : cameras){
      cam.updateCameraReading();
    }

    camerasWithReadings = cameras.stream()
    .filter((camera) -> {
      return camera.getCameraReading()
        .flatMap(CameraReading::robotPose).isPresent();}).toList();

    // sorts the camera readings by time (care less about older readings)
    
    camerasWithValidPose = camerasWithReadings.stream() // turn the list into a stream
    .filter((camera) -> { // only get the cameras with a valid, filtered EstimatedRobotPose
         return filterCameraReading(camera).isPresent();
    })
    .sorted((camera_a, camera_b) -> { // simplified comparator because we've filtered out invalid readings.
         return Double.compare(camera_a.getCameraReading().orElseThrow().timestampSeconds(), camera_b.getCameraReading().orElseThrow().timestampSeconds());
     })
    .toList();

    if (camerasWithValidPose.size() > 0) {
      // update m_lastReading to be the most recent reading
      m_lastReading = camerasWithValidPose.get(camerasWithValidPose.size() - 1).getCameraReading();
    }
  }

  private Optional<CameraReading> filterCameraReading(LocalizationCamera cam) {
    if (!cam.getCameraReading().isPresent()) {
      return Optional.empty();
    }

    CameraReading reading = cam.getCameraReading().get();

    // --- timestamp check ---
    if (m_lastReading.isPresent()) {
      // if timestamp is older than last reading, return empty
      if (reading.timestampSeconds() < m_lastReading.get().timestampSeconds()) {
        SmartDashboard.putString("vision/" + cam.getCameraName() + "/timestampCheck", "failed");
        return Optional.empty();
      }
    }

    // return reading if roll/pitch/yaw is sane AND reading is NOT jumpy (compared to all newest readings)
    return VisionUtils.poseIsSane(reading.robotPose().get().estimatedPose) && !cameraReadingMatchesOtherReadings(reading) ? Optional.of(reading) : Optional.empty();
  }

  // checks if reading is too far from avg pose calculated from all camera readings
  private boolean cameraReadingMatchesOtherReadings(CameraReading reading) {
    // not enough data to determine jumpiness
    if (camerasWithReadings.size() < VisionConstants.MIN_NUM_CAMERA_READINGS) {
      return false;
    }

    double sumX = 0, sumY = 0;
    int numPoses = 0;
    // cam.getCameraReading() will never be empty bc camerasWithReadings will filter.
    for (LocalizationCamera cam : camerasWithReadings) {
      if (cam.getCameraReading().get().robotPose().isPresent()){
        Pose2d pose = cam.getCameraReading().get().robotPose().get().estimatedPose.toPose2d();
        sumX += pose.getX();
        sumY += pose.getY();
        numPoses++;
      }
    }

    // checks for division by zero
    if (numPoses == 0){
      return false;
    }

    Pose2d avgPose = new Pose2d(sumX / numPoses, sumY / numPoses, new Rotation2d());

    return reading.robotPose().get().estimatedPose.toPose2d().getTranslation().getDistance(avgPose.getTranslation()) > VisionConstants.MAX_VISION_READING_DISTANCE;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}