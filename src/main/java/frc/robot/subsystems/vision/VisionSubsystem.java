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
  private List<CameraReading> readingsWithValidPose = new ArrayList<>();

  private double m_lastTimestamp = 0.0;

  // has to be optional b/c starts as empty
  private Optional<CameraReading> m_lastReading; // last vision measurement added to drivetrain


  /** Creates a new Vision Subsystem. */
  public VisionSubsystem() {
    cameras.add(new LocalizationCamera(VisionConstants.CAMERA1_NAME, VisionConstants.ROBOT_TO_CAM1_3D));
    cameras.add(new LocalizationCamera(VisionConstants.CAMERA2_NAME, VisionConstants.ROBOT_TO_CAM2_3D));
  }

  public List<CameraReading> getCameraReadings(){
    return readingsWithValidPose;
  }

  @Override
  public void periodic() {
    for (LocalizationCamera cam : cameras){
      cam.updateCameraReading();
    }
        
    // sorts the camera readings by time (care less about older readings)
    readingsWithValidPose = cameras.stream() // turn the list into a stream
    .flatMap((camera) -> { // only get the cameras with a valid EstimatedRobotPose
      return filterCameraReading(camera).stream();  // .stream() converts Optional to Stream
    })
    .sorted((reading_a, reading_b) -> { // simplified comparator because we've filtered out invalid readings.
         return Double.compare(reading_a.timestampSeconds(),
                              reading_b.timestampSeconds());
     })
    .toList();
    
    if (readingsWithValidPose.size() > 0){
      // update most recent timestamp
      m_lastTimestamp = readingsWithValidPose.get(readingsWithValidPose.size() - 1).timestampSeconds();

      // update m_lastReading to be the most recent reading
      m_lastReading = Optional.of(readingsWithValidPose.get(readingsWithValidPose.size() - 1));
    }
  }

  public void setFilter(VisionFilter filter) {
    // loop through cameras and set filter for each one
    for (LocalizationCamera cam : cameras) {
      cam.setVisionFilter(filter);
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
      if (reading.timestampSeconds() < m_lastTimestamp) {
        SmartDashboard.putString("vision/" + cam.getCameraName() + "/timestampCheck", "failed");
        return Optional.empty();
      }
    }

    // return reading if reading is NOT jumpy (compared to all newest readings)
    return Optional.of(reading);
  }

  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}