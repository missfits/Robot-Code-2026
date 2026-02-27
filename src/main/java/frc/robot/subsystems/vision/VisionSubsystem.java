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
import edu.wpi.first.wpilibj2.command.Command;
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

  // all valid readings from all cameras in the same periodic loop
  private List<CameraReading> allValidReadings = new ArrayList<>();

  // filtering pipeline for logic that requires ALL cameras
  private GlobalVisionFilterPipeline globalFilterPipeline = new GlobalVisionFilterPipeline();

  private double m_lastTimestamp = 0.0;

  private boolean m_driverModeEnabled = false; // true = raw video feed, false = normal AprilTag processing

  /** Creates a new Vision Subsystem. */
  public VisionSubsystem() {
    cameras.add(new LocalizationCamera(VisionConstants.CAMERA1_NAME, VisionConstants.ROBOT_TO_CAM1_3D));
    cameras.add(new LocalizationCamera(VisionConstants.CAMERA2_NAME, VisionConstants.ROBOT_TO_CAM2_3D));
  }

  public List<CameraReading> getValidCameraReadings(){
    return allValidReadings;
  }

  // --- filtering methods ---
  // NOTE: only sets single instance variable globalFilterPipeline
  public void setGlobalFilterPipeline(GlobalVisionFilterPipeline filterPipeline) {
    globalFilterPipeline = filterPipeline;
  }

  // NOTE: sets filterPipeline for each camera INDIVIDUALLY, requires loop
  public void setLocalFilteringPipeline(LocalVisionFilterPipeline filterPipeline){
    for (LocalizationCamera cam : cameras){
      cam.setFilterPipeline(filterPipeline);
    }
  }

  /*
   * Helper method that toggles driver mode on all cameras.
   * NOTE: driver mode means robot does nothing with the camera input
   * @param driverMode true = raw video field
   *                   false = normal AprilTag processing
   */
  public void setCamerasToDriverMode(boolean driverMode) {
    for (LocalizationCamera cam : cameras) {
      cam.setDriverMode(driverMode);
    }
    SmartDashboard.putBoolean("vision/oldDriverModeState", driverMode);
    SmartDashboard.putBoolean("vision/driverMode", driverMode);
  }

  // Command to toggle driver mode on all cameras
  public Command toggleDriverModeCommand() {
    return runOnce(() -> {
      m_driverModeEnabled = !m_driverModeEnabled;
      setCamerasToDriverMode(m_driverModeEnabled);
    });
  }

  @Override
  public void periodic() {
    // clear allValidReadings at the beginning of every loop
    allValidReadings.clear();

    // --- FILTERING ---
    if (globalFilterPipeline.getNumFilters() > 0) {
      // Get most recent filter toggle states from SmartDashboard
      globalFilterPipeline.updateSmartDashboardToggles();
    }

    for (LocalizationCamera cam : cameras){
      cam.updateCameraReading();

      // If no camera reading, skip filtering.
      if (!cam.getCameraReading().isPresent()) {
        continue;
      }

      CameraReading newReading = cam.getCameraReading().get();

      // Filter by timestamp: if camera reading more recent than last recorded reading,
      //    AND passes ALL GlobalFilterPipeline filters, add to allValidReadings.
      if (isReadingTimestampValid(cam)) {
        allValidReadings.add(newReading);
      }
    }

    // Run all enabled global filters on allValidReadings
    // NOTE: upates allValidReadings into readings that have passed all the filters
    //       --> runAll() returns a list of readings that passed all the filters
    if (globalFilterPipeline.getNumFilters() > 0) {
      allValidReadings = globalFilterPipeline.runAll(allValidReadings);
    }

    // Sort allValidReadings by timestamp (oldest first)
    allValidReadings.sort(Comparator.comparingDouble(CameraReading::timestampSeconds));
    
    // Periodically update m_lastTimestamp!
    if (allValidReadings.size() > 0){
      m_lastTimestamp = allValidReadings.get(allValidReadings.size() - 1).timestampSeconds();
    }
  }

  // Returns true if the given camera reading is newer than the last timestamp
  // NOTE: cam.getCameraReading().get() will never be .empty() bc of periodic() failsafe.
  private boolean isReadingTimestampValid(LocalizationCamera cam) {
    return cam.getCameraReading().get().timestampSeconds() > m_lastTimestamp;
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}