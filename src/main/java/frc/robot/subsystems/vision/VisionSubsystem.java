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
import frc.robot.subsystems.vision.VisionUtils;



public class VisionSubsystem extends SubsystemBase {
  private ArrayList<LocalizationCamera> cameras = new ArrayList<>();
  private List<LocalizationCamera> camerasWithValidPose = new ArrayList<>();

  private double m_lastTimestamp = 0.0;

  /** Creates a new Vision Subsystem. */
  public VisionSubsystem() {
    cameras.add(new LocalizationCamera(VisionConstants.CAMERA1_NAME, VisionConstants.ROBOT_TO_CAM1_3D));
    cameras.add(new LocalizationCamera(VisionConstants.CAMERA2_NAME, VisionConstants.ROBOT_TO_CAM2_3D));
  }

  public List<LocalizationCamera> getLocalizationCameras(){
    return camerasWithValidPose;
  }

  @Override
  public void periodic() {
    for (LocalizationCamera cam : cameras){
      cam.updateCameraReading();
    }

    // sorts the camera readings by time (care less about older readings)
    camerasWithValidPose = cameras.stream() // turn the list into a stream
    .filter((camera) -> { // only get the cameras with a valid EstimatedRobotPose
         return camera.getCameraReading().isPresent() && camera.getCameraReading().get().timestampSeconds() > m_lastTimestamp;
    })
    .sorted((camera_a, camera_b) -> { // simplified comparator because we've filtered out invalid readings.
         return Double.compare(camera_a.getCameraReading().get().timestampSeconds(), 
                              camera_b.getCameraReading().get().timestampSeconds());
     })
    .toList();
    
    if (camerasWithValidPose.size() > 0){
      // update last timestamp
      m_lastTimestamp = camerasWithValidPose.get(camerasWithValidPose.size() - 1).getCameraReading().get().timestampSeconds();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}