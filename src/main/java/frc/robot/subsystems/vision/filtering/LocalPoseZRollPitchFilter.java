package frc.robot.subsystems.vision.filtering;

import frc.robot.subsystems.vision.LocalVisionFilter;
import frc.robot.subsystems.vision.LocalizationCamera;
import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;
import frc.robot.subsystems.vision.VisionUtils;

public class LocalPoseZRollPitchFilter implements LocalVisionFilter{
    /*
     * Filter for isPoseSane() check in Vision.Utils
     * Checks if z of estimatedPose, roll of reading, and pitch of reading are within certain bounds
     */
    public LocalPoseZRollPitchFilter() {}


    @Override
    public boolean isValid(CameraReading reading, LocalizationCamera cam) {
        return VisionUtils.poseIsSane(reading.robotPose().estimatedPose);
    }
}