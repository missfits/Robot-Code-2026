package frc.robot.subsystems.vision;

import java.util.List;

import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;

public interface GlobalVisionFilter {
    /**
     * @return a new list of CameraReadings that are all "valid" based on a certain filter.
     * @param allReadings All camera readings at the same timestamp.
     *
     * Interface for filtering logic that looks at all cameras.
     */
    List<CameraReading> validReadings(List<CameraReading> allReadings);
}
