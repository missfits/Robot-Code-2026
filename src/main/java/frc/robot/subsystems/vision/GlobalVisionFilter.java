package frc.robot.subsystems.vision;

import java.util.List;

import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;

public interface GlobalVisionFilter {
    /**
     * Returns true if the given camera reading is valid.
     * @param reading The camera reading to validate.
     * @param allReadings All camera readings at the same timestamp.
     * @return True if the reading is valid.
     * 
     * Interface for filtering logic that looks at all cameras.
     */
    boolean isValid(CameraReading reading, List<CameraReading> allReadings);
}
