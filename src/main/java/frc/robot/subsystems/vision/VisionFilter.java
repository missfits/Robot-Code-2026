package frc.robot.subsystems.vision;

import java.util.List;

import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;
import frc.robot.subsystems.vision.LocalizationCamera;


public interface VisionFilter {
    /**
     * Determines if a camera reading is valid based on filtering criteria.
     * 
     * @param reading The camera reading to validate
     * @param allReadings All current camera readings (for cross-camera comparison)
     * @return true if the reading passes the filter, false otherwise
     */
    boolean isValid(CameraReading reading, LocalizationCamera camera);
}
