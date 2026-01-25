package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;

public interface LocalVisionFilter {
    /**
     * Returns true if the given camera reading is valid.
     * @param reading The camera reading to validate.
     * @param cam The camera that took the reading.
     * @return True if the reading is valid.
     * 
     * Interface for filtering logic that is specific to one camera.
     */
    boolean isValid(CameraReading reading, LocalizationCamera cam);

}
