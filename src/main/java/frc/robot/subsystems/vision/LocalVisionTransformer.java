package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;

// Interface for changing single-tag readings to alternate pose before filtering
public interface LocalVisionTransformer {
  /**
   * Transforms a CameraReading before filtering.
   * Can modify which pose is "primary" for single-tag readings.
   * 
   * @param reading The camera reading (may contain alternatePose for single-tag)
   * @param cam The camera that produced the reading
   * @return The (possibly modified) CameraReading to pass to filters
   */
  CameraReading transformReading(CameraReading reading);
}
