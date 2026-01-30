package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.photonvision.EstimatedRobotPose;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;

import java.lang.reflect.Field;
import java.util.LinkedList;
import java.util.List;

class LocalizationCameraTest {
  static final double DELTA = 1e-5; // acceptable deviation range
  static final int NUM_TEST_EST_POSES = 10;

  LocalizationCamera m_camera;
  Transform3d m_robotToCam;

  @BeforeEach
  void setup() {
    // Initialize the HAL for simulation
    assert HAL.initialize(500, 0);
    
    // Create a test camera transform
    m_robotToCam = new Transform3d(
        new Translation3d(0.2, 0.0, 0.5),
        new Rotation3d(0, Math.toRadians(-15), 0)
    );
    
    m_camera = new LocalizationCamera("test_camera", m_robotToCam);
  }

  @AfterEach
  void shutdown() {
    // Cleanup - LocalizationCamera doesn't implement AutoCloseable, 
    // but we should still clean up HAL state
    HAL.shutdown();
  }

  // ==================== Basic Getter Tests ====================

  @Test
  void testGetCameraName() {
    assertEquals("test_camera", m_camera.getCameraName());
  }

  @Test
  void testInitialCameraReadingIsEmpty() {
    assertTrue(m_camera.getCameraReading().isEmpty(),
        "Initial camera reading should be empty");
  }

  // ==================== areRecentCameraPosesConsistent Tests ====================

  @Test
  void testAreRecentCameraPosesConsistent_NoReadings() throws Exception {
    // With no readings added, should return false (not enough data)
    assertFalse(m_camera.areRecentCameraPosesConsistent(),
        "Should return false when no readings exist");
  }

  @Test
  void testAreRecentCameraPosesConsistent_NotEnoughReadings() throws Exception {
    LinkedList<CameraReading> stableReadings = new LinkedList<>();

    for (int i = 0; i < NUM_TEST_EST_POSES-1; i++) {
      Pose3d pose = new Pose3d(0 + i * 0.001, 0, 0, new Rotation3d(0, 0, 0));
      EstimatedRobotPose estPose = new EstimatedRobotPose(pose, i * 0.02, List.of(), null);
      stableReadings.add(new CameraReading("test_camera", estPose, VisionConstants.kSingleTagStdDevs,i * 0.02,1));
      injectLastReadings(stableReadings);

      // With less than NUM_LAST_EST_POSES reading added, should return false (not enough data)
      // assertFalse(m_camera.areRecentCameraPosesConsistent(),
          // "Should return false when fewer than NUM_LAST_EST_POSES readings exist");

      stableReadings.remove(0);
    }
  }

  @Test
  void testAreRecentCameraPosesConsistent_StablePoses() throws Exception {
    // Inject stable poses that are close together with reasonable time intervals
    LinkedList<CameraReading> stableReadings = new LinkedList<>();

    double baseTime = 1.0;
    double timeInterval = 0.02; // 50 fps - 20ms between frames

    for (int i = 0; i < NUM_TEST_EST_POSES; i++) {
      // Create poses that move very slowly (well under the max speed threshold)
      Pose3d pose = new Pose3d(
          1.0 + i * 0.001, // Very small movement: 1mm per reading
          2.0,
          0.0, // Z = 0 to be sane
          new Rotation3d(0, 0, 0)
      );
      double timestamp = baseTime + i * timeInterval;
      EstimatedRobotPose estPose = new EstimatedRobotPose(pose, timestamp, List.of(), null);
      stableReadings.add(new CameraReading(
          "test_camera",
          estPose,
          VisionConstants.kSingleTagStdDevs,
          timestamp,
          1
      ));
    }

    injectLastReadings(stableReadings);

    assertTrue(m_camera.areRecentCameraPosesConsistent(),
        "Should return true when poses are stable and close together");
  }

  @Test
  void testAreRecentCameraPosesConsistent_JumpyPoses() throws Exception {
    // Inject poses that jump around significantly
    LinkedList<CameraReading> jumpyReadings = new LinkedList<>();

    double baseTime = 1.0;
    double timeInterval = 0.02; // 50 fps

    for (int i = 0; i < NUM_TEST_EST_POSES; i++) {
      // Create poses that move very fast (well above max speed threshold)
      // Max speed is MAX_AVG_DIST * 50, so we need to exceed that
      Pose3d pose = new Pose3d(
          1.0 + i * 2.0, // 2 meters per reading = 100 m/s at 50fps
          2.0,
          0.0,
          new Rotation3d(0, 0, 0)
      );
      double timestamp = baseTime + i * timeInterval;
      EstimatedRobotPose estPose = new EstimatedRobotPose(pose, timestamp, List.of(), null);
      jumpyReadings.add(new CameraReading(
          "test_camera",
          estPose,
          VisionConstants.kSingleTagStdDevs,
          timestamp,
          1
      ));
    }

    injectLastReadings(jumpyReadings);

    assertFalse(m_camera.areRecentCameraPosesConsistent(),
        "Should return false when poses move too fast (inconsistent)");
  }

  @Test
  void testAreRecentCameraPosesConsistent_ZeroTimeInterval() throws Exception {
    // Edge case: all poses have the same timestamp - should return false (inconsistent)
    LinkedList<CameraReading> sameTimeReadings = new LinkedList<>();

    double sameTime = 1.0;

    for (int i = 0; i < NUM_TEST_EST_POSES; i++) {
      Pose3d pose = new Pose3d(1.0 + i * 0.01, 2.0, 0.0, new Rotation3d(0, 0, 0));
      EstimatedRobotPose estPose = new EstimatedRobotPose(pose, sameTime, List.of(), null);
      sameTimeReadings.add(new CameraReading("test_camera", estPose, VisionConstants.kSingleTagStdDevs, sameTime, 1));
    }

    injectLastReadings(sameTimeReadings);

    assertFalse(m_camera.areRecentCameraPosesConsistent(),
        "Should return false when time interval is zero (inconsistent)");
  }

  // ==================== Helper Methods ====================

  /**
   * Uses reflection to inject test readings into the private m_lastReadings field
   */
  private void injectLastReadings(LinkedList<CameraReading> readings) throws Exception {
    Field field = LocalizationCamera.class.getDeclaredField("m_lastReadings");
    field.setAccessible(true);
    field.set(m_camera, readings);
  }
}

// 