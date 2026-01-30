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
import frc.robot.subsystems.vision.LocalizationCamera;
import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;
import frc.robot.subsystems.vision.filtering.LocalCameraPoseConsistencyFilter;
import frc.robot.subsystems.vision.filtering.LocalPoseZRollPitchFilter;

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

  // ==================== LocalCameraPoseConsistencyFilter Tests ====================
  @Test
  void testLocalCameraPoseConsistencyFilter_NotEnoughReadings() throws Exception {
    LocalCameraPoseConsistencyFilter filter = new LocalCameraPoseConsistencyFilter();

    // Test with fewer than NUM_LAST_EST_POSES readings - filter should return true (skip/pass)
    for (int i = 0; i < VisionConstants.NUM_LAST_EST_POSES - 1; i++) {
      LinkedList<CameraReading> readings = new LinkedList<>();

      for (int j = 0; j <= i; j++) {
        Pose3d pose = new Pose3d(j * 0.001, 0, 0, new Rotation3d(0, 0, 0));
        EstimatedRobotPose estPose = new EstimatedRobotPose(pose, j * 0.02, List.of(), null);
        readings.add(new CameraReading("test_camera", estPose, VisionConstants.kSingleTagStdDevs, j * 0.02, 1));
      }

      injectLastReadings(readings);

      // Create a new reading to validate
      Pose3d newPose = new Pose3d((i + 1) * 0.001, 0, 0, new Rotation3d(0, 0, 0));
      EstimatedRobotPose newEstPose = new EstimatedRobotPose(newPose, (i + 1) * 0.02, List.of(), null);
      CameraReading newReading = new CameraReading("test_camera", newEstPose, VisionConstants.kSingleTagStdDevs, (i + 1) * 0.02, 1);

      // With less than NUM_LAST_EST_POSES readings, filter should return true (skip filter)
      assertTrue(filter.isValid(newReading, m_camera),
          "Should return true (skip filter) when fewer than NUM_LAST_EST_POSES readings exist");
    }
  }

  @Test
  void testLocalCameraPoseConsistencyFilter_StablePoses() throws Exception {
    LocalCameraPoseConsistencyFilter filter = new LocalCameraPoseConsistencyFilter();

    // Inject stable poses that are close together with reasonable time intervals
    LinkedList<CameraReading> stableReadings = new LinkedList<>();

    double baseTime = 1.0;
    double timeInterval = 0.02; // 50 fps - 20ms between frames

    // Create history readings (NUM_LAST_EST_POSES readings)
    for (int i = 0; i < VisionConstants.NUM_LAST_EST_POSES; i++) {
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

    // Create a new stable reading to validate
    int nextIndex = VisionConstants.NUM_LAST_EST_POSES;
    Pose3d newPose = new Pose3d(1.0 + nextIndex * 0.001, 2.0, 0.0, new Rotation3d(0, 0, 0));
    double newTimestamp = baseTime + nextIndex * timeInterval;
    EstimatedRobotPose newEstPose = new EstimatedRobotPose(newPose, newTimestamp, List.of(), null);
    CameraReading newReading = new CameraReading("test_camera", newEstPose, VisionConstants.kSingleTagStdDevs, newTimestamp, 1);

    assertTrue(filter.isValid(newReading, m_camera),
        "Should return true when poses are stable and close together");
  }

  @Test
  void testLocalCameraPoseConsistencyFilter_JumpyPoses() throws Exception {
    LocalCameraPoseConsistencyFilter filter = new LocalCameraPoseConsistencyFilter();

    // Inject poses that jump around significantly
    LinkedList<CameraReading> jumpyReadings = new LinkedList<>();

    double baseTime = 1.0;
    double timeInterval = 0.02; // 50 fps

    for (int i = 0; i < VisionConstants.NUM_LAST_EST_POSES; i++) {
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

    // Create a new jumpy reading to validate
    int nextIndex = VisionConstants.NUM_LAST_EST_POSES;
    Pose3d newPose = new Pose3d(1.0 + nextIndex * 2.0, 2.0, 0.0, new Rotation3d(0, 0, 0));
    double newTimestamp = baseTime + nextIndex * timeInterval;
    EstimatedRobotPose newEstPose = new EstimatedRobotPose(newPose, newTimestamp, List.of(), null);
    CameraReading newReading = new CameraReading("test_camera", newEstPose, VisionConstants.kSingleTagStdDevs, newTimestamp, 1);

    assertFalse(filter.isValid(newReading, m_camera),
        "Should return false when poses move too fast (inconsistent)");
  }

  @Test
  void testLocalCameraPoseConsistencyFilter_ZeroTimeInterval() throws Exception {
    LocalCameraPoseConsistencyFilter filter = new LocalCameraPoseConsistencyFilter();

    // Edge case: all poses have the same timestamp - should return false (division by zero protection)
    LinkedList<CameraReading> sameTimeReadings = new LinkedList<>();

    double sameTime = 1.0;

    for (int i = 0; i < VisionConstants.NUM_LAST_EST_POSES; i++) {
      Pose3d pose = new Pose3d(1.0 + i * 0.01, 2.0, 0.0, new Rotation3d(0, 0, 0));
      EstimatedRobotPose estPose = new EstimatedRobotPose(pose, sameTime, List.of(), null);
      sameTimeReadings.add(new CameraReading("test_camera", estPose, VisionConstants.kSingleTagStdDevs, sameTime, 1));
    }

    injectLastReadings(sameTimeReadings);

    // Create a new reading with the same timestamp
    Pose3d newPose = new Pose3d(1.0 + VisionConstants.NUM_LAST_EST_POSES * 0.01, 2.0, 0.0, new Rotation3d(0, 0, 0));
    EstimatedRobotPose newEstPose = new EstimatedRobotPose(newPose, sameTime, List.of(), null);
    CameraReading newReading = new CameraReading("test_camera", newEstPose, VisionConstants.kSingleTagStdDevs, sameTime, 1);

    assertFalse(filter.isValid(newReading, m_camera),
        "Should return false when time interval is zero (division by zero protection)");
  }

  // ==================== LocalDistanceToFusedPoseFilter Tests ====================
  // NOTE: LocalDistanceToFusedPoseFilter requires a CommandSwerveDrivetrain instance,
  // which extends CTRE's TunerSwerveDrivetrain and requires complex CAN bus hardware setup.
  // To properly test this filter, consider one of the following approaches:
  // 1. Refactor the filter to accept a Supplier<Pose2d> instead of CommandSwerveDrivetrain
  // 2. Create an integration test with simulated drivetrain hardware
  // 3. Use a mocking framework like Mockito to mock the drivetrain's getState() method

  // ==================== LocalPoseZRollPitchFilter Tests ====================

  @Test
  void testLocalPoseZRollPitchFilter_ValidPose() {
    LocalPoseZRollPitchFilter filter = new LocalPoseZRollPitchFilter();

    // Create a valid pose with Z, roll, and pitch all within bounds
    Pose3d validPose = new Pose3d(
        1.0, 2.0,
        0.05, // Z within MAX_VISION_POSE_Z (0.1)
        new Rotation3d(
            0.02, // Roll within MAX_VISION_POSE_ROLL (0.05 rad)
            0.02, // Pitch within MAX_VISION_POSE_PITCH (0.05 rad)
            0.0   // Yaw doesn't matter
        )
    );

    EstimatedRobotPose estPose = new EstimatedRobotPose(validPose, 1.0, List.of(), null);
    CameraReading reading = new CameraReading("test_camera", estPose, VisionConstants.kSingleTagStdDevs, 1.0, 1);

    assertTrue(filter.isValid(reading, m_camera),
        "Should return true for pose with valid Z, roll, and pitch");
  }

  @Test
  void testLocalPoseZRollPitchFilter_InvalidZ() {
    LocalPoseZRollPitchFilter filter = new LocalPoseZRollPitchFilter();

    // Create a pose with Z above the maximum
    Pose3d invalidZPose = new Pose3d(
        1.0, 2.0,
        0.2, // Z above MAX_VISION_POSE_Z (0.1)
        new Rotation3d(0, 0, 0)
    );

    EstimatedRobotPose estPose = new EstimatedRobotPose(invalidZPose, 1.0, List.of(), null);
    CameraReading reading = new CameraReading("test_camera", estPose, VisionConstants.kSingleTagStdDevs, 1.0, 1);

    assertFalse(filter.isValid(reading, m_camera),
        "Should return false for pose with Z above maximum");
  }

  @Test
  void testLocalPoseZRollPitchFilter_InvalidRoll() {
    LocalPoseZRollPitchFilter filter = new LocalPoseZRollPitchFilter();

    // Create a pose with roll above the maximum
    Pose3d invalidRollPose = new Pose3d(
        1.0, 2.0, 0.0,
        new Rotation3d(
            0.1, // Roll above MAX_VISION_POSE_ROLL (0.05 rad)
            0.0,
            0.0
        )
    );

    EstimatedRobotPose estPose = new EstimatedRobotPose(invalidRollPose, 1.0, List.of(), null);
    CameraReading reading = new CameraReading("test_camera", estPose, VisionConstants.kSingleTagStdDevs, 1.0, 1);

    assertFalse(filter.isValid(reading, m_camera),
        "Should return false for pose with roll above maximum");
  }

  @Test
  void testLocalPoseZRollPitchFilter_InvalidPitch() {
    LocalPoseZRollPitchFilter filter = new LocalPoseZRollPitchFilter();

    // Create a pose with pitch above the maximum
    Pose3d invalidPitchPose = new Pose3d(
        1.0, 2.0, 0.0,
        new Rotation3d(
            0.0,
            0.1, // Pitch above MAX_VISION_POSE_PITCH (0.05 rad)
            0.0
        )
    );

    EstimatedRobotPose estPose = new EstimatedRobotPose(invalidPitchPose, 1.0, List.of(), null);
    CameraReading reading = new CameraReading("test_camera", estPose, VisionConstants.kSingleTagStdDevs, 1.0, 1);

    assertFalse(filter.isValid(reading, m_camera),
        "Should return false for pose with pitch above maximum");
  }

  @Test
  void testLocalPoseZRollPitchFilter_AllInvalid() {
    LocalPoseZRollPitchFilter filter = new LocalPoseZRollPitchFilter();

    // Create a pose with all values above maximum
    Pose3d allInvalidPose = new Pose3d(
        1.0, 2.0,
        0.5, // Z way above MAX_VISION_POSE_Z
        new Rotation3d(
            0.2, // Roll above MAX_VISION_POSE_ROLL
            0.2, // Pitch above MAX_VISION_POSE_PITCH
            0.0
        )
    );

    EstimatedRobotPose estPose = new EstimatedRobotPose(allInvalidPose, 1.0, List.of(), null);
    CameraReading reading = new CameraReading("test_camera", estPose, VisionConstants.kSingleTagStdDevs, 1.0, 1);

    assertFalse(filter.isValid(reading, m_camera),
        "Should return false for pose with all values above maximum");
  }

  @Test
  void testLocalPoseZRollPitchFilter_NegativeRoll() {
    LocalPoseZRollPitchFilter filter = new LocalPoseZRollPitchFilter();

    // Create a pose with large negative roll (should be invalid - magnitude exceeds threshold)
    Pose3d negativeRollPose = new Pose3d(
        1.0, 2.0, 0.0,
        new Rotation3d(
            -0.1, // Negative roll with magnitude above MAX_VISION_POSE_ROLL (0.05 rad)
            0.0,
            0.0
        )
    );

    EstimatedRobotPose estPose = new EstimatedRobotPose(negativeRollPose, 1.0, List.of(), null);
    CameraReading reading = new CameraReading("test_camera", estPose, VisionConstants.kSingleTagStdDevs, 1.0, 1);

    assertFalse(filter.isValid(reading, m_camera),
        "Should return false for pose with negative roll magnitude above maximum");
  }

  @Test
  void testLocalPoseZRollPitchFilter_NegativePitch() {
    LocalPoseZRollPitchFilter filter = new LocalPoseZRollPitchFilter();

    // Create a pose with large negative pitch (should be invalid - magnitude exceeds threshold)
    Pose3d negativePitchPose = new Pose3d(
        1.0, 2.0, 0.0,
        new Rotation3d(
            0.0,
            -0.1, // Negative pitch with magnitude above MAX_VISION_POSE_PITCH (0.05 rad)
            0.0
        )
    );

    EstimatedRobotPose estPose = new EstimatedRobotPose(negativePitchPose, 1.0, List.of(), null);
    CameraReading reading = new CameraReading("test_camera", estPose, VisionConstants.kSingleTagStdDevs, 1.0, 1);

    assertFalse(filter.isValid(reading, m_camera),
        "Should return false for pose with negative pitch magnitude above maximum");
  }

  @Test
  void testLocalPoseZRollPitchFilter_NegativeZ() {
    LocalPoseZRollPitchFilter filter = new LocalPoseZRollPitchFilter();

    // Create a pose with large negative Z (robot below ground - should be invalid)
    Pose3d negativeZPose = new Pose3d(
        1.0, 2.0,
        -0.2, // Negative Z with magnitude above MAX_VISION_POSE_Z (0.1)
        new Rotation3d(0, 0, 0)
    );

    EstimatedRobotPose estPose = new EstimatedRobotPose(negativeZPose, 1.0, List.of(), null);
    CameraReading reading = new CameraReading("test_camera", estPose, VisionConstants.kSingleTagStdDevs, 1.0, 1);

    assertFalse(filter.isValid(reading, m_camera),
        "Should return false for pose with negative Z magnitude above maximum (robot below ground)");
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