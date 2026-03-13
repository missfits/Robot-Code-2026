package frc.robot.subsystems.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.FieldConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DrivetrainCommandFactoryTest {
  private static final double DELTA = 1e-9;
  private static final double POSE_OFFSET_METERS = 0.1;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
  }

  @AfterEach
  void shutdown() {
    DriverStationSim.resetData();
    HAL.shutdown();
  }

  @Test
  void calculateBumpAngle_blueAllianceZone_returnsZeroDegrees() {
    setAllianceStation(AllianceStationID.Blue1);

    Rotation2d heading = DrivetrainCommandFactory.calculateBumpAngle(
        new Pose2d(FieldConstants.LinesVertical.hubCenter - POSE_OFFSET_METERS, 0.0, Rotation2d.kZero));

    assertRotationEquals(0.0, heading);
  }

  @Test
  void calculateBumpAngle_blueOutsideAllianceZone_returnsOneEightyDegrees() {
    setAllianceStation(AllianceStationID.Blue1);

    Rotation2d heading = DrivetrainCommandFactory.calculateBumpAngle(
        new Pose2d(FieldConstants.LinesVertical.hubCenter + POSE_OFFSET_METERS, 0.0, Rotation2d.kZero));

    assertRotationEquals(180.0, heading);
  }

  @Test
  void calculateBumpAngle_redAllianceZone_returnsOneEightyDegrees() {
    setAllianceStation(AllianceStationID.Red1);

    Rotation2d heading = DrivetrainCommandFactory.calculateBumpAngle(
        new Pose2d(FieldConstants.fieldLength - FieldConstants.LinesVertical.hubCenter + POSE_OFFSET_METERS, 0.0, Rotation2d.kZero));

    assertRotationEquals(180.0, heading);
  }

  @Test
  void calculateBumpAngle_redOutsideAllianceZone_returnsZeroDegrees() {
    setAllianceStation(AllianceStationID.Red1);

    Rotation2d heading = DrivetrainCommandFactory.calculateBumpAngle(
        new Pose2d(FieldConstants.fieldLength - FieldConstants.LinesVertical.hubCenter - POSE_OFFSET_METERS, 0.0, Rotation2d.kZero));

    assertRotationEquals(0.0, heading);
  }

  private static void setAllianceStation(AllianceStationID allianceStationID) {
    DriverStationSim.setAllianceStationId(allianceStationID);
    DriverStationSim.notifyNewData();
  }

  private static void assertRotationEquals(double expectedDegrees, Rotation2d actualRotation) {
    assertEquals(0.0, actualRotation.minus(Rotation2d.fromDegrees(expectedDegrees)).getRadians(), DELTA);
  }
}