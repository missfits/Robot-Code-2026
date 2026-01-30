package frc.robot.subsystems.scorer;

import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class IndexerIOHardware extends MechanismsIOHardwareBase {

  public IndexerIOHardware(int motorID) {
    super(motorID, ScorerConstants.INDEXER_MOTOR_STATOR_LIMIT);
  }

  public double getPositionRadians() {
    return Math.toRadians(getRotation() * ScorerConstants.INDEXER_DEGREES_PER_ROTATION);
  }

  public double getPositionDegrees() {
    return getRotation() * ScorerConstants.INDEXER_DEGREES_PER_ROTATION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRPS() * ScorerConstants.INDEXER_DEGREES_PER_ROTATION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRPS() * ScorerConstants.INDEXER_DEGREES_PER_ROTATION;
  }

  public void setPositionRadians(double radians) {
    double rotations = Math.toRadians(radians / ScorerConstants.INDEXER_DEGREES_PER_ROTATION);
    setPositionRotations(rotations);
  }

  public void setPositionDegrees(double degrees) {
    double rotations = degrees / ScorerConstants.INDEXER_DEGREES_PER_ROTATION;
    setPositionRotations(rotations);
  }
}