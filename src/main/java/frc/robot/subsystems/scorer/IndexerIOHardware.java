package frc.robot.subsystems.scorer;

import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class IndexerIOHardware extends MechanismsIOHardwareBase {

  public IndexerIOHardware(int motorID) {
    super(motorID, ScorerConstants.INDEXER_MOTOR_STATOR_LIMIT, "indexer/");
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * ScorerConstants.INDEXER_DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * ScorerConstants.INDEXER_DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * ScorerConstants.INDEXER_DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * ScorerConstants.INDEXER_DEGREES_PER_REVOLUTION;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / ScorerConstants.INDEXER_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / ScorerConstants.INDEXER_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }
}