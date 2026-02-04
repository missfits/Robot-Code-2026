package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class PivotIOHardware extends MechanismsIOHardwareBase {

  public PivotIOHardware(int motorID) {
    super(motorID, IntakeConstants.PIVOT_MOTOR_STATOR_LIMIT, "pivot/");
  }

  public double getPositionRadians() {
    return Math.toRadians(getPositionRevolutions() * IntakeConstants.PIVOT_DEGREES_PER_REVOLUTION);
  }

  public double getPositionDegrees() {
    return getPositionRevolutions() * IntakeConstants.PIVOT_DEGREES_PER_REVOLUTION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRevolutionsPerSecond() * IntakeConstants.PIVOT_DEGREES_PER_REVOLUTION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * IntakeConstants.PIVOT_DEGREES_PER_REVOLUTION;
  }

  public void setPositionRadians(double radians) {
    double revolutions = Math.toDegrees(radians) / IntakeConstants.PIVOT_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

  public void setPositionDegrees(double degrees) {
    double revolutions = degrees / IntakeConstants.PIVOT_DEGREES_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }
}