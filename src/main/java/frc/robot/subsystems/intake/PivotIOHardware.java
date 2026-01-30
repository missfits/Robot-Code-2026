package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class PivotIOHardware extends MechanismsIOHardwareBase {

  public PivotIOHardware(int motorID) {
    super(motorID, IntakeConstants.PIVOT_MOTOR_STATOR_LIMIT);
  }

  public double getPositionRadians() {
    return Math.toRadians(getRotation() * IntakeConstants.PIVOT_DEGREES_PER_ROTATION);
  }

  public double getPositionDegrees() {
    return getRotation() * IntakeConstants.PIVOT_DEGREES_PER_ROTATION;
  }

  public double getVelocityRadiansPerSecond() {
    return Math.toRadians(getMotorVelocityRPS() * IntakeConstants.PIVOT_DEGREES_PER_ROTATION);
  }

  public double getVelocityDegreesPerSecond() {
    return getMotorVelocityRPS() * IntakeConstants.PIVOT_DEGREES_PER_ROTATION;
  }

  public void setPositionRadians(double radians) {
    double rotations = Math.toRadians(radians / IntakeConstants.PIVOT_DEGREES_PER_ROTATION);
    setPositionRotations(rotations);
  }

  public void setPositionDegrees(double degrees) {
    double rotations = degrees / IntakeConstants.PIVOT_DEGREES_PER_ROTATION;
    setPositionRotations(rotations);
  }
}