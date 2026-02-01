package frc.robot.subsystems.climber;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ClimberIOHardware extends MechanismsIOHardwareBase {

  public ClimberIOHardware(int motorID) {
    super(motorID, ClimberConstants.CLIMBER_MOTOR_STATOR_LIMIT, "climber/");
  }

  public double getPositionMeters() {
    return getPositionRevolutions() * ClimberConstants.CLIMBER_METERS_PER_REVOLUTION;
  }

  public double getVelocityMetersPerSecond() {
    return getMotorVelocityRevolutionsPerSecond() * ClimberConstants.CLIMBER_METERS_PER_REVOLUTION;
  }

  public void setPositionMeters(double meters) {
    double revolutions = meters / ClimberConstants.CLIMBER_METERS_PER_REVOLUTION;
    setPositionRevolutions(revolutions);
  }

}