package frc.robot.subsystems.climber;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.MechanismsIOHardwareBase;

public class ClimberIOHardware extends MechanismsIOHardwareBase {

  public ClimberIOHardware(int motorID) {
    super(motorID, ClimberConstants.CLIMBER_MOTOR_STATOR_LIMIT);
  }

  public double getPositionMeters() {
    return getRotations() * ClimberConstants.CLIMBER_METERS_PER_ROTATION;
  }

  public double getVelocityMetersPerSecond() {
    return getRotationsPerSecond() * ClimberConstants.CLIMBER_METERS_PER_ROTATION;
  }
}