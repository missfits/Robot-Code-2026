import au.grapplerobotics.LaserCan;
import au.grapplerobotics.CanBridge;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Class for a LaserCAN sensor
// acting as a beam break sensor.
public class LaserCANSensorBase extends SubsystemBase {
  private LaserCan m_intakeSensor;
  private LaserCan.Measurement m_intakeSensorMeasurement;
  private String m_logPrefix;
  private double m_minDistance;

  public LaserCANSensorBase(int canID, String logPrefix, double minDistanceForBeamBreak) {
    CanBridge.runTCP(); // allow grapplehook to communicate w/ lasercan:)
    m_intakeSensor = new LaserCan(canID);
    m_logPrefix = logPrefix;
    m_minDistance = minDistanceForBeamBreak; 
  }

  public Trigger beamBrokenTrigger() {
    return new Trigger(() -> beamBroken());
  }

  public boolean beamBroken() {
    return m_intakeSensorMeasurement != null && 
      m_intakeSensorMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && 
      m_intakeSensorMeasurement.distance_mm < m_minDistance &&
      m_intakeSensorMeasurement.distance_mm > 0.1; // from 2025 -- bypass lasercan skill issue (periodically outputs 0 mm)
  }

  @Override
  public void periodic() {
    m_intakeSensorMeasurement = m_intakeSensor.getMeasurement();
    if (m_intakeSensorMeasurement != null) {
      SmartDashboard.putNumber("" + m_logPrefix + "/distance", m_intakeSensorMeasurement.distance_mm);
      SmartDashboard.putBoolean("" + m_logPrefix + "/isValid", m_intakeSensorMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
    }

    SmartDashboard.putBoolean("" + m_logPrefix + "/beamBroken", beamBroken());

  }
}
