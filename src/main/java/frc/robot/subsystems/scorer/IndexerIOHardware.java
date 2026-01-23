package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants.IndexerConstants;

public class IndexerIOHardware {

    private final TalonFX m_indexerMotor = new TalonFX(IndexerConstants.MECHANISM_MOTOR_ID);

    private final StatusSignal<AngularVelocity> m_velocitySignal = m_indexerMotor.getVelocity();
    private final StatusSignal<Voltage> m_voltageSignal = m_indexerMotor.getMotorVoltage();

    // constructor
    public IndexerIOHardware() {
        var talonFXConfigurator = m_indexerMotor.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = IndexerConstants.MOTOR_STATOR_LIMIT;
        limitConfigs.StatorCurrentLimitEnable = true;

        talonFXConfigurator.apply(limitConfigs);
    }

    // getters
    public double getVelocity() { //in radians
    return Math.toRadians(m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*IndexerConstants.DEGREES_PER_ROTATION);
  }

    public double getVoltae() {
    return m_voltageSignal.refresh().getValue().in(Volts);
  }

    // setters
    public void motorOff() {
        m_indexerMotor.stopMotor();
    }

    public void setVoltage(double volts) {
        m_indexerMotor.setControl(new VoltageOut(volts));
    }
}
