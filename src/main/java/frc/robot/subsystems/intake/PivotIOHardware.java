package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants.IntakeConstants;

public class PivotIOHardware {
    private final TalonFX m_pivotMotor;
    private final StatusSignal<Angle> m_positionSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;


    // constructor
    public PivotIOHardware(int motorID) {
        m_pivotMotor = new TalonFX(motorID);
        m_positionSignal = m_pivotMotor.getPosition();
        m_velocitySignal = m_pivotMotor.getVelocity();
        m_voltageSignal = m_pivotMotor.getMotorVoltage();
        m_currentSignal = m_pivotMotor.getStatorCurrent();

        var talonFXConfigurator = m_pivotMotor.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = IntakeConstants.PIVOT_MOTOR_STATOR_LIMIT;
        limitConfigs.StatorCurrentLimitEnable = true;

        talonFXConfigurator.apply(limitConfigs);

    }

    // getters
    public double getPosition() {
        return Math.toRadians(m_positionSignal.refresh().getValue().in(Revolutions)*IntakeConstants.PIVOT_DEGREES_PER_ROTATION);
    }

    public double getPositionDegrees() {
        return m_positionSignal.refresh().getValue().in(Revolutions)*IntakeConstants.PIVOT_DEGREES_PER_ROTATION;
    }

    public double getVelocity() {
        return Math.toRadians(m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*IntakeConstants.PIVOT_DEGREES_PER_ROTATION);
    }

    public double getVelocityDegrees() {
        return m_velocitySignal.refresh().getValue().in(RevolutionsPerSecond)*IntakeConstants.PIVOT_DEGREES_PER_ROTATION;
    }

    public double getVoltage() {
        return m_voltageSignal.refresh().getValue().in(Volts);
    }

    public double getCurrent() {
        return m_currentSignal.refresh().getValue().in(Amps);
    }

    // setters
    public void motorOff() {
        m_pivotMotor.stopMotor();
    }

    public void setPosition(double value) {
        m_pivotMotor.setPosition(value);
    }

    public void resetPosition() {
        setPosition(0);
    }
    
    public void setVoltage(double value) {
        m_pivotMotor.setControl(new VoltageOut(value));
    }
}