package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ScorerConstants;

public class IndexerSubsystem extends SubsystemBase {
  private final IndexerIOHardware m_IO = new IndexerIOHardware(ScorerConstants.INDEXER_MOTOR_ID);

  public IndexerSubsystem() {
    m_IO.resetPosition();
  }

  public Command runIndexer(double velocity) {
    return this.run(() -> {
        m_IO.setVoltage(velocity);
        SmartDashboard.putNumber("intake/input velocity", velocity);
    });
  }

  public Command runIndexerPID(double velocity) {
    return this.run(() -> {
        m_IO.setVelocityVoltage(velocity);
        SmartDashboard.putNumber("intake/input velocity", velocity);
    });
  }

  public Command runIndexerOff() {
    return new RunCommand(() -> {
        m_IO.setVoltage(0);
        SmartDashboard.putBoolean("intake/off", true);
      },
      this
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("indexer/subsystem", this);
    SmartDashboard.putNumber("indexer/current", m_IO.getCurrent());
  }
}