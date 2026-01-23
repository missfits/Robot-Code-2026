package frc.robot.subsystems.scorer;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IndexerConstants;


public class IndexerSubsystem extends SubsystemBase {
  private final ShooterIOHardware m_IO = new ShooterIOHardware(IndexerConstants.MECHANISM_MOTOR_ID);

  public IndexerSubsystem() {
        m_IO.resetPosition();
  }

  public Command runShooter(double speed) {
        return new FunctionalCommand(
            // set voltage in init also
            () -> {m_IO.setVoltage(speed); SmartDashboard.putString("indexer/currentlyRunningCommand", "runIndexer");},
            () -> {m_IO.setVoltage(speed); SmartDashboard.putString("indexer/currentlyRunningCommand", "runIndexer");},
            (interrupted) -> {},
            () -> false,
            this
        ).withName("runIndexer");
    }

  public Command VelocityVoltage(double velocity, boolean enableFOC, double feedForward, int slot, boolean overrideBrakeDurNeutral) {
    VelocityVoltage request = new VelocityVoltage(velocity)
        .withEnableFOC(enableFOC)
        .withFeedForward(feedForward)
        .withSlot(slot)
        .withOverrideBrakeDurNeutral(overrideBrakeDurNeutral);

    return this.run(() -> {
        m_IO.setVelocityVoltage(request);
        SmartDashboard.putNumber("indexer/velocity voltage", velocity);
    });
}

  public Command runShooterOff() {
    return new RunCommand(
      () -> {
        m_IO.setVoltage(0);
        SmartDashboard.putBoolean("indexer/off", true);
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