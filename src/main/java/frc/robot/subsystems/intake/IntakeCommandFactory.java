package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.LaserCANSensorBase;

public class IntakeCommandFactory {
  private RollerSubsystem m_roller;
  private LaserCANSensorBase m_intakeSensor;
  private PivotSubsystem m_pivot;
  private IndexerSubsystem m_indexer;
  private ColumnSubsystem m_column;

  public IntakeCommandFactory(RollerSubsystem roller, LaserCANSensorBase intakeSensor, PivotSubsystem pivot, IndexerSubsystem indexer, ColumnSubsystem column) {
    m_roller = roller;
    m_intakeSensor = intakeSensor;
    m_pivot = pivot;
    m_indexer = indexer;
    m_column = column;
  }

  // add beam braker sensor to column
  public ParallelCommandGroup runIntakeMode() {
    return new ParallelCommandGroup(
      deployPivot(),
      runRoller(),
      runIndexer(),
      runColumn());
  }

  public ParallelCommandGroup runShooterMode() {
    return new ParallelCommandGroup(
      deployPivot(),
      runIndexer(),
      runColumn());
  }

  public ParallelCommandGroup runNeutralMode() {
    return new ParallelCommandGroup(
      deployPivot());
  }

  public Command deployPivot() {
    return m_pivot.deployIntakeFactory().withName("deploy pivot");
  }

  public Command storePivot() {
    return m_pivot.storeIntakeFactory().withName("store pivot");
  }

  public Command runRoller() {
    return m_roller.runMechanism(RollerConstants.ROLLER_VOLTAGE).withName("run roller");
  }

  public Command runIndexer() {
    return m_indexer.runMechanism(IndexerConstants.INDEXER_VOLTAGE).withName("run indexer");
  }

  public Command runIndexerBack() {
    return m_indexer.runMechanism(-IndexerConstants.INDEXER_VOLTAGE).withName("run indexer");
  }

  public Command runIndexerPID() {
    return m_indexer.runMechanismPID(IndexerConstants.INDEXER_VELOCITY).withName("run indexer");
  }

  public Command runColumn() {
    return m_column.runMechanism(ColumnConstants.COLUMN_VOLTAGE).withName("run column");
  }

  public Command runColumnBack() {
    return m_column.runMechanism(-ColumnConstants.COLUMN_VOLTAGE).withName("run column");
  }

  public Command runColumnPID() {
    return m_column.runMechanismPID(ColumnConstants.COLUMN_VELOCITY).withName("run column");
  }

  public Command runRollerBack() {
    return m_roller.runMechanism(RollerConstants.ROLLER_BACK_VOLTAGE).withName("run roller");
  }

  public Command runRollerWithTimeout() {
    return m_roller.runMechanism(RollerConstants.ROLLER_VOLTAGE)
    .withTimeout(RollerConstants.RUN_INTAKE_TIME)
    .withName("run intake timeout");
  }

  public Command runRollerPID() { 
    return m_roller.runMechanismPID(RollerConstants.ROLLER_VELOCITY)
      .withName("run intake PID");
  }

  public Command intakeOff() {
    return m_roller.runMechanismOff().withName("intake off");
  }

  public void setDefaultCommand() {
    m_roller.setDefaultCommand(m_roller.runMechanismOff());
    m_indexer.setDefaultCommand(m_indexer.runMechanismOff());
    m_column.setDefaultCommand(m_column.runMechanismOff());
    m_pivot.setDefaultCommand(m_pivot.runMechanismOff());
  }
}
