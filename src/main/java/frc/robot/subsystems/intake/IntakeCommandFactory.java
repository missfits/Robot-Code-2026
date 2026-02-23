package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.PivotConstants;
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
      deployPivotCommand(),
      rollerVoltageCommand(),
      indexerVoltageCommand(),
      columnVoltageCommand());
  }

  public ParallelCommandGroup runShooterMode() {
    return new ParallelCommandGroup(
      deployPivotCommand(),
      indexerVoltageCommand(),
      columnVoltageCommand());
  }

  public ParallelCommandGroup runNeutralMode() {
    return new ParallelCommandGroup(
      deployPivotCommand());
  }

  public Command deployPivotCommand() {
    return m_pivot.deployIntakeFactory().withName("deploy pivot");
  }

  public Command storePivotCommand() {
    return m_pivot.storeIntakeFactory().withName("store pivot");
  }

  public Command pivotVelocityCommand() {
    return m_pivot.velocityCommand(PivotConstants.DEPLOY_VELOCITY).withName("run pivot velocity");
  }

  public Command rollerVoltageCommand() {
    return m_roller.voltageCommand(RollerConstants.ROLLER_VOLTAGE).withName("run roller voltage");
  }

  public Command indexerVoltageCommand() {
    return m_indexer.voltageCommand(IndexerConstants.INDEXER_VOLTAGE).withName("run indexer voltage");
  }

  public Command indexerBackVoltageCommand() {
    return m_indexer.voltageCommand(-IndexerConstants.INDEXER_VOLTAGE).withName("run indexer back voltage");
  }

  public Command indexerVelocityCommand() {
    return m_indexer.velocityCommand(IndexerConstants.INDEXER_VELOCITY).withName("run indexer velocity");
  }

  public Command columnVoltageCommand() {
    return m_column.voltageCommand(ColumnConstants.COLUMN_VOLTAGE).withName("run column voltage");
  }

  public Command columnBackVoltageCommand() {
    return m_column.voltageCommand(-ColumnConstants.COLUMN_VOLTAGE).withName("run column back voltage");
  }

  public Command columnVelocityCommand() {
    return m_column.velocityCommand(ColumnConstants.COLUMN_VELOCITY).withName("run column velocity");
  }

  public Command rollerBackVoltageCommand() {
    return m_roller.voltageCommand(RollerConstants.ROLLER_BACK_VOLTAGE).withName("run roller back voltage");
  }

  public Command rollerWithTimeoutVoltageCommand() {
    return m_roller.voltageCommand(RollerConstants.ROLLER_VOLTAGE)
    .withTimeout(RollerConstants.RUN_INTAKE_TIME)
    .withName("run intake voltage timeout");
  }

  public Command rollerVelocityCommand() { 
    return m_roller.velocityCommand(RollerConstants.ROLLER_VELOCITY)
      .withName("run intake velocity");
  }

  public Command intakeOffCommand() {
    return m_roller.offCommand().withName("intake off");
  }

  public void setDefaultCommand() {
    m_roller.setDefaultCommand(m_roller.offCommand());
    m_indexer.setDefaultCommand(m_indexer.offCommand());
    m_column.setDefaultCommand(m_column.offCommand());
    m_pivot.setDefaultCommand(m_pivot.offCommand());
  }
}
