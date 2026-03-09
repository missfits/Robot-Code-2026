package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer.JoystickVals;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainCommandFactory;
import frc.robot.subsystems.intake.ColumnSubsystem;
import frc.robot.subsystems.intake.IndexerSubsystem;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.intake.RollerSubsystem;
import frc.robot.subsystems.scorer.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.ShooterLookupTable;
import frc.robot.utils.HubCalculations;

public class RobotCommandFactory {
  // subsystems
  private final CommandSwerveDrivetrain m_drivetrain;
  private final PivotSubsystem m_pivot;
  private final RollerSubsystem m_roller;
  private final IndexerSubsystem m_indexer;
  private final ColumnSubsystem m_column;
  private final ShooterSubsystem m_shooter;
  private final LaserCANSensorBase m_intakeSensor;
  private final LaserCANSensorBase m_shooterSensor;
  private final VisionSubsystem m_vision;
  private final DrivetrainCommandFactory m_drivetrainCommandFactory;

  // velocity suppliers
  private final Supplier<Double> m_pivotDeployVelocitySupplier = () -> setPivotVelocity();
  private final Supplier<Double> m_pivotStoreVelocitySupplier = () -> -setPivotVelocity();
  private final Supplier<Double> m_rollerVelocitySupplier = () -> setRollerVelocity();
  private final Supplier<Double> m_rollerBackVelocitySupplier = () -> -setRollerVelocity();
  private final Supplier<Double> m_indexerVelocitySupplier = () -> setIndexerVelocity();
  private final Supplier<Double> m_indexerBackVelocitySupplier = () -> -setIndexerVelocity();
  private final Supplier<Double> m_columnVelocitySupplier = () -> setColumnVelocity();
  private final Supplier<Double> m_shooterVelocitySupplier = () -> setShooterVelocity(); 
  private final Supplier<Double> m_shooterBackVelocitySupplier = () -> -setShooterVelocity(); 
  private final Supplier<Double> m_shooterVelocityCalculatedSupplier = () -> calculateShooterVelocity(); 

  public RobotCommandFactory(CommandSwerveDrivetrain drivetrain, 
      PivotSubsystem pivot, RollerSubsystem roller, IndexerSubsystem indexer, ColumnSubsystem column, 
      ShooterSubsystem shooter, LaserCANSensorBase intakeSensor, LaserCANSensorBase shooterSensor, 
      VisionSubsystem vision, DrivetrainCommandFactory drivetrainCommandFactory) {
    m_drivetrain = drivetrain;
    m_pivot = pivot;
    m_roller = roller;
    m_indexer = indexer;
    m_column = column;
    m_shooter = shooter;
    m_intakeSensor = intakeSensor;
    m_shooterSensor = shooterSensor;
    m_vision = vision;
    m_drivetrainCommandFactory = drivetrainCommandFactory;
  }

  public void setDefaultCommand() {
    m_pivot.setDefaultCommand(m_pivot.offCommand());
    m_roller.setDefaultCommand(m_roller.offCommand());
    m_indexer.setDefaultCommand(m_indexer.offCommand());
    m_column.setDefaultCommand(m_column.offCommand());
    m_shooter.setDefaultCommand(m_shooter.shooterVelocityCommand(10));
  }

  public Command offCommand() {
    return new ParallelCommandGroup(
      m_pivot.offCommand(),
      m_roller.offCommand(),
      m_indexer.offCommand(),
      m_column.offCommand(),
      m_shooter.shooterVelocityCommand(10)
    );
  }

  public void resetPosition() {
    m_pivot.resetPosition();
    m_roller.resetPosition();
    m_indexer.resetPosition();
    m_column.resetPosition();
    m_shooter.resetPosition();
  }

  // --- INTAKE COMMANDS ---
  // pivot
  public Command deployPivotCommand() {
    return m_pivot.velocityCommand(m_pivotDeployVelocitySupplier).withName("deployPivot");
  }

  public Command storePivotCommand() {
    return m_pivot.velocityCommand(m_pivotStoreVelocitySupplier).withName("storePivot");
  }

  // rollers
  public Command runRollerCommand() {
    return m_roller.velocityCommand(m_rollerVelocitySupplier).withName("runRoller");
  }

  public Command runRollerBackCommand() {
    return m_roller.velocityCommand(m_rollerBackVelocitySupplier).withName("runRollerBack");
  }

  // indexer
  public Command runIndexerCommand() {
    return m_indexer.velocityCommand(m_indexerVelocitySupplier).withName("runIndexer");
  }

  public Command runIndexerBackCommand() {
    return m_indexer.velocityCommand(m_indexerBackVelocitySupplier).withName("runIndexerBack");
  }

  // column
  public Command runColumnCommand() {
    return m_column.velocityCommand(m_columnVelocitySupplier).withName("runColumn");
  }

  public Command runColumnBackCommand() {
    return m_column.velocityCommand(() -> -m_columnVelocitySupplier.get()).withName("runColumnBack");
  }

  // combos
  public Command runIntakeRollersCommand() {
    return Commands.parallel(
      m_roller.velocityCommand(m_rollerVelocitySupplier),
      m_indexer.velocityCommand(m_indexerVelocitySupplier)
    ).withName("runIntakeRollers");
  }

  public Command runIntakeRollersBackCommand() {
    return Commands.parallel(
      m_roller.velocityCommand(() -> -m_rollerVelocitySupplier.get()),
      m_indexer.velocityCommand(() -> -m_indexerVelocitySupplier.get())
    ).withName("runIntakeRollersBack");
  }

  public Command outtakeCommand() {
    return Commands.parallel(
      runIntakeRollersBackCommand(),
      runColumnBackCommand(),
      m_shooter.velocityCommand(ShooterConstants.SHOOTER_RECYCLE_VELOCITY)
    ).withName("outtake");
  }

  public Command recycleFuelCommand() {
    return Commands.parallel(
      runIntakeRollersCommand(),
      m_column.velocityCommand(ColumnConstants.COLUMN_RECYCLE_VELOCITY),
      m_shooter.velocityCommand(ShooterConstants.SHOOTER_RECYCLE_VELOCITY)
    ).withName("recycleFuel");
  }

  public Command intakeCommand() {
    return Commands.parallel(
      deployPivotCommand(),
      recycleFuelCommand()
    ).withName("intake");
  }

  public Command storeIntakeCommand() {
    return Commands.parallel(
      storePivotCommand(),
      m_roller.offCommand(),
      m_indexer.offCommand(),
      m_column.offCommand()
    ).withName("storeIntake");
  }

  public Command runShooterBackCommand() {
    return m_shooter.velocityCommand(m_shooterBackVelocitySupplier);
  }

  // --- SCORE COMMANDS ---

  // with vision and est pose

  /**
   * Command that shoots based on distance to hub using vision
   * Simultaneously runs the shooter and snap to angle, then runs column and indexer
  */
  public Command shootByDistanceCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return Commands.parallel(
      snapToHubCommand(joystickValsSupplier),
      shootToHubCommandWithDisplacement(m_shooterVelocityCalculatedSupplier, m_columnVelocitySupplier, m_indexerVelocitySupplier))
    .withName("shootByDistance");
  }

  /**
   * Command that shoots based on distance to hub using vision
   */
  public Command shootByDistanceTestCommand() {
    return m_shooter.shooterVelocityCommand(m_shooterVelocityCalculatedSupplier);
  }

  /**
   * Command that shoots with shooter, column, indexer velocity supplier
   * Simultaneously runs the shooter, then runs column and indexer **once the drivetrain is at the correct angle**
   * 
   * @param shooterSupplier Supplier for shooter velocity
   * @param columnSupplier Supplier for column velocity
   * @param indexerSupplier Supplier for indexer velocity
   * @return Command that shoots with given velocity suppliers
   */
  public Command shootToHubCommand(Supplier<Double> shooterSupplier, Supplier<Double> columnSupplier, Supplier<Double> indexerSupplier) {

    return Commands.parallel(
      // shooter 
      Commands.sequence(
        new WaitCommand(3) // wait until (timeout after 3 seconds)
          .until(m_drivetrainCommandFactory.atAngleTrigger(() -> HubCalculations.angleToHub(m_drivetrain.getState().Pose))), // facing hub 
        m_shooter.shooterVelocityCommand(shooterSupplier)), // run shooter at given velocity  

      // column 
      Commands.sequence( 
        m_column.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger(shooterSupplier) // shooter at target velocity 
            .and(m_drivetrainCommandFactory.atAngleTrigger(() -> HubCalculations.angleToHub(m_drivetrain.getState().Pose)))), // and facing hub
        m_column.velocityCommand(columnSupplier)),

      // indexer 
      Commands.sequence( // indexer: 
        m_indexer.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger(shooterSupplier) // shooter at target velocity 
            .and(m_drivetrainCommandFactory.atAngleTrigger(() -> HubCalculations.angleToHub(m_drivetrain.getState().Pose)))), // and facing hub
        m_indexer.velocityCommand(indexerSupplier))
    ).withName("shootToHub"); 
  }

  public Command shootToHubCommandWithDisplacement(Supplier<Double> shooterSupplier, Supplier<Double> columnSupplier, Supplier<Double> indexerSupplier) {
    return Commands.parallel(
      shootToHubCommand(shooterSupplier, columnSupplier, indexerSupplier),
      Commands.sequence(
        new WaitCommand(1000) // wait until 
          .until(m_shooter.atTargetVelocityTrigger(shooterSupplier)), // shooter at target velocity
        new WaitCommand(2), // wait 2 seconds for some of the fuel to be shot out 
        m_pivot.repeatingDisplaceFuelCommand())
    ).withName("shootToHubWithDisplacement");
  }

  public Command snapToHubCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return m_drivetrainCommandFactory.snapToAngle( // drivetrain: snap to angle 
      joystickValsSupplier,
      () -> HubCalculations.angleToHub(m_drivetrain.getState().Pose));
  }

  // without vision and est pose

  /**
   * Command that shoots at a given velocity supplier
   * Simultaneously runs the shooter and snap to angle, then runs column and indexer
   */
  public Command shootManualCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return Commands.parallel(
      snapToHubCommand(joystickValsSupplier),
      shootToHubCommandWithDisplacement(m_shooterVelocitySupplier, m_columnVelocitySupplier, m_indexerVelocitySupplier))
    .withName("shootManual");
  }

  /**
   * Command that shoots with shooter, column, indexer velocity supplier
   * Simultaneously runs the shooter, then runs column and indexer
   */
  public Command shootManualWithoutSnapCommand() {
    return shootCommand(m_shooterVelocitySupplier, m_columnVelocitySupplier, m_indexerVelocitySupplier)
      .withName("shootManualWithoutSnap");
  }

  /**
   * Run shooter with shooter velocity supplier
   */
  public Command shootManualTestCommand() {
    return m_shooter.shooterVelocityCommand(m_shooterVelocitySupplier)
      .withName("shootManualTest");
  }

  public Command backupScoreCommand(double velocity) {
    return Commands.parallel(
      m_shooter.velocityCommand(velocity),
      m_roller.velocityCommand(RollerConstants.ROLLER_VELOCITY),
      m_indexer.velocityCommand(IndexerConstants.INDEXER_VELOCITY),
      m_column.velocityCommand(ColumnConstants.COLUMN_VELOCITY)
    );
  }

  /**
   * Command that shoots with shooter, column, indexer velocity supplier
   * Simultaneously runs the shooter, then runs column and indexer
   * 
   * @param shooterSupplier Supplier for shooter velocity
   * @param columnSupplier Supplier for column velocity
   * @param indexerSupplier Supplier for indexer velocity
   * @return Command that shoots with given velocity suppliers
   */
  public Command shootCommand(Supplier<Double> shooterSupplier, Supplier<Double> columnSupplier, Supplier<Double> indexerSupplier) {
    return Commands.parallel(
      m_shooter.shooterVelocityCommand(shooterSupplier), // run shooter at given velocity  
      Commands.sequence( // column: 
        m_column.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger(shooterSupplier)), // shooter at target velocity 
        m_column.velocityCommand(columnSupplier)),
      Commands.sequence( // indexer: 
        m_indexer.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger(shooterSupplier)), // shooter at target velocity
        m_indexer.velocityCommand(indexerSupplier))
    ).withName("shootCommand");
  }

  /**
   * Command that shoots with shooter, column, indexer velocity supplier
   * Simultaneously runs the shooter, then runs column and indexer
   * Runs displace fuel command after shooter is at target velocity
   * 
   * @param shooterSupplier Supplier for shooter velocity
   * @param columnSupplier Supplier for column velocity
   * @param indexerSupplier Supplier for indexer velocity
   * @return Command that shoots with given velocity suppliers
   */
  public Command shootCommandWithDisplacement(Supplier<Double> shooterSupplier, Supplier<Double> columnSupplier, Supplier<Double> indexerSupplier) {
    return Commands.parallel(
      shootCommand(shooterSupplier, columnSupplier, indexerSupplier),
      Commands.sequence(
        new WaitCommand(1000) // wait until 
          .until(m_shooter.atTargetVelocityTrigger(shooterSupplier)), // shooter at target velocity
        new WaitCommand(2), // wait 2 seconds for some of the fuel to be shot out 
        m_pivot.repeatingDisplaceFuelCommand())
    ).withName("shootCommandWithDisplacement");
  }

  /**
   * Command that shoots based on set[mechanism]Velocity()
   * Simultaneously runs the shooter, then runs column and indexer
   * @return Command that shoots with set[mechanism]Velocity()
   */
  public Command shootWithoutDistance() {
    return Commands.parallel(
        m_shooter.shooterBackVoltageCommand().withTimeout(0.25).andThen( 
          m_shooter.shooterVelocityCommand(m_shooterVelocitySupplier)), // run shooter at given velocity  
        Commands.sequence( // column: 
          m_column.offCommand() // wait until 
            .until(m_shooter.atTargetVelocityTrigger(m_shooterVelocitySupplier)), // shooter at target velocity 
          m_column.velocityCommand(m_columnVelocitySupplier)),
        Commands.sequence( // roller: 
          m_roller.offCommand()  // wait until 
            .until(m_shooter.atTargetVelocityTrigger(m_shooterVelocitySupplier)), // shooter at target velocity 
          m_roller.velocityCommand(m_rollerVelocitySupplier)),
        Commands.sequence( // indexer: 
          m_indexer.offCommand() // wait until 
            .until(m_shooter.atTargetVelocityTrigger(m_shooterVelocitySupplier)), // shooter at target velocity
          m_indexer.velocityCommand(m_indexerVelocitySupplier))
    ).withName("shootWithoutDistance");
  }

  // HELPER FUNCTIONS
  private Double setPivotVelocity() {
    return SmartDashboard.getNumber("pivot IO/velocity", 1);
  }

  private Double setRollerVelocity() {
    return SmartDashboard.getNumber("roller IO/velocity", RollerConstants.ROLLER_VELOCITY);
  }

  private Double setIndexerVelocity() {
    return SmartDashboard.getNumber("indexer IO/velocity", IndexerConstants.INDEXER_VELOCITY);
  }

  private Double setColumnVelocity() {
    return SmartDashboard.getNumber("column IO/velocity", ColumnConstants.COLUMN_VELOCITY);
  }

  private Double setShooterVelocity() {
    return SmartDashboard.getNumber("shooter influencer IO/velocity", ShooterConstants.SHOOTER_VELOCITY);
  }

  /** 
   * Calculates the shooter velocity based on the distance to the hub
   */
  private Double calculateShooterVelocity() {
    // Calculate distance from hub
    Pose2d robotPose = m_drivetrain.getState().Pose;
    double distanceToHub = HubCalculations.distanceToHub(robotPose);

    // Look up target velocity from distance
    Optional<Double> velocityOptional = ShooterLookupTable.getVelocityForDistance(distanceToHub);

    if (velocityOptional.isPresent()) {
      return velocityOptional.get();
    } else {
      return 0.0; // TODO: fix, based on robot mode?? 
    }
  }

  public double getDistanceToHub() {
    return HubCalculations.distanceToHub(m_drivetrain.getState().Pose);
  }

  public double getAngleToHub() {
    return HubCalculations.angleToHub(m_drivetrain.getState().Pose).getDegrees();
  }

  public double getTargetShooterVelocity() {
    return m_shooterVelocitySupplier.get();
  }

  public double getCalculatedShooterVelocity() {
    return m_shooterVelocityCalculatedSupplier.get();
  }

}
