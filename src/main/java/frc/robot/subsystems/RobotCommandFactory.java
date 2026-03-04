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
  private final Supplier<Double> m_columnVelocitySupplier = () -> setColumnVelocity();
  private final Supplier<Double> m_shooterVelocitySupplier = () -> setShooterVelocity(); 
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
    m_shooter.setDefaultCommand(m_shooter.offCommand());
  }

  public void resetPosition() {
    m_pivot.resetPosition();
    m_roller.resetPosition();
    m_indexer.resetPosition();
    m_column.resetPosition();
    m_shooter.resetPosition();
  }

  // --- INTAKE COMMANDS ---
  public Command deployPivotCommand() {
    return m_pivot.velocityCommand(m_pivotDeployVelocitySupplier).withName("deployPivot");
  }

  public Command storePivotCommand() {
    return m_pivot.velocityCommand(m_pivotStoreVelocitySupplier).withName("storePivot");
  }

  public Command runRollerCommand() {
    return m_roller.velocityCommand(m_rollerVelocitySupplier).withName("runRoller");
  }

  public Command runRollersBackCommand() {
    return m_roller.velocityCommand(m_rollerBackVelocitySupplier).withName("runRollersBack");
  }

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

  public Command runColumnCommand() {
    return m_column.velocityCommand(m_columnVelocitySupplier);
  }

  public Command deployIntake() {
    return Commands.parallel(
      deployPivotCommand(),
      runIntakeRollersCommand()
    ).withName("deployIntake");
  }

  public Command storeIntake() {
    return Commands.parallel(
      storePivotCommand(),
      m_roller.offCommand(),
      m_column.offCommand()
    ).withName("storeIntake");
  }

  // --- SCORE COMMANDS ---

  /**
   * Command that shoots based on distance to hub using vision
   * Simultaneously runs the shooter and snap to angle, then runs column and indexer
  */
  public Command shootByDistanceCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return Commands.parallel(
      snapToHubCommand(joystickValsSupplier),
      shootCommand(m_shooterVelocityCalculatedSupplier, m_indexerVelocitySupplier, m_columnVelocitySupplier))
    .withName("shootByDistance");
  }

  /**
   * Command that shoots based on distance to hub using vision
   */
  public Command shootByDistanceTestCommand() {
    return m_shooter.shooterVelocityCommand(m_shooterVelocityCalculatedSupplier);
  }

  /**
   * Command that shoots at a given velocity supplier
   * Simultaneously runs the shooter and snap to angle, then runs column and indexer
   */
  public Command shootManualCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return Commands.parallel(
      snapToHubCommand(joystickValsSupplier),
      shootCommand(m_shooterVelocitySupplier, m_columnVelocitySupplier, m_indexerVelocitySupplier))
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
        m_indexer.velocityCommand(indexerSupplier)));
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
    return Commands.sequence(
        new WaitCommand(5) // timeout and just shoot after 5 seconds 
          .until(m_drivetrainCommandFactory.atAngleTrigger(() -> HubCalculations.angleToHub(m_drivetrain.getState().Pose))),
        Commands.parallel(
        m_shooter.shooterVelocityCommand(shooterSupplier), // run shooter at given velocity  
        Commands.sequence( // column: 
          m_column.offCommand() // wait until 
            .until(m_shooter.atTargetVelocityTrigger(shooterSupplier)), // shooter at target velocity 
          m_column.velocityCommand(columnSupplier)),
        Commands.sequence( // indexer: 
          m_indexer.offCommand() // wait until 
            .until(m_shooter.atTargetVelocityTrigger(shooterSupplier)), // shooter at target velocity
          m_indexer.velocityCommand(indexerSupplier)))
    );
  }

  public Command snapToHubCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return m_drivetrainCommandFactory.snapToAngle( // drivetrain: snap to angle 
      joystickValsSupplier,
      () -> HubCalculations.angleToHub(m_drivetrain.getState().Pose));
  }

  // HELPER FUNCTIONS
  private Double setPivotVelocity() {
    return SmartDashboard.getNumber("pivot IO/velocity", 1);
  }

  private Double setRollerVelocity() {
    return SmartDashboard.getNumber("roller IO/velocity", 10);
  }

  private Double setIndexerVelocity() {
    return SmartDashboard.getNumber("indexer IO/velocity", 10);
  }

  private Double setColumnVelocity() {
    return SmartDashboard.getNumber("column IO/velocity", 10);
  }

  private Double setShooterVelocity() {
    return SmartDashboard.getNumber("shooter influencer IO/velocity", 10);
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
