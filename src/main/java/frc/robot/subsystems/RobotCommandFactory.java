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

  // --- SCORE COMMANDS ---

  /**
   * Command that shoots based on distance to hub using vision
   * Simultaneously runs the shooter and snap to angle, then runs column and indexer
  */
  public Command shootByDistanceCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return Commands.parallel(
      m_shooter.shooterVelocityCommand(m_shooterVelocityCalculatedSupplier), // run shooter at velocity  
      m_drivetrainCommandFactory.snapToAngle( // drivetrain: snap to angle 
        joystickValsSupplier,
        () -> HubCalculations.angleToHub(m_drivetrain.getState().Pose)),
      Commands.sequence( // column: 
        m_column.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger() // shooter at target velocity 
            .and(m_drivetrainCommandFactory.atTargetAngleTrigger())), // and drivetrain at target angle
        m_column.velocityCommand(ColumnConstants.COLUMN_VELOCITY)),
      Commands.sequence( // column: 
        m_indexer.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger() // shooter at target velocity
            .and(m_drivetrainCommandFactory.atTargetAngleTrigger())), // and drivetrain at target angle
        m_indexer.velocityCommand(IndexerConstants.INDEXER_VELOCITY)))
    .withName("shootByDistance");
  }

  /**
   * Command that shoots based on distance to hub using vision
   */
  public Command shootByDistanceTestCommand() {
    return m_shooter.shooterVelocityCommand(m_shooterVelocityCalculatedSupplier);
  }

  /**
   * Command that shoots at a given velocity
   * Simultaneously runs the shooter and snap to angle, then runs column and indexer
   */
  public Command shootManualCommand(Supplier<JoystickVals> joystickValsSupplier, double velocity) {
    return Commands.parallel(
      m_shooter.shooterVelocityCommand(velocity), // run shooter at given velocity  
      m_drivetrainCommandFactory.snapToAngle( // drivetrain: snap to angle 
        joystickValsSupplier,
        () -> HubCalculations.angleToHub(m_drivetrain.getState().Pose)),
      Commands.sequence( // column: 
        m_column.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger() // shooter at target velocity 
            .and(m_drivetrainCommandFactory.atTargetAngleTrigger())), // and drivetrain at target angle
        m_column.velocityCommand(ColumnConstants.COLUMN_VELOCITY)),
      Commands.sequence( // indexer: 
        m_indexer.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger() // shooter at target velocity
            .and(m_drivetrainCommandFactory.atTargetAngleTrigger())), // and drivetrain at target angle
        m_indexer.velocityCommand(IndexerConstants.INDEXER_VELOCITY)))
    .withName("shootManual " + velocity);
  }

  /**
   * Command that shoots at a given shooter and column velocity
   * Simultaneously runs the shooter, then runs column and indexer
   */
  public Command shootManualWithoutSnapCommand(double shooterVelocity, double columnVelocity) {
    return Commands.parallel(
      m_shooter.shooterVelocityCommand(shooterVelocity), // run shooter at given velocity  
      Commands.sequence( // column: 
        m_column.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger()), // shooter at target velocity 
        new WaitCommand(1.0),
        m_column.velocityCommand(columnVelocity)),
      Commands.sequence( // indexer: 
        m_indexer.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger()), // shooter at target velocity
        new WaitCommand(1.0), 
        m_indexer.velocityCommand(IndexerConstants.INDEXER_VELOCITY))
    ).withName("shootManualWithoutSnap " + shooterVelocity + " " + columnVelocity);
  }

  /**
   * Command that shoots at a given velocity
   * Simultaneously runs the shooter, then runs column and indexer
   */
  public Command shootManualWithoutSnapCommand(double velocity) {
    return shootManualWithoutSnapCommand(velocity, ColumnConstants.COLUMN_VELOCITY);
  }

  /**
   * Command that shoots with velocity supplier
   * Simultaneously runs the shooter, then runs column and indexer
   */
  public Command shootManualWithoutSnapCommand() {
    return Commands.parallel(
      m_shooter.shooterVelocityCommand(m_shooterVelocitySupplier), // run shooter at given velocity  
      Commands.sequence( // column: 
        m_column.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger()), // shooter at target velocity 
        new WaitCommand(1.0),
        m_column.velocityCommand(ColumnConstants.COLUMN_VELOCITY)),
      Commands.sequence( // indexer: 
        m_indexer.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger()), // shooter at target velocity
        new WaitCommand(1.0), 
        m_indexer.velocityCommand(IndexerConstants.INDEXER_VELOCITY))
    ).withName("shootManualWithSupplier");
  }

  /**
   * Run shooter at given velocity
   */
  public Command shootManualTestCommand(double velocity) {
    return m_shooter.shooterVelocityCommand(velocity);
  }

  // HELPER FUNCTIONS

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

  public Double setShooterVelocity() {
    return SmartDashboard.getNumber("shooter influencer IO/test velocity", 10);
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

}
