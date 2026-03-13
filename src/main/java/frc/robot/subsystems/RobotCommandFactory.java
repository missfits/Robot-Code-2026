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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  public enum RobotMode {
    NEUTRAL,
    INTAKE,
    SCORE
  }

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
  private final Supplier<Double> m_shooterVelocityInitialSupplier = 
    () -> setShooterVelocity() + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY;
  private final Supplier<Double> m_shooterBackVelocitySupplier = () -> -setShooterVelocity(); 
  private final Supplier<Double> m_shooterVelocityCalculatedSupplier = () -> calculateShooterVelocity(); 
  private final Supplier<Double> m_shooterVelocityInitialCalculatedSupplier = 
    () -> calculateShooterVelocity() + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY;

  public RobotMode m_robotMode = RobotMode.NEUTRAL;  

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

  // --- TESTING COMMANDS ---
  // pivot
  public Command deployPivotTestCommand() {
    return m_pivot.velocityCommand(m_pivotDeployVelocitySupplier).withName("deployPivot");
  }

  public Command storePivotTestCommand() {
    return m_pivot.velocityCommand(m_pivotStoreVelocitySupplier).withName("storePivot");
  }

  // rollers
  public Command runRollerTestCommand() {
    return m_roller.velocityCommand(m_rollerVelocitySupplier).withName("runRoller");
  }

  public Command runRollerBackTestCommand() {
    return m_roller.velocityCommand(m_rollerBackVelocitySupplier).withName("runRollerBack");
  }

  // indexer
  public Command runIndexerTestCommand() {
    return m_indexer.velocityCommand(m_indexerVelocitySupplier).withName("runIndexer");
  }

  public Command runIndexerBackTestCommand() {
    return m_indexer.velocityCommand(m_indexerBackVelocitySupplier).withName("runIndexerBack");
  }

  // column
  public Command runColumnTestCommand() {
    return m_column.velocityCommand(m_columnVelocitySupplier).withName("runColumn");
  }

  public Command runColumnBackTestCommand() {
    return m_column.velocityCommand(() -> -m_columnVelocitySupplier.get()).withName("runColumnBack");
  }

  // shooter
  /**
   * Run shooter with shooter velocity supplier
   */
  public Command runShooterTestCommand() {
    return m_shooter.shooterVelocityCommand(m_shooterVelocitySupplier)
      .withName("runShooterTest");
  }

  public Command runShooterBackTestCommand() {
    return m_shooter.velocityCommand(m_shooterBackVelocitySupplier);
  }

  /**
   * Command that shoots at a given velocity supplier
   * Simultaneously runs the shooter and snap to angle, then runs column and indexer
   */
  public Command shootWithoutVisionTestCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return Commands.parallel(
      snapToHubCommand(joystickValsSupplier),
      shootWithoutVisionWithDisplacement(m_shooterVelocityInitialSupplier, m_shooterVelocitySupplier, m_columnVelocitySupplier, m_indexerVelocitySupplier, m_rollerVelocitySupplier))
    .withName("shootWithoutVisionTest");
  }

  /**
   * Command that shoots based on distance to hub using vision
   * Simultaneously runs the shooter and snap to angle, then runs column and indexer
  */
  public Command shootWithVisionTestCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return Commands.parallel(
      snapToHubCommand(joystickValsSupplier),
      shootWithVisionWithDisplacement(m_shooterVelocityInitialCalculatedSupplier, m_shooterVelocityCalculatedSupplier, m_columnVelocitySupplier, m_indexerVelocitySupplier, m_rollerVelocitySupplier))
    .withName("shootWithVisionTest");
  }
  /**
   * Command that shoots based on distance to hub using vision
   */
  public Command shootByDistanceTestCommand() {
    return m_shooter.shooterVelocityCommand(m_shooterVelocityCalculatedSupplier);
  }

  // --- COMPETITION COMMANDS ---

  // - manual commands -
  public Command runRollerIndexerCommand() {
    return Commands.parallel(
      m_roller.velocityCommand(RollerConstants.MANUAL_VELOCITY),
      m_indexer.velocityCommand(IndexerConstants.MANUAL_VELOCITY)
    ).withName("runRollerIndexer");
  }

  public Command runRollerIndexerBackCommand() {
    return Commands.parallel(
      m_roller.velocityCommand(RollerConstants.MANUAL_BACK_VELOCITY),
      m_indexer.velocityCommand(IndexerConstants.MANUAL_BACK_VELOCITY)
    ).withName("runRollerIndexerBack");
  }

  public Command runIndexerColumnCommand() {
    return Commands.parallel(
      m_indexer.velocityCommand(IndexerConstants.MANUAL_VELOCITY),
      m_column.velocityCommand(ColumnConstants.MANUAL_VELOCITY)
    ).withName("runIndexerColumn");
  }

  public Command runIndexerColumnBackCommand() {
    return Commands.parallel(
      m_indexer.velocityCommand(IndexerConstants.MANUAL_BACK_VELOCITY),
      m_column.velocityCommand(ColumnConstants.MANUAL_BACK_VELOCITY)
    ).withName("runIndexerColumnBack");
  }

  public Command runAllRollersCommand() {
    return Commands.parallel(
      m_roller.velocityCommand(RollerConstants.MANUAL_VELOCITY),
      m_indexer.velocityCommand(IndexerConstants.MANUAL_VELOCITY),
      m_column.velocityCommand(ColumnConstants.MANUAL_VELOCITY)
    ).withName("runAllRollers");
  }

  public Command runAllRollersBackCommand() {
    return Commands.parallel(
      m_roller.velocityCommand(RollerConstants.MANUAL_BACK_VELOCITY), 
      m_indexer.velocityCommand(IndexerConstants.MANUAL_BACK_VELOCITY),
      m_column.velocityCommand(ColumnConstants.MANUAL_BACK_VELOCITY)
    ).withName("runAllRollersBack");
  }

  public Command runShooterCloseDistanceCommand() {
    return shootWithoutVisionWithDisplacement(
      () -> ShooterConstants.SHOOTER_DISTANCE1_VELOCITY + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY,
      () -> ShooterConstants.SHOOTER_DISTANCE1_VELOCITY,
      () -> ColumnConstants.SHOOT_VELOCITY,
      () -> IndexerConstants.SHOOT_VELOCITY,
      () -> RollerConstants.SHOOT_VELOCITY
    );
  }

  public Command runShooterMediumDistanceCommand() {
    return shootWithoutVisionWithDisplacement(
      () -> ShooterConstants.SHOOTER_DISTANCE2_VELOCITY + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY,
      () -> ShooterConstants.SHOOTER_DISTANCE2_VELOCITY,
      () -> ColumnConstants.SHOOT_VELOCITY,
      () -> IndexerConstants.SHOOT_VELOCITY,
      () -> RollerConstants.SHOOT_VELOCITY
    );
  }

  public Command runShooterFarDistanceCommand() {
    return shootWithoutVisionWithDisplacement(
      () -> ShooterConstants.SHOOTER_DISTANCE3_VELOCITY + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY,
      () -> ShooterConstants.SHOOTER_DISTANCE3_VELOCITY,
      () -> ColumnConstants.SHOOT_VELOCITY,
      () -> IndexerConstants.SHOOT_VELOCITY,
      () -> RollerConstants.SHOOT_VELOCITY
    );
  }

  public Command backupScoreCommand(double shooterVelocity) {
    return shootWithoutVisionWithDisplacement(
      () -> shooterVelocity + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY,
      () -> shooterVelocity,
      () -> ColumnConstants.SHOOT_VELOCITY,
      () -> IndexerConstants.SHOOT_VELOCITY,
      () -> RollerConstants.SHOOT_VELOCITY
    );
  }

  // - driver commands -

  // neutral mode
  public Command outtakeCommand() {
    return Commands.parallel(
      m_pivot.storePivotCommand(),
      m_roller.velocityCommand(RollerConstants.OUTTAKE_VELOCITY),
      m_indexer.velocityCommand(IndexerConstants.OUTTAKE_VELOCITY),
      m_column.velocityCommand(ColumnConstants.OUTTAKE_VELOCITY),
      m_shooter.voltageCommand(ShooterConstants.OUTTAKE_VOLTAGE)
    ).withName("outtake");
  }

  public Command recycleFuelCommand() {
    return Commands.parallel(
      m_roller.velocityCommand(RollerConstants.RECYCLE_VELOCITY),
      m_indexer.velocityCommand(IndexerConstants.RECYCLE_VELOCITY),
      m_column.velocityCommand(ColumnConstants.RECYCLE_VELOCITY),
      m_shooter.velocityCommand(ShooterConstants.RECYCLE_VELOCITY)
    ).withName("recycleFuel");
  }

  public Command shuttleCommand() {
    return shootWithoutVisionWithDisplacement(
        () -> ShooterConstants.SHUTTLE_VELOCITY + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY,
        () -> ShooterConstants.SHUTTLE_VELOCITY,
        () -> ColumnConstants.SHUTTLE_VELOCITY,
        () -> IndexerConstants.SHUTTLE_VELOCITY,
        () -> RollerConstants.ROLLER_VELOCITY
      ).withName("shuttle");
  }

  public Command neutralModeCommand() {
    return Commands.parallel(
      new InstantCommand(() -> m_robotMode = RobotMode.NEUTRAL),
      m_roller.offCommand(),
      m_indexer.offCommand(),
      m_column.offCommand(),
      m_shooter.shooterVelocityCommand(10)
    ).withName("neutralMode");
  }

  // intake mode
  public Command intakeModeCommand() {
    return Commands.parallel(
      new InstantCommand(() -> m_robotMode = RobotMode.INTAKE),
      m_pivot.deployPivotCommand(),
      m_roller.velocityCommand(RollerConstants.INTAKE_VELOCITY),
      m_indexer.velocityCommand(IndexerConstants.INTAKE_VELOCITY),
      m_column.velocityCommand(ColumnConstants.INTAKE_VELOCITY) // will run backwards
    ).withName("intakeMode");
  }

  // score mode
  public Command scoreModeCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return Commands.parallel(
      new InstantCommand(() -> m_robotMode = RobotMode.SCORE),
      snapToHubThenPointWheelsInXCommand(joystickValsSupplier),
      shootWithVisionWithDisplacement(
        m_shooterVelocityInitialCalculatedSupplier,
        m_shooterVelocityCalculatedSupplier,
        () -> ColumnConstants.SHOOT_VELOCITY,
        () -> IndexerConstants.SHOOT_VELOCITY,
        () -> RollerConstants.ROLLER_VELOCITY))
    .withName("scoreMode");
  }

  // shoot helper commands

  public Command shootWithVisionWithDisplacement(Supplier<Double> initialShooterSupplier, Supplier<Double> shooterSupplier, Supplier<Double> columnSupplier, Supplier<Double> indexerSupplier, Supplier<Double> rollerSupplier) {
    return Commands.parallel(
      shootWithVision(initialShooterSupplier, shooterSupplier, columnSupplier, indexerSupplier, rollerSupplier),
      Commands.sequence(
        new WaitCommand(2), // wait 2 seconds for some of the fuel to be shot out 
        m_pivot.repeatingDisplaceFuelCommand())
    ).withName("shootWithVisionWithDisplacement");
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
  public Command shootWithoutVisionWithDisplacement(Supplier<Double> initialShooterSupplier, Supplier<Double> shooterSupplier, Supplier<Double> columnSupplier, Supplier<Double> indexerSupplier, Supplier<Double> rollerSupplier) {
    return Commands.parallel(
      shootWithoutVision(initialShooterSupplier, shooterSupplier, columnSupplier, indexerSupplier, rollerSupplier),
      Commands.sequence(
        new WaitCommand(2), // wait 2 seconds for some of the fuel to be shot out 
        m_pivot.repeatingDisplaceFuelCommand())
    ).withName("shootWithoutVisionWithDisplacement");
  }

   /**
   * Command that shoots with shooter, column, indexer velocity supplier
   * Simultaneously runs the shooter, then runs column and indexer **once the drivetrain is at the correct angle**
   * 
   * @param initialShooterSupplier Supplier for shooter velocity
   * @param shooterSupplier Supplier for shooter velocity
   * @param columnSupplier Supplier for column velocity
   * @param indexerSupplier Supplier for indexer velocity
   * @param rollerSupplier Supplier for roller velocity
   * @return Command that shoots with given velocity suppliers
   */
  public Command shootWithVision(Supplier<Double> initialShooterSupplier, Supplier<Double> shooterSupplier, Supplier<Double> columnSupplier, Supplier<Double> indexerSupplier, Supplier<Double> rollerSupplier) {
    return Commands.parallel(
      // shooter 
      Commands.sequence(
        new WaitCommand(10000) // wait until (no timeout)
          .until(m_drivetrainCommandFactory.atAngleTrigger(() -> HubCalculations.angleToHub(m_drivetrain.getState().Pose))), // facing hub 
        m_shooter.shooterVelocityCommand(initialShooterSupplier)
          .until(m_shooter.isFuelShot(initialShooterSupplier.get())),
        m_shooter.shooterVelocityCommand(shooterSupplier)),  // run shooter at given velocity  
        
      // column 
      Commands.sequence( 
        m_column.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger(shooterSupplier) // shooter at target velocity 
            .and(m_drivetrainCommandFactory.atAngleTrigger(() -> HubCalculations.angleToHub(m_drivetrain.getState().Pose)))), // and facing hub
        m_column.velocityCommand(columnSupplier)),

      // indexer 
      Commands.sequence(
        m_indexer.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger(shooterSupplier) // shooter at target velocity 
            .and(m_drivetrainCommandFactory.atAngleTrigger(() -> HubCalculations.angleToHub(m_drivetrain.getState().Pose)))), // and facing hub
        m_indexer.velocityCommand(indexerSupplier)),

      // roller
      Commands.sequence(
        m_roller.offCommand()  // wait until 
          .until(m_shooter.atTargetVelocityTrigger(shooterSupplier) // shooter at target velocity 
            .and(m_drivetrainCommandFactory.atAngleTrigger(() -> HubCalculations.angleToHub(m_drivetrain.getState().Pose)))), // and facing hub
        m_roller.velocityCommand(rollerSupplier))
    ).withName("shootWithVision"); 
  }

  /**
   * Command that shoots based on set[mechanism]Velocity()
   * Simultaneously runs the shooter, then runs column, indexer and roller 
   * 
   * @param initialShooterSupplier Supplier for initial shooter velocity
   * @param shooterSupplier Supplier for shooter velocity
   * @param columnSupplier Supplier for column velocity
   * @param indexerSupplier Supplier for indexer velocity
   * @param rollerSupplier Supplier for roller velocity
   * @return Command that shoots with set[mechanism]Velocity()
   */
  public Command shootWithoutVision(Supplier<Double> initialShooterSupplier, Supplier<Double> shooterSupplier, Supplier<Double> columnSupplier, Supplier<Double> indexerSupplier, Supplier<Double> rollerSupplier) {
    return Commands.parallel(
      // shooter 
      Commands.sequence(
        // run shooter at slightly higher velocity until we shoot some fuel  
        m_shooter.shooterVelocityCommand(initialShooterSupplier)
          .until(m_shooter.isFuelShot(initialShooterSupplier.get())),
        m_shooter.shooterVelocityCommand(shooterSupplier)),  // run shooter at given velocity  
      // column 
      Commands.sequence(
        m_column.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger(shooterSupplier)), // shooter at target velocity 
        m_column.velocityCommand(columnSupplier)),
      // indexer
      Commands.sequence(
        m_indexer.offCommand() // wait until 
          .until(m_shooter.atTargetVelocityTrigger(shooterSupplier)), // shooter at target velocity
        m_indexer.velocityCommand(indexerSupplier)),
      // roller
      Commands.sequence(
        m_roller.offCommand()  // wait until 
          .until(m_shooter.atTargetVelocityTrigger(shooterSupplier)), // shooter at target velocity 
        m_roller.velocityCommand(rollerSupplier))
    ).withName("shootWithoutVision");
  }

  public Command snapToHubCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return m_drivetrainCommandFactory.snapToAngle( // drivetrain: snap to angle 
      joystickValsSupplier,
      () -> HubCalculations.angleToHub(m_drivetrain.getState().Pose));
  }
  
  // snaps to hub, then points wheels in x
  // warning: driver cannot drive while this is running
  public Command snapToHubThenPointWheelsInXCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return Commands.sequence(
      snapToHubCommand(joystickValsSupplier).until(m_drivetrainCommandFactory.atAngleTrigger(() -> HubCalculations.angleToHub(m_drivetrain.getState().Pose))),
      m_drivetrainCommandFactory.pointWheelsinX());
  }

  // auto
  public Command autoShootWithVisionCommand() {
    return Commands.parallel(
      snapToHubCommand(() -> new JoystickVals(0, 0)).withName("snapToHubProxied").asProxy(),
      shootWithVisionWithDisplacement(
        m_shooterVelocityInitialCalculatedSupplier,
        m_shooterVelocityCalculatedSupplier,
        () -> ColumnConstants.SHOOT_VELOCITY,
        () -> IndexerConstants.SHOOT_VELOCITY,
        () -> RollerConstants.SHOOT_VELOCITY)
    ).withName("autoShootWithVision");
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

  public Trigger robotModeTrigger(RobotMode robotMode) {
    return new Trigger(() -> m_robotMode == robotMode);
  }

}
