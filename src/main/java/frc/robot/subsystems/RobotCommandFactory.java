package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants;
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
import lombok.experimental.PackagePrivate;
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
  private final Supplier<Double> m_pivotDeployDashboardSupplier = () -> getPivotVelocityFromDashboard();
  private final Supplier<Double> m_pivotStoreDashboardSupplier = () -> -getPivotVelocityFromDashboard();
  private final Supplier<Double> m_rollerDashboardSupplier = () -> getRollerVelocityFromDashboard();
  private final Supplier<Double> m_rollerBackDashboardySupplier = () -> -getRollerVelocityFromDashboard();
  private final Supplier<Double> m_indexerDashboardSupplier = () -> geIndexerVelocityFromDashboard();
  private final Supplier<Double> m_indexerBackDashboardSupplier = () -> -geIndexerVelocityFromDashboard();
  private final Supplier<Double> m_columnDashboardSupplier = () -> getColumnVelocityFromDashboard();
  private final Supplier<Double> m_shooterDashboardSupplier = () -> getShooterVelocityFromDashboard(); 
  private final Supplier<Double> m_shooterDashboardInitialSupplier = 
    () -> getShooterVelocityFromDashboard() + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY;
  private final Supplier<Double> m_shooterBackVelocitySupplier = () -> -getShooterVelocityFromDashboard(); 
  private final Supplier<Double> m_shooterVelocityCalculatedSupplier = () -> calculateShooterVelocity(); 
  private final Supplier<Double> m_shooterVelocityInitialCalculatedSupplier = 
    () -> calculateShooterVelocity() + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY;
    // For a fixed shooter, SOTF uses the same velocity as stationary - only the angle changes
  private final Supplier<Double> m_shooterVelocitySOTFSupplier = () -> calculateShooterVelocity();
  private final Supplier<Double> m_shooterVelocityInitialSOTFSupplier =
    () -> calculateShooterVelocity() + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY;

  private final Supplier<Rotation2d> m_drivetrainAngleSupplier = () -> calculateShootOnTheFlyAngle();

  // Shuttle suppliers
  private final Supplier<Double> m_shuttleShooterVelocitySupplier = () -> calculateShuttleShooterVelocity();
  private final Supplier<Double> m_shuttleShooterVelocityInitialSupplier =
    () -> calculateShuttleShooterVelocity() + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY;
  private final Supplier<Rotation2d> m_shuttleAngleSupplier = () -> calculateShuttleAngle();

  private final Supplier<Double> m_dynamicShooterVelocitySupplier;
  private final Supplier<Boolean> m_scoreModeSupplier;
  private final Trigger m_readyToShootTrigger;

  // Latch for shooter velocity - stays true once shooter reaches velocity until conditions reset it
  private boolean m_shooterVelocityLatch = false;

  // Getters for suppliers (used in RobotContainer for trigger bindings)
  public Supplier<Double> getShooterVelocityCalculatedSupplier() {
    return m_shooterVelocityCalculatedSupplier;
  }

  public RobotCommandFactory(CommandSwerveDrivetrain drivetrain,
      PivotSubsystem pivot, RollerSubsystem roller, IndexerSubsystem indexer, ColumnSubsystem column,
      ShooterSubsystem shooter, LaserCANSensorBase intakeSensor, LaserCANSensorBase shooterSensor,
      VisionSubsystem vision, DrivetrainCommandFactory drivetrainCommandFactory,
      Supplier<Boolean> scoreModeSupplier) {
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
    m_scoreModeSupplier = scoreModeSupplier;

    m_dynamicShooterVelocitySupplier = () -> {
      boolean columnAtVelocity = m_column.isMotorVelocityOverPercentToleranceTrigger(
          () -> ColumnConstants.SHOOT_VELOCITY).getAsBoolean();
      return columnAtVelocity
          ? m_shooterVelocityCalculatedSupplier.get()
          : m_shooterVelocityInitialCalculatedSupplier.get();
    };

    m_readyToShootTrigger = readyToShootTrigger();
  }

  public void setDefaultCommand() {
    m_pivot.setDefaultCommand(m_pivot.offCommand());
    m_roller.setDefaultCommand(m_roller.offCommand());
    m_indexer.setDefaultCommand(m_indexer.offCommand());
    m_column.setDefaultCommand(m_column.offCommand());
    m_shooter.setDefaultCommand(m_shooter.offCommand());
  }

  public Command offCommand() {
    return new ParallelCommandGroup(
      m_pivot.offCommand(),
      m_roller.offCommand(),
      m_indexer.offCommand(),
      m_column.offCommand(),
      m_shooter.offCommand()
    ).withName("offCommand");
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
    return m_pivot.velocityCommand(m_pivotDeployDashboardSupplier).withName("deployPivotTestCommand");
  }

  public Command storePivotTestCommand() {
    return m_pivot.velocityCommand(m_pivotStoreDashboardSupplier).withName("storePivotTestCommand");
  }

  // rollers
  public Command runRollerTestCommand() {
    return m_roller.velocityCommand(m_rollerDashboardSupplier).withName("runRollerTestCommand");
  }

  public Command runRollerBackTestCommand() {
    return m_roller.velocityCommand(m_rollerBackDashboardySupplier).withName("runRollerBackTestCommand");
  }

  // indexer
  public Command runIndexerTestCommand() {
    return m_indexer.velocityCommand(m_indexerDashboardSupplier).withName("runIndexerTestCommand");
  }

  public Command runIndexerBackTestCommand() {
    return m_indexer.velocityCommand(m_indexerBackDashboardSupplier).withName("runIndexerBackTestCommand");
  }

  // column
  public Command runColumnTestCommand() {
    return m_column.velocityCommand(m_columnDashboardSupplier).withName("runColumnTestCommand");
  }

  public Command runColumnBackTestCommand() {
    return m_column.velocityCommand(() -> -m_columnDashboardSupplier.get()).withName("runColumnBackTestCommand");
  }

  // shooter
  /**
   * Run shooter with shooter velocity supplier
   */
  public Command runShooterTestCommand() {
    return m_shooter.shooterVelocityCommand(m_shooterDashboardSupplier)
      .withName("runShooterTestCommand");
  }

  public Command runShooterBackTestCommand() {
    return m_shooter.shooterVelocityCommand(m_shooterBackVelocitySupplier)
      .withName("runShooterBackTestCommand");
  }

  /**
   * Command that shoots at a given velocity supplier
   * Simultaneously runs the shooter and snap to angle, then runs column and indexer
   */
  public Command shootWithoutVisionTestCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return Commands.parallel(
      snapToHubCommand(joystickValsSupplier),
      shootWithoutVisionWithDisplacement(m_shooterDashboardInitialSupplier, m_shooterDashboardSupplier, m_columnDashboardSupplier, m_indexerDashboardSupplier, m_rollerDashboardSupplier))
    .withName("shootWithoutVisionTestCommand");
  }

  /**
   * Command that shoots based on distance to hub using vision (ORIGINAL VERSION)
   * Simultaneously runs the shooter and snap to angle, then runs column and indexer
  */
  public Command shootWithVisionTestCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return Commands.parallel(
      snapToHubCommand(joystickValsSupplier),
      shootWithVisionWithDisplacement(m_shooterVelocityInitialCalculatedSupplier, m_shooterVelocityCalculatedSupplier, m_columnDashboardSupplier, m_indexerDashboardSupplier, m_rollerDashboardSupplier))
    .withName("shootWithVisionTestCommand");
  }

  /**
   * Command that shoots based on distance to hub using vision (DYNAMIC VERSION)
   * Simultaneously runs the shooter and snap to angle, then runs column and indexer
  */
  public Command shootWithVisionDynamicTestCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return Commands.parallel(
      snapToHubCommand(joystickValsSupplier),
      shootWithVisionDynamicWithDisplacement(m_shooterVelocityInitialCalculatedSupplier, m_shooterVelocityCalculatedSupplier, m_columnDashboardSupplier, m_indexerDashboardSupplier, m_rollerDashboardSupplier))
    .withName("shootWithVisionDynamicTest");
  }

  /**
   * Command that shoots based on distance to hub using vision
   */
  public Command shootByDistanceTestCommand() {
    return m_shooter.shooterVelocityCommand(m_shooterVelocityCalculatedSupplier)
      .withName("shootByDistanceTestCommand");
  }

  // --- COMPETITION COMMANDS ---

  // - manual commands -
  public Command runRollerIndexerCommand() {
    return Commands.parallel(
      m_roller.velocityCommand(RollerConstants.MANUAL_VELOCITY),
      m_indexer.velocityCommand(IndexerConstants.MANUAL_VELOCITY)
    ).withName("runRollerIndexerCommand");
  }

  public Command runRollerIndexerBackCommand() {
    return Commands.parallel(
      m_roller.velocityCommand(RollerConstants.MANUAL_BACK_VELOCITY),
      m_indexer.velocityCommand(IndexerConstants.MANUAL_BACK_VELOCITY)
    ).withName("runRollerIndexerBackCommand");
  }

  public Command runIndexerColumnCommand() {
    return Commands.parallel(
      m_indexer.velocityCommand(IndexerConstants.MANUAL_VELOCITY),
      m_column.velocityCommand(ColumnConstants.MANUAL_VELOCITY)
    ).withName("runIndexerColumnCommand");
  }

  public Command runIndexerColumnBackCommand() {
    return Commands.parallel(
      m_indexer.velocityCommand(IndexerConstants.MANUAL_BACK_VELOCITY),
      m_column.velocityCommand(ColumnConstants.MANUAL_BACK_VELOCITY)
    ).withName("runIndexerColumnBackCommand");
  }

  public Command runAllRollersCommand() {
    return Commands.parallel(
      m_roller.velocityCommand(RollerConstants.MANUAL_VELOCITY),
      m_indexer.velocityCommand(IndexerConstants.MANUAL_VELOCITY),
      m_column.velocityCommand(ColumnConstants.MANUAL_VELOCITY)
    ).withName("runAllRollersCommand");
  }

  public Command runAllRollersBackCommand() {
    return Commands.parallel(
      m_roller.velocityCommand(RollerConstants.MANUAL_BACK_VELOCITY), 
      m_indexer.velocityCommand(IndexerConstants.MANUAL_BACK_VELOCITY),
      m_column.velocityCommand(ColumnConstants.MANUAL_BACK_VELOCITY)
    ).withName("runAllRollersBackCommand");
  }

  public Command runShooterCloseDistanceCommand() {
    return shootWithoutVisionWithDisplacement(
      () -> ShooterConstants.SHOOTER_DISTANCE1_VELOCITY + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY,
      () -> ShooterConstants.SHOOTER_DISTANCE1_VELOCITY,
      () -> ColumnConstants.SHOOT_VELOCITY,
      () -> IndexerConstants.SHOOT_VELOCITY,
      () -> RollerConstants.SHOOT_VELOCITY
    ).withName("runShooterCloseDistanceCommand");
  }

  public Command runShooterMediumDistanceCommand() {
    return shootWithoutVisionWithDisplacement(
      () -> ShooterConstants.SHOOTER_DISTANCE2_VELOCITY + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY,
      () -> ShooterConstants.SHOOTER_DISTANCE2_VELOCITY,
      () -> ColumnConstants.SHOOT_VELOCITY,
      () -> IndexerConstants.SHOOT_VELOCITY,
      () -> RollerConstants.SHOOT_VELOCITY
    ).withName("runShooterMediumDistanceCommand");
  }

  public Command runShooterFarDistanceCommand() {
    return shootWithoutVisionWithDisplacement(
      () -> ShooterConstants.SHOOTER_DISTANCE3_VELOCITY + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY,
      () -> ShooterConstants.SHOOTER_DISTANCE3_VELOCITY,
      () -> ColumnConstants.SHOOT_VELOCITY,
      () -> IndexerConstants.SHOOT_VELOCITY,
      () -> RollerConstants.SHOOT_VELOCITY
    ).withName("runShooterFarDistanceCommand");
  }

  public Command dashboardScoreCommand() {
    return shootWithoutVisionWithDisplacement(
      () -> getShooterVelocityFromDashboard() + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY,
      () -> getShooterVelocityFromDashboard(),
      () -> ColumnConstants.SHOOT_VELOCITY,
      () -> IndexerConstants.SHOOT_VELOCITY,
      () -> RollerConstants.SHOOT_VELOCITY
    ).withName("dashboardScoreCommand");
  }

  public Command backupScoreCommand(double shooterVelocity) {
    return shootWithoutVisionWithDisplacement(
      () -> shooterVelocity + ShooterConstants.INTIAL_ADDITIONAL_VELOCITY,
      () -> shooterVelocity,
      () -> ColumnConstants.SHOOT_VELOCITY,
      () -> IndexerConstants.SHOOT_VELOCITY,
      () -> RollerConstants.SHOOT_VELOCITY
    ).withName("backupScoreCommand");
  }

  // - driver commands -

  // neutral mode
  public Command outtakeCommand() {
    return Commands.parallel(
      m_roller.velocityCommand(RollerConstants.OUTTAKE_VELOCITY),
      m_indexer.velocityCommand(IndexerConstants.OUTTAKE_VELOCITY),
      m_column.velocityCommand(ColumnConstants.OUTTAKE_VELOCITY),
      m_shooter.voltageCommand(ShooterConstants.OUTTAKE_VOLTAGE)
    ).withName("outtakeCommand");
  }

  public Command recycleFuelCommand() {
    return Commands.parallel(
      m_roller.velocityCommand(RollerConstants.RECYCLE_VELOCITY),
      m_indexer.velocityCommand(IndexerConstants.RECYCLE_VELOCITY),
      m_column.velocityCommand(ColumnConstants.RECYCLE_VELOCITY),
      m_shooter.velocityCommand(ShooterConstants.RECYCLE_VELOCITY)
    ).withName("recycleFuelCommand");
  }

  public Command shuttleCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return Commands.parallel(
      // Snap to shuttle target corner while allowing driver translation
      snapToShuttleTargetCommand(joystickValsSupplier),
      // Run shooter and feeding mechanisms with dynamic velocity based on distance
      // Only feed when at the correct angle
      shootForShuttle(
        m_shuttleShooterVelocityInitialSupplier,
        m_shuttleShooterVelocitySupplier,
        () -> ColumnConstants.SHUTTLE_VELOCITY,
        () -> IndexerConstants.SHUTTLE_VELOCITY,
        () -> RollerConstants.SHUTTLE_VELOCITY
      )
    ).withName("shuttleCommand");
  }

  public Command snapToShuttleTargetCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return m_drivetrainCommandFactory.snapToAngle(
      joystickValsSupplier,
      m_shuttleAngleSupplier
    ).withName("snapToShuttleTargetCommand");
  }

  public Command neutralModeCommand() {
    return Commands.parallel(
      m_roller.offCommand(),
      m_indexer.offCommand(),
      m_column.offCommand(),
      m_shooter.offCommand()
    ).withName("neutralModeCommand");
  }

  // intake mode
  public Command intakeModeCommand() {
    return Commands.parallel(
      m_pivot.deployPivotCommand(),
      m_roller.velocityCommand(RollerConstants.INTAKE_VELOCITY),
      m_indexer.velocityCommand(IndexerConstants.INTAKE_VELOCITY),
      m_column.velocityCommand(ColumnConstants.INTAKE_VELOCITY) // will run backwards
    ).withName("intakeModeCommand");
  }

  public Command autoIntakeModeCommand() {
    return Commands.parallel(
      intakeModeCommand(),
      m_pivot.repeatingZeroPivotCommand()
    ).withName("autoIntakeModeCommand");
  }

  // score mode

  /**
   * ORIGINAL VERSION: Score mode with simple sequences
   */
  public Command scoreModeCommand(Supplier<JoystickVals> joystickValsSupplier, Trigger driverInputTrigger) {
    return Commands.parallel(
      snapToHubThenPointWheelsInXCommand(joystickValsSupplier, driverInputTrigger),
      shootWithVisionDynamicWithDisplacement(
        m_shooterVelocityInitialCalculatedSupplier,
        m_shooterVelocityCalculatedSupplier,
        () -> ColumnConstants.SHOOT_VELOCITY,
        () -> IndexerConstants.SHOOT_VELOCITY,
        () -> RollerConstants.ROLLER_VELOCITY))
    .withName("scoreModeCommand");
  }

  /**
   * DYNAMIC VERSION: Score mode with dynamic stop/start feeding
   */
  public Command scoreModeDynamicCommand(Supplier<JoystickVals> joystickValsSupplier, Trigger driverInputTrigger) {
    return Commands.parallel(
      snapToHubThenPointWheelsInXCommand(joystickValsSupplier, driverInputTrigger),
      shootWithVisionDynamicWithDisplacement(
        m_shooterVelocityInitialCalculatedSupplier,
        m_shooterVelocityCalculatedSupplier,
        () -> ColumnConstants.SHOOT_VELOCITY,
        () -> IndexerConstants.SHOOT_VELOCITY,
        () -> RollerConstants.ROLLER_VELOCITY))
    .withName("scoreModeDynamic");
  }

  /**
   * Aims the drivetrain at the hub and spins up the shooter.
   * Shooter velocity automatically adjusts based on column motor state:
   * - Uses initial (higher) velocity when column is not at speed
   * - Switches to target velocity when column reaches speed
   *
   * This command should be bound to a button press in RobotContainer.
   * Then use readyToShootTrigger().whileTrue() to feed gamepieces.
   *
   * @param joystickValsSupplier Supplier for driver translation joystick input
   * @param driverInputTrigger Trigger that detects driver input for wheel unlocking
   * @return Command that aims and spins up shooter
   */
  public Command aimAndSpinUpShooterCommand(Supplier<JoystickVals> joystickValsSupplier, Trigger driverInputTrigger) {

    return Commands.parallel(
      // Aim at hub, then lock wheels in X when aligned
      snapToHubThenPointWheelsInXCommand(joystickValsSupplier, driverInputTrigger),

      // Spin up shooter with dynamic velocity
      m_shooter.shooterVelocityCommand(m_dynamicShooterVelocitySupplier)
    ).withName("aimAndSpinUpShooter");
  }

  public Command spinUpShooterCommand() {
    return m_shooter.shooterVelocityCommand(m_dynamicShooterVelocitySupplier).withName("spinUpShooterCommand");
  }

  /**
   * Feeds gamepieces through all mechanisms (column, indexer, roller, pivot).
   * This command should be bound to readyToShootTrigger().whileTrue() in RobotContainer.
   *
   * @return Command that runs all feeding mechanisms
   */
  public Command feedGamepieceCommand() {
    return Commands.parallel(
      m_column.velocityCommand(() -> ColumnConstants.SHOOT_VELOCITY),
      Commands.sequence(
        new WaitCommand(2).until(m_column.isMotorVelocityOverPercentToleranceTrigger(() -> ColumnConstants.SHOOT_VELOCITY)), 
        m_indexer.velocityCommand(() -> IndexerConstants.SHOOT_VELOCITY)),
      Commands.sequence(
        new WaitCommand(2).until(m_column.isMotorVelocityOverPercentToleranceTrigger(() -> ColumnConstants.SHOOT_VELOCITY)), 
        m_roller.velocityCommand(() -> RollerConstants.SHOOT_VELOCITY)),
      Commands.sequence(
        new WaitCommand(2).until(m_column.isMotorVelocityOverPercentToleranceTrigger(() -> ColumnConstants.SHOOT_VELOCITY)), 
        m_pivot.repeatingDisplaceFuelCommand())
    ).withName("feedGamepiece");
  }

  // shoot helper commands

  /**
   * ORIGINAL VERSION: Wraps shootWithVision (simple sequences) with displacement
   */
  public Command shootWithVisionWithDisplacement(Supplier<Double> initialShooterSupplier, Supplier<Double> shooterSupplier, Supplier<Double> columnSupplier, Supplier<Double> indexerSupplier, Supplier<Double> rollerSupplier) {
    return Commands.parallel(
      shootWithVision(initialShooterSupplier, shooterSupplier, columnSupplier, indexerSupplier, rollerSupplier),
      Commands.sequence(
        new WaitCommand(1), // wait 1 second for some of the fuel to be shot out 
        m_pivot.repeatingDisplaceFuelCommand())
    ).withName("shootWithVisionWithDisplacement");
  }

  /**
   * DYNAMIC VERSION: Wraps shootWithVisionDynamic (repeating sequences) with displacement
   */
  public Command shootWithVisionDynamicWithDisplacement(Supplier<Double> initialShooterSupplier, Supplier<Double> shooterSupplier, Supplier<Double> columnSupplier, Supplier<Double> indexerSupplier, Supplier<Double> rollerSupplier) {
    return Commands.parallel(
      shootWithVisionDynamic(initialShooterSupplier, shooterSupplier, columnSupplier, indexerSupplier, rollerSupplier),
      Commands.sequence(
        new WaitCommand(1), // wait 1 second for some of the fuel to be shot out
        m_pivot.repeatingDisplaceFuelCommand())
    ).withName("shootWithVisionDynamicWithDisplacement");
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
        new WaitCommand(1), // wait 1 second for some of the fuel to be shot out 
        m_pivot.repeatingDisplaceFuelCommand())
    ).withName("shootWithoutVisionWithDisplacement");
  }

   /**
   * Command that shoots with shooter, column, indexer velocity supplier
   * Simultaneously runs the shooter, then runs column and indexer **once the drivetrain is at the correct angle**
   * DYNAMIC VERSION: Uses repeating sequences to dynamically stop/start feeding based on conditions
   *
   * @param initialShooterSupplier Supplier for shooter velocity
   * @param shooterSupplier Supplier for shooter velocity
   * @param columnSupplier Supplier for column velocity
   * @param indexerSupplier Supplier for indexer velocity
   * @param rollerSupplier Supplier for roller velocity
   * @return Command that shoots with given velocity suppliers
   */
  public Command shootWithVisionDynamic(Supplier<Double> initialShooterSupplier, Supplier<Double> shooterSupplier, Supplier<Double> columnSupplier, Supplier<Double> indexerSupplier, Supplier<Double> rollerSupplier) {
    return Commands.parallel(

      // log isFuelShot
      Commands.run(() -> {
        SmartDashboard.putBoolean("robot command factory/isColumnHappy", m_column.isMotorVelocityOverPercentToleranceTrigger(columnSupplier).getAsBoolean());
        SmartDashboard.putBoolean("robot command factory/atAngleTrigger", m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier).getAsBoolean());
        SmartDashboard.putBoolean("robot command factory/isMotorVelocityWithinPercentTolerance", m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier).getAsBoolean());
      }),

      // shooter 
      Commands.sequence(
        m_shooter.shooterVelocityCommand(initialShooterSupplier)
          .until(m_column.isMotorVelocityOverPercentToleranceTrigger(columnSupplier))
          .withTimeout(ShooterConstants.FUEL_SHOT_TIMEOUT),
        m_shooter.shooterVelocityCommand(shooterSupplier)),  // run shooter at given velocity

      // column
      Commands.repeatingSequence(
        m_column.offCommand() // wait until
          .until(m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier) // shooter at target velocity
            .and(m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier))) // and facing hub
          .withTimeout(ShooterConstants.WAIT_FOR_SHOOTER_TIMEOUT),
        m_column.velocityCommand(columnSupplier) // shoot until
          .until(not(m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier)) // shooter not at target velocity
            .or(not(m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier))))), // or not facing hub

      // indexer
      Commands.repeatingSequence(
        m_indexer.offCommand() // wait until
          .until(m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier) // shooter at target velocity
            .and(m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier))) // and facing hub
          .withTimeout(ShooterConstants.WAIT_FOR_SHOOTER_TIMEOUT),
        m_indexer.velocityCommand(indexerSupplier)
          .until(not(m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier)) // shooter not at target velocity
            .or(not(m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier))))), // or not facing hub

      // roller
      Commands.repeatingSequence(
        m_roller.offCommand()  // wait until
          .until(m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier) // shooter at target velocity
            .and(m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier))) // and facing hub
          .withTimeout(ShooterConstants.WAIT_FOR_SHOOTER_TIMEOUT),
         m_roller.velocityCommand(rollerSupplier)
          .until(not(m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier)) // shooter not at target velocity
            .or(not(m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier))))) // or not facing hub
    ).withName("shootWithVisionDynamic");
  }

   /**
   * Command that shoots with shooter, column, indexer velocity supplier
   * Simultaneously runs the shooter, then runs column and indexer **once the drivetrain is at the correct angle**
   * ORIGINAL VERSION: Uses simple sequences - column/indexer/roller run continuously after conditions met
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

      // log isFuelShot
      Commands.run(() -> {
        SmartDashboard.putBoolean("robotCommandFactory/isColumnHappy", m_column.isMotorVelocityOverPercentToleranceTrigger(columnSupplier).getAsBoolean());
        SmartDashboard.putBoolean("robotCommandFactory/atAngleTrigger", m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier).getAsBoolean());
        SmartDashboard.putBoolean("robotCommandFactory/isMotorVelocityWithinPercentTolerance", m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier).getAsBoolean());
      }),

      // shooter
      Commands.sequence(
        m_shooter.shooterVelocityCommand(initialShooterSupplier)
          .until(m_column.isMotorVelocityOverPercentToleranceTrigger(columnSupplier))
          .withTimeout(ShooterConstants.FUEL_SHOT_TIMEOUT),
        m_shooter.shooterVelocityCommand(shooterSupplier)),  // run shooter at given velocity

      // column
      Commands.sequence(
        m_column.offCommand() // wait until
          .until(m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier) // shooter at target velocity
            .and(m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier))) // and facing hub
          .withTimeout(ShooterConstants.WAIT_FOR_SHOOTER_TIMEOUT),
        m_column.velocityCommand(columnSupplier)),

      // indexer
      Commands.sequence(
        m_indexer.offCommand() // wait until
          .until(m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier) // shooter at target velocity
            .and(m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier))) // and facing hub
          .withTimeout(ShooterConstants.WAIT_FOR_SHOOTER_TIMEOUT),
        m_indexer.velocityCommand(indexerSupplier)),

      // roller
      Commands.sequence(
        m_roller.offCommand()  // wait until
          .until(m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier) // shooter at target velocity
            .and(m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier))) // and facing hub
          .withTimeout(ShooterConstants.WAIT_FOR_SHOOTER_TIMEOUT),
         m_roller.velocityCommand(rollerSupplier))
    ).withName("shootWithVision");
  }

  /**
   * Command for shuttle shooting - waits for correct angle before feeding
   * Simultaneously runs the shooter, then runs column **once the drivetrain is at the shuttle angle**
   * Note that the indexer and roller run at all times so intaking at the same time is possible
   *
   * @param initialShooterSupplier Supplier for initial shooter velocity
   * @param shooterSupplier Supplier for shooter velocity
   * @param columnSupplier Supplier for column velocity
   * @param indexerSupplier Supplier for indexer velocity
   * @param rollerSupplier Supplier for roller velocity
   * @return Command that shoots for shuttle with given velocity suppliers
   */
  public Command shootForShuttle(Supplier<Double> initialShooterSupplier, Supplier<Double> shooterSupplier, Supplier<Double> columnSupplier, Supplier<Double> indexerSupplier, Supplier<Double> rollerSupplier) {
    return Commands.parallel(

      // log isFuelShot
      Commands.run(() -> {
        SmartDashboard.putBoolean("robotCommandFactory/isColumnHappy", m_column.isMotorVelocityOverPercentToleranceTrigger(columnSupplier).getAsBoolean());
        SmartDashboard.putBoolean("robotCommandFactory/atShuttleAngleTrigger", m_drivetrainCommandFactory.atAngleTrigger(m_shuttleAngleSupplier).getAsBoolean());
        SmartDashboard.putBoolean("robotCommandFactory/isMotorVelocityWithinPercentTolerance", m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier).getAsBoolean());
      }),

      // shooter
      Commands.sequence(
        m_shooter.shooterVelocityCommand(initialShooterSupplier)
          .until(m_column.isMotorVelocityOverPercentToleranceTrigger(columnSupplier))
          .withTimeout(ShooterConstants.FUEL_SHOT_TIMEOUT),
        m_shooter.shooterVelocityCommand(shooterSupplier)),  // run shooter at given velocity

      // column
      Commands.sequence(
        m_column.offCommand() // wait until
          .until(m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier) // shooter at target velocity
            .and(m_drivetrainCommandFactory.atAngleTrigger(m_shuttleAngleSupplier))) // and facing shuttle corner
          .withTimeout(ShooterConstants.WAIT_FOR_SHOOTER_TIMEOUT),
        m_column.velocityCommand(columnSupplier)),

      // indexer -- always run 
      m_indexer.velocityCommand(indexerSupplier),

      // roller -- always run 
      m_roller.velocityCommand(rollerSupplier)
    ).withName("shootForShuttle");
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
      // log isFuelShot
      Commands.run(() -> {
        SmartDashboard.putBoolean("robotCommandFactory/isColumnHappy", m_column.isMotorVelocityOverPercentToleranceTrigger(columnSupplier).getAsBoolean());
        SmartDashboard.putBoolean("robotCommandFactory/isMotorVelocityWithinPercentTolerance", m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier).getAsBoolean());
      }),

      // shooter 
      Commands.sequence(
        m_shooter.shooterVelocityCommand(initialShooterSupplier)
          .until(m_column.isMotorVelocityOverPercentToleranceTrigger(columnSupplier))
          .withTimeout(ShooterConstants.FUEL_SHOT_TIMEOUT),
        m_shooter.shooterVelocityCommand(shooterSupplier)),  // run shooter at given velocity  
        
      // column 
      Commands.sequence(
        m_column.offCommand() // wait until 
          .until(m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier)) // shooter at target velocity 
          .withTimeout(ShooterConstants.WAIT_FOR_SHOOTER_TIMEOUT),
        m_column.velocityCommand(columnSupplier)),
      // indexer
      Commands.sequence(
        m_indexer.offCommand() // wait until 
          .until(m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier)) // shooter at target velocity
          .withTimeout(ShooterConstants.WAIT_FOR_SHOOTER_TIMEOUT),
        m_indexer.velocityCommand(indexerSupplier)),
      // roller
      Commands.sequence(
        m_roller.offCommand()  // wait until 
          .until(m_shooter.isMotorVelocityWithinPercentTolerance(shooterSupplier)) // shooter at target velocity 
          .withTimeout(ShooterConstants.WAIT_FOR_SHOOTER_TIMEOUT),
        m_roller.velocityCommand(rollerSupplier))
    ).withName("shootWithoutVision");
  }

  public Command snapToHubCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return m_drivetrainCommandFactory.snapToAngle( // drivetrain: snap to angle 
      joystickValsSupplier,
      m_drivetrainAngleSupplier).withName("snapToHubCommand");
  }
  
  // snaps to hub, then points wheels in x
  // warning: driver cannot drive while this is running
  public Command snapToHubThenPointWheelsInXCommand(Supplier<JoystickVals> joystickValsSupplier, Trigger driverInputTrigger) {
    return Commands.repeatingSequence(
      snapToHubCommand(joystickValsSupplier) // snap to hub until at angle 
        .until(m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier)),
      m_drivetrainCommandFactory.pointWheelsinX() // snap to hub until not at angle or driver input
        .until(not(m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier))
        .or(driverInputTrigger))).withName("snapToHubThenPointWheelsInXCommand");
  }

  // auto

  /**
   * ORIGINAL VERSION: Auto shoot with vision using simple sequences
   */
  public Command autoShootWithVisionCommand() {
    return Commands.parallel(
      shootWithVisionWithDisplacement(
        m_shooterVelocityInitialCalculatedSupplier,
        m_shooterVelocityCalculatedSupplier,
        () -> ColumnConstants.SHOOT_VELOCITY,
        () -> IndexerConstants.SHOOT_VELOCITY,
        () -> RollerConstants.SHOOT_VELOCITY)
    ).withName("autoShootWithVisionCommand");
  }

  /**
   * DYNAMIC VERSION: Auto shoot with vision using dynamic stop/start feeding
   */
  public Command autoShootWithVisionDynamicCommand() {
    return Commands.parallel(
      shootWithVisionDynamicWithDisplacement(
        m_shooterVelocityInitialCalculatedSupplier,
        m_shooterVelocityCalculatedSupplier,
        () -> ColumnConstants.SHOOT_VELOCITY,
        () -> IndexerConstants.SHOOT_VELOCITY,
        () -> RollerConstants.SHOOT_VELOCITY)
    ).withName("autoShootWithVisionDynamic");
  }

  /**
   * Creates a wait command with a variable duration determined by a DoubleSupplier.
   * The duration is evaluated when the command is scheduled.
   *
   * @param durationSupplier Supplier that provides the wait duration in seconds
   * @return Command that waits for the supplied duration
   */
  public Command variableWaitCommand(DoubleSupplier durationSupplier) {
    return Commands.defer(
      () -> new WaitCommand(durationSupplier.getAsDouble()),
      java.util.Set.of()
    ).withName("variableWaitCommand");

  }
  
  public Command autoZeroPivotCommand() {
    return Commands.parallel(
      Commands.sequence(
        new WaitCommand(0.5),
        m_pivot.zeroPivotCommand()),
      m_roller.velocityCommand(-10).withTimeout(1)
    ).withName("autoZeroPivot"); 
  }

  // HELPER FUNCTIONS
  private Double getPivotVelocityFromDashboard() {
    return SmartDashboard.getNumber("pivot/dashboardTestVelocityRotationsPerSecond", 1);
  }

  private Double getRollerVelocityFromDashboard() {
    return SmartDashboard.getNumber("roller/dashboardTestVelocityRotationsPerSecond", RollerConstants.ROLLER_VELOCITY);
  }

  private Double geIndexerVelocityFromDashboard() {
    return SmartDashboard.getNumber("indexer/dashboardTestVelocityRotationsPerSecond", IndexerConstants.INDEXER_VELOCITY);
  }

  private Double getColumnVelocityFromDashboard() {
    return SmartDashboard.getNumber("column/dashboardTestVelocityRotationsPerSecond", ColumnConstants.COLUMN_VELOCITY);
  }

  private Double getShooterVelocityFromDashboard() {
    return SmartDashboard.getNumber("shooter/influencer/dashboardTestVelocityRotationsPerSecond", ShooterConstants.SHOOTER_VELOCITY);
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

  /**
   * Calculates the required field-relative drivetrain angle for shooting on the fly.
   * (NOTE: for a fixed shooter, the RPM doesn't change - only the angle does)
   */
  private Rotation2d calculateShootOnTheFlyAngle() {
    // Get robot state
    Pose2d robotPose = m_drivetrain.getState().Pose;
    ChassisSpeeds robotSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
      m_drivetrain.getState().Speeds,
      robotPose.getRotation());

    // Calculate SOTF shot vector using HubCalculations
    Rotation2d drivetrainAngle = HubCalculations.calculateShootOnTheFlyAngle(robotPose, robotSpeeds);

    if (drivetrainAngle == null) {
      // Distance out of range, fall back to stationary angle (which will be like 0 lol)
      return HubCalculations.angleToHub(robotPose);
    }

    SmartDashboard.putNumber("robotCommandFactory/SOTFdrivetrainAngleDegrees", drivetrainAngle.getDegrees());
    SmartDashboard.putNumber("robotCommandFactory/SOTFdrivetrainAngleRadians", drivetrainAngle.getRadians());

    return drivetrainAngle;
  }

  /**
   * Calculates the shooter velocity based on distance to shuttle corner.
   */
  private Double calculateShuttleShooterVelocity() {
    // Calculate distance to closest shuttle corner
    Pose2d robotPose = m_drivetrain.getState().Pose;
    double distanceToCorner = HubCalculations.distanceToShuttleCorner(robotPose);

    SmartDashboard.putNumber("robotCommandFactory/shuttleDistance", distanceToCorner); 

    // Look up target velocity from distance
    Optional<Double> velocityOptional = ShooterLookupTable.getVelocityForDistance(distanceToCorner);

    if (velocityOptional.isPresent()) {
      return velocityOptional.get();
    } else {
      return 0.0; // Distance out of range
    }
  }

  /**
   * Calculates the required field-relative drivetrain angle for shuttling.
   * Uses SOTM math to compensate for robot velocity.
   */
  private Rotation2d calculateShuttleAngle() {
    // Get robot state
    Pose2d robotPose = m_drivetrain.getState().Pose;
    ChassisSpeeds robotSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
      m_drivetrain.getState().Speeds,
      robotPose.getRotation());

    // Calculate shuttle angle using HubCalculations
    Rotation2d shuttleAngle = HubCalculations.calculateShuttleAngle(robotPose, robotSpeeds);

    if (shuttleAngle == null) {
      // Distance out of range, fall back to simple angle to closest corner
      Translation2d closestCorner = HubCalculations.getClosestShuttleCorner(robotPose);
      Translation2d toCorner = closestCorner.minus(robotPose.getTranslation());
      return toCorner.getAngle();
    }

    SmartDashboard.putNumber("robotCommandFactory/shuttleAngleDegrees", shuttleAngle.getDegrees());
    SmartDashboard.putNumber("robotCommandFactory/shuttleAngleRadians", shuttleAngle.getRadians());

    return shuttleAngle;
  }

  public double getDistanceToHub() {
    return HubCalculations.distanceToHub(m_drivetrain.getState().Pose);
  }

  public double getAngleToHub() {
    return m_drivetrainAngleSupplier.get().getDegrees();
  }

  public double getTargetShooterVelocity() {
    return m_shooterDashboardSupplier.get();
  }

  public double getCalculatedShooterVelocity() {
    return m_dynamicShooterVelocitySupplier.get();
  }

  /**
   * Gets the calculated shooter velocity for shooting on the fly
   * @return SOTF shooter velocity in rotations per second
   */
  public double getSOTFShooterVelocity() {
    return m_shooterVelocitySOTFSupplier.get();
  }

  /**
   * Gets the calculated angle for shooting on the fly
   * @return SOTF angle 
   */
  public Rotation2d getSOTFAngle() {
    return calculateShootOnTheFlyAngle();
  }

  /**
   * Checks if the robot's speed is low enough to shoot accurately
   * @return true if robot speed is below threshold
   */
  private boolean isRobotSpeedLowEnough() {
    ChassisSpeeds robotSpeeds = m_drivetrain.getState().Speeds;
    double robotSpeed = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

    SmartDashboard.putNumber("robotCommandFactory/robotSpeed", robotSpeed);
    SmartDashboard.putBoolean("robotCommandFactory/isRobotSpeedLowEnough",
      robotSpeed < ShooterConstants.MAX_ROBOT_SPEED_TO_SHOOT);

    return robotSpeed < ShooterConstants.MAX_ROBOT_SPEED_TO_SHOOT;
  }

  public Trigger getReadyToShootTrigger() {
    return m_readyToShootTrigger;
  }

  public boolean atAngle() {
    return m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier).getAsBoolean();
  }

  public Trigger atAngleTrigger() {
    return m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier);
  }

  public boolean atVelocity() {
    return m_shooter.isMotorVelocityWithinPercentTolerance(m_dynamicShooterVelocitySupplier).getAsBoolean();
  }

  /**
   * Updates the shooter velocity latch based on current conditions.
   * This method should be called periodically to update the latch state.
   *
   * Latch logic:
   * - Latch sets to true when shooter reaches target velocity
   * - Latch stays true even if shooter dips below velocity
   * - Latch resets to false when: heading is wrong, speed is too high, or scoreMode is false
   */
  private void updateShooterVelocityLatch() {
    boolean shooterAtVel = m_shooter.isMotorVelocityWithinPercentTolerance(m_dynamicShooterVelocitySupplier).getAsBoolean();
    boolean headingCorrect = m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier).getAsBoolean();
    boolean speedLow = isRobotSpeedLowEnough();
    boolean scoreMode = m_scoreModeSupplier.get();


    // Set latch if shooter reaches velocity
    if (shooterAtVel) {
      m_shooterVelocityLatch = true;
    }
    
    // Reset latch if any critical condition fails
    if (!headingCorrect || !speedLow || !scoreMode) {
      m_shooterVelocityLatch = false;
    }

    SmartDashboard.putBoolean("robotCommandFactory/shooterVelocityLatch", m_shooterVelocityLatch);
  }

  /**
   * Creates a trigger that indicates the robot is ready to shoot.
   * Combines shooter velocity (with latching), heading, and robot speed checks with debouncing.
   *
   * The latch allows continuous feeding even if shooter velocity fluctuates,
   * while still stopping when aim or movement conditions fail.
   *
   * @return Trigger that is true when all shooting conditions are met
   */
  private Trigger readyToShootTrigger() {
    Trigger robotHeadingCorrect = m_drivetrainCommandFactory.atAngleTrigger(m_drivetrainAngleSupplier);
    Trigger robotSpeedLow = new Trigger(this::isRobotSpeedLowEnough);

    // Create trigger using latched shooter velocity (updated in updateShooterVelocityLatch)
    Trigger shooterReady = new Trigger(() -> {
      updateShooterVelocityLatch();
      return m_shooterVelocityLatch;
    });

    Trigger readyTrigger = shooterReady
      .and(robotHeadingCorrect)
      .and(robotSpeedLow);

    // Add debouncing - especially important on the falling edge to prevent rapid on/off
    return readyTrigger.debounce(ShooterConstants.READY_TO_SHOOT_DEBOUNCE_TIME, DebounceType.kBoth);
  }

  // syntactic sugar
  private Trigger not(Trigger trigger) {
    return trigger.negate();
    
  }
}
