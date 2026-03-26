// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrainSim;
import frc.robot.subsystems.drivetrain.DrivetrainCommandFactory;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.intake.ColumnSubsystem;
import frc.robot.subsystems.intake.IndexerSubsystem;
import frc.robot.subsystems.vision.LocalVisionFilterPipeline;
import frc.robot.subsystems.vision.GlobalVisionFilterPipeline;
import frc.robot.subsystems.vision.LocalizationCamera;
import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.filtering.GlobalCrossCameraConsensusFilter;
import frc.robot.subsystems.vision.filtering.LocalCameraPoseConsistencyDistanceToFusedPoseFilter;
import frc.robot.subsystems.vision.filtering.LocalCameraPoseConsistencyFilter;
import frc.robot.subsystems.vision.filtering.LocalDistanceToFusedPoseFilter;
import frc.robot.subsystems.vision.filtering.LocalPoseZRollPitchFilter;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.HubCalculations;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.intake.RollerSubsystem;
import frc.robot.subsystems.scorer.ShooterSubsystem;
import frc.robot.subsystems.LaserCANSensorBase;
import frc.robot.subsystems.RobotCommandFactory;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer.JoystickVals;
import frc.robot.Constants.SensorConstants;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix6.SignalLogger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static record JoystickVals(double x, double y) {
    public boolean isMagnitudeGreaterThan(double threshold) {
      return Math.hypot(x, y) > threshold;
    }
  }

  private final SendableChooser<Command> m_autoChooser; // Sendable chooser that holds the autos
  private final Telemetry logger = new Telemetry(DrivetrainConstants.MAX_TRANSLATION_SPEED);

  // Subsystems
  public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  public final RollerSubsystem m_roller = new RollerSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final VisionSubsystem m_vision = new VisionSubsystem(() -> m_drivetrain.getState().Pose.getRotation());
  public final IndexerSubsystem m_indexer = new IndexerSubsystem();
  public final PivotSubsystem m_pivot = new PivotSubsystem();
  public final ColumnSubsystem m_column = new ColumnSubsystem();

  // Sensors
  private final LaserCANSensorBase m_intakeSensor = new LaserCANSensorBase(
    SensorConstants.INTAKE_SENSOR_CAN_ID,
    "intake/sensor",
    SensorConstants.INTAKE_SENSOR_MIN_DISTANCE
  );
  private final LaserCANSensorBase m_shooterSensor = new LaserCANSensorBase(
    SensorConstants.FEEDER_SENSOR_CAN_ID,
    "shooter/sensor",
    SensorConstants.FEEDER_SENSOR_MIN_DISTANCE
  );

  // Command factories
  private final DrivetrainCommandFactory m_drivetrainCommandFactory = new DrivetrainCommandFactory(m_drivetrain);
  private final RobotCommandFactory m_robotCommandFactory = new RobotCommandFactory(m_drivetrain, m_pivot, m_roller, m_indexer, m_column, m_shooter, m_intakeSensor, m_shooterSensor, m_vision, m_drivetrainCommandFactory);

  private final CommandXboxController m_driverJoystick =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorJoystick =
    new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController m_testJoystick =
    new CommandXboxController(OperatorConstants.kTestControllerPort);

  // Joystick suppliers
  private final Supplier<JoystickVals> m_driverTranslationJoystickValsSupplier =
    () -> new JoystickVals(m_driverJoystick.getLeftX(), m_driverJoystick.getLeftY());
  private final Supplier<JoystickVals> m_driverRotationJoystickValsSupplier =
    () -> new JoystickVals(m_driverJoystick.getRightX(), m_driverJoystick.getRightY());

  private final Field2d m_actualField = new Field2d(); // field simulation

  /** The container for the robot. Contains subsystems and commands. */
  public RobotContainer() {

    // Configure trigger bindings
    if (Utils.isSimulation()) {
      configureBindingsSimulation();
    } else {
      configureBindingsCompetition();
      configureBindingsVision();
    }

    // Configure auto builder
    createNamedCommands();
    m_autoChooser = AutoBuilder.buildAutoChooser("drive forward 1m");
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Data logging
    DataLogManager.start(); // Starts recording to data log
    DriverStation.startDataLog(DataLogManager.getLog()); // Record both DS control and joystick data
    DriverStation.silenceJoystickConnectionWarning(true); // Turn off unplugged joystick errors

    logToSmartDashboard();

    SignalLogger.start();
  }


  // ----- CONFIGURE BINDINGS -----

  /**
   * Define trigger -> command mappings
   */
  private void configureBindingsCompetition() {

    // --- DRIVER COMMANDS ---
    // Default drive
    m_drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      m_drivetrainCommandFactory.defaultDrive(
        m_driverTranslationJoystickValsSupplier,
        m_driverRotationJoystickValsSupplier
      )
    );

    // x (on true): intake + led red
    m_driverJoystick.x().and(m_driverJoystick.leftBumper().negate()).onTrue(m_robotCommandFactory.intakeModeCommand());
    // y (on true): neutral + led blue
    m_driverJoystick.y().and(m_driverJoystick.leftBumper().negate()).onTrue(m_robotCommandFactory.neutralModeCommand());
    // b (on true): score + led green
    m_driverJoystick.b().and(m_driverJoystick.leftBumper().negate()).onTrue(
      m_robotCommandFactory.scoreModeCommand(m_driverTranslationJoystickValsSupplier, driverInputTrigger()));

    // a: snap to bump
    m_driverJoystick.a().and(m_driverJoystick.leftBumper().negate()).whileTrue(
      m_drivetrainCommandFactory.snapToBump(m_driverTranslationJoystickValsSupplier));
    // left bumper + x: deploy pivot
    m_driverJoystick.leftBumper().and(m_driverJoystick.x()).onTrue(m_pivot.deployPivotCommand());
    // left bumper + y: store pivot
    m_driverJoystick.leftBumper().and(m_driverJoystick.y()).onTrue(m_pivot.storePivotCommand());
    // left bumper + b: snap to hub
    m_driverJoystick.leftBumper().and(m_driverJoystick.b()).whileTrue(
      m_robotCommandFactory.snapToHubCommand(m_driverTranslationJoystickValsSupplier));
    // left bumper + a: point wheels in x
    m_driverJoystick.leftBumper().and(m_driverJoystick.a()).whileTrue(m_drivetrainCommandFactory.pointWheelsinX());
    // right bumper: slowmode
    m_drivetrainCommandFactory.setSlowmodeButton(m_driverJoystick.rightBumper());
    // left trigger: shuttle
    m_driverJoystick.leftTrigger().whileTrue(m_robotCommandFactory.shuttleCommand());
    // right trigger: outtake / everything backwards (voltage -5)
    m_driverJoystick.rightTrigger().whileTrue(m_robotCommandFactory.outtakeCommand());

    // center d-pad: zero pivot
    m_driverJoystick.povCenter().negate().whileTrue(m_pivot.zeroPivotCommand());

    // ----------

    // --- OPERATOR COMMANDS ---
    // x: intake + indexer forward
    m_operatorJoystick.x().and(m_operatorJoystick.leftBumper().negate()).whileTrue(m_robotCommandFactory.runRollerIndexerCommand());
    // y: pivot down
    m_operatorJoystick.y().and(m_operatorJoystick.leftBumper().negate()).whileTrue(m_pivot.voltageDeployPivotCommand());
    // b: indexer + column forward
    m_operatorJoystick.b().and(m_operatorJoystick.leftBumper().negate()).whileTrue(m_robotCommandFactory.runIndexerColumnCommand());
    // a: intake, indexer, column forwards
    m_operatorJoystick.a().and(m_operatorJoystick.leftBumper().negate()).whileTrue(m_robotCommandFactory.runAllRollersCommand());
    // left bumper + x: intake + indexer backwards
    m_operatorJoystick.leftBumper().and(m_operatorJoystick.x()).whileTrue(m_robotCommandFactory.runRollerIndexerBackCommand());
    // left bumper + y: pivot up
    m_operatorJoystick.leftBumper().and(m_operatorJoystick.y()).whileTrue(m_pivot.voltageStorePivotCommand());
    // left bumper + b: indexer + column backwards
    m_operatorJoystick.leftBumper().and(m_operatorJoystick.b()).whileTrue(m_robotCommandFactory.runIndexerColumnBackCommand());
    // left bumper + a: intake, indexer, column backwards
    m_operatorJoystick.leftBumper().and(m_operatorJoystick.a()).whileTrue(m_robotCommandFactory.runAllRollersBackCommand());
    // right bumper: score speed 1
    m_operatorJoystick.rightBumper().whileTrue(m_robotCommandFactory.runShooterCloseDistanceCommand());
    // left trigger: score speed 2
    m_operatorJoystick.leftTrigger().whileTrue(m_robotCommandFactory.runShooterMediumDistanceCommand());
    // right trigger: score speed 3
    m_operatorJoystick.rightTrigger().whileTrue(m_robotCommandFactory.runShooterFarDistanceCommand());

    m_operatorJoystick.povCenter().negate().whileTrue(new InstantCommand(() -> m_pivot.resetToDeployPosition()));


    m_drivetrain.registerTelemetry(logger::telemeterize);

    configureDefaultCommands();
  }

  // updated 3/1/26
  private void configureBindingsTestingMechanisms() {
    // x: deploy pivot
    m_testJoystick.x().and(m_testJoystick.leftBumper().negate()).whileTrue(m_pivot.deployPivotCommand());
    // y: store pivot
    m_testJoystick.y().and(m_testJoystick.leftBumper().negate()).whileTrue(m_pivot.storePivotCommand());
    // b: run roller
    m_testJoystick.b().and(m_testJoystick.leftBumper().negate()).whileTrue(m_robotCommandFactory.runRollerBackTestCommand());
    // a: run roller and indexer 
    m_testJoystick.a().and(m_testJoystick.leftBumper().negate()).whileTrue(m_robotCommandFactory.runRollerIndexerBackCommand());

    // left bumper + x: deploy pivot motion magic
    m_testJoystick.leftBumper().and(m_testJoystick.x()).whileTrue(m_pivot.deployPivotCommand());
    // left bumper + y: store pivot motion magic
    m_testJoystick.leftBumper().and(m_testJoystick.y()).whileTrue(m_pivot.storePivotCommand());
    // left bumper + b: run roller back
    // m_testJoystick.leftBumper().and(m_testJoystick.b()).whileTrue(m_robotCommandFactory.runRollersBackCommand());
    // left bumper + a: run roller and indexer back
    // m_testJoystick.leftBumper().and(m_testJoystick.a()).whileTrue(m_robotCommandFactory.runIntakeRollersBackCommand());

    m_testJoystick.leftTrigger().whileTrue(m_pivot.storePivotCommand());
    m_testJoystick.rightTrigger().whileTrue(
      new ParallelCommandGroup(
        m_robotCommandFactory.runRollerIndexerCommand(),
        m_robotCommandFactory.runShooterTestCommand(),
        m_column.columnVelocityCommand()
      )
    );

    m_testJoystick.povCenter().negate().onTrue(new InstantCommand(() -> resetControllerConstantsSmartDashboard()));

    configureDefaultCommands();
  }

  private void configureBindingsTestingDrivetrain() {
    m_testJoystick.a().and(m_testJoystick.leftBumper().negate()).whileTrue(m_drivetrainCommandFactory.sysIdQuasistaticTranslationForward());
    m_testJoystick.leftBumper().and(m_testJoystick.a()).whileTrue(m_drivetrainCommandFactory.sysIdQuasistaticTranslationReverse());
    m_testJoystick.b().and(m_testJoystick.leftBumper().negate()).whileTrue(m_drivetrainCommandFactory.sysIdDynamicTranslationForward());
    m_testJoystick.leftBumper().and(m_testJoystick.b()).whileTrue(m_drivetrainCommandFactory.sysIdDynamicTranslationReverse());
    m_testJoystick.x().and(m_testJoystick.leftBumper().negate()).whileTrue(m_drivetrainCommandFactory.sysIdQuasistaticRotationForward());
    m_testJoystick.leftBumper().and(m_testJoystick.x()).whileTrue(m_drivetrainCommandFactory.sysIdQuasistaticRotationReverse());
    m_testJoystick.y().and(m_testJoystick.leftBumper().negate()).whileTrue(m_drivetrainCommandFactory.sysIdDynamicRotationForward());
    m_testJoystick.leftBumper().and(m_testJoystick.y()).whileTrue(m_drivetrainCommandFactory.sysIdDynamicRotationReverse());
  }

  private void configureBindingsVision() {
    // --- CONFIGURE VISION FILTERING ---
    GlobalVisionFilterPipeline globalPipeline = new GlobalVisionFilterPipeline();
    LocalVisionFilterPipeline localPipeline = new LocalVisionFilterPipeline();

    // Add global filters
    globalPipeline.addFilter("crossCameraConsensus", new GlobalCrossCameraConsensusFilter());

     // Add local filters
    localPipeline.addFilter("poseZRollPitch", new LocalPoseZRollPitchFilter());
    localPipeline.addFilter("localCameraPoseConsistencyDistanceToFusedPose", new LocalCameraPoseConsistencyDistanceToFusedPoseFilter(m_drivetrain));

    m_vision.setGlobalFilterPipeline(globalPipeline);
    m_vision.setLocalFilteringPipeline(localPipeline);

    // --- CONFIGURE RAW VIDEO MODE TOGGLE ---
    m_testJoystick.start().onTrue(m_vision.toggleRawVideoModeCommand());
  }

  private void configureBindingsSimulation() {
    // Default drive
    m_drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      m_drivetrainCommandFactory.defaultDrive(
        () -> new JoystickVals(m_driverJoystick.getLeftX(), m_driverJoystick.getLeftY()),
        () -> new JoystickVals(m_driverJoystick.getRightX(), m_driverJoystick.getRightY())
      )
    );

    // Drive in slowmode while right bumper is pressed
    m_drivetrainCommandFactory.setSlowmodeButton(m_driverJoystick.rightBumper());

    Consumer<SwerveDriveState> telemetry =  ((CommandSwerveDrivetrainSim) m_drivetrain)
      .getSimTelemetryConsumer().andThen(logger::telemeterize);
    m_drivetrain.registerTelemetry(telemetry);
  }

  private void configureDefaultCommands() {
    m_robotCommandFactory.setDefaultCommand();
  }

  public void resetPosition() {
    m_robotCommandFactory.resetPosition();
  }

  // ----- LOGGING -----
  public void logToSmartDashboard() {
    // Indexer
    SmartDashboard.putNumber("indexer/kP", SmartDashboard.getNumber("indexer/kP", IndexerConstants.kP));
    SmartDashboard.putNumber("indexer/kI", SmartDashboard.getNumber("indexer/kI", IndexerConstants.kI));
    SmartDashboard.putNumber("indexer/kD", SmartDashboard.getNumber("indexer/kD", IndexerConstants.kD));
    SmartDashboard.putNumber("indexer/dashboardTestVelocityRotationsPerSecond",
        SmartDashboard.getNumber("indexer/dashboardTestVelocityRotationsPerSecond", IndexerConstants.INDEXER_VELOCITY));
    SmartDashboard.putNumber("indexer/dashboardTestVoltage",
        SmartDashboard.getNumber("indexer/dashboardTestVoltage", IndexerConstants.INDEXER_VOLTAGE));

    // Column
    SmartDashboard.putNumber("column/kP", SmartDashboard.getNumber("column/kP", ColumnConstants.kP));
    SmartDashboard.putNumber("column/kI", SmartDashboard.getNumber("column/kI", ColumnConstants.kI));
    SmartDashboard.putNumber("column/kD", SmartDashboard.getNumber("column/kD", ColumnConstants.kD));
    SmartDashboard.putNumber("column/dashboardTestVelocityRotationsPerSecond",
        SmartDashboard.getNumber("column/dashboardTestVelocityRotationsPerSecond", ColumnConstants.COLUMN_VELOCITY));
    SmartDashboard.putNumber("column/dashboardTestVoltage",
        SmartDashboard.getNumber("column/dashboardTestVoltage", ColumnConstants.COLUMN_VOLTAGE));

    // Roller
    SmartDashboard.putNumber("roller/kP", SmartDashboard.getNumber("roller/kP", RollerConstants.kP));
    SmartDashboard.putNumber("roller/kI", SmartDashboard.getNumber("roller/kI", RollerConstants.kI));
    SmartDashboard.putNumber("roller/kD", SmartDashboard.getNumber("roller/kD", RollerConstants.kD));
    SmartDashboard.putNumber("roller/dashboardTestVelocityRotationsPerSecond",
        SmartDashboard.getNumber("roller/dashboardTestVelocityRotationsPerSecond", RollerConstants.ROLLER_VELOCITY));
    SmartDashboard.putNumber("roller/dashboardTestVoltage",
        SmartDashboard.getNumber("roller/dashboardTestVoltage", RollerConstants.ROLLER_VOLTAGE));

    // Pivot
    SmartDashboard.putNumber("pivot/kP", SmartDashboard.getNumber("pivot/kP", PivotConstants.kP));
    SmartDashboard.putNumber("pivot/kI", SmartDashboard.getNumber("pivot/kI", PivotConstants.kI));
    SmartDashboard.putNumber("pivot/kD", SmartDashboard.getNumber("pivot/kD", PivotConstants.kD));
    SmartDashboard.putNumber("pivot/kS", SmartDashboard.getNumber("pivot/kS", PivotConstants.kS));
    SmartDashboard.putNumber("pivot/kV", SmartDashboard.getNumber("pivot/kV", PivotConstants.kV));
    SmartDashboard.putNumber("pivot/kA", SmartDashboard.getNumber("pivot/kA", PivotConstants.kA));
    SmartDashboard.putNumber("pivot/dashboardTestVelocityRotationsPerSecond",
        SmartDashboard.getNumber("pivot/dashboardTestVelocityRotationsPerSecond", PivotConstants.DEPLOY_VELOCITY));
    SmartDashboard.putNumber("pivot/dashboardTestVoltage",
        SmartDashboard.getNumber("pivot/dashboardTestVoltage", PivotConstants.DEPLOY_VOLTAGE));

    SmartDashboard.putNumber("pivot/motionMagicCruiseVelocity",
        SmartDashboard.getNumber("pivot/motionMagicCruiseVelocity", PivotConstants.CRUISE_VELOCITY));
    SmartDashboard.putNumber("pivot/motionMagicAcceleration",
        SmartDashboard.getNumber("pivot/motionMagicAcceleration", PivotConstants.ACCELERATION));
    SmartDashboard.putNumber("pivot/motionMagicJerk",
        SmartDashboard.getNumber("pivot/motionMagicJerk", PivotConstants.JERK));

    SmartDashboard.putNumber("pivot/dashboardTestStorePositionDegrees",
        SmartDashboard.getNumber("pivot/dashboardTestStorePositionDegrees", PivotConstants.STORE_POSITION_DEGREES));
    SmartDashboard.putNumber("pivot/dashboardTestDeployPositionDegrees",
        SmartDashboard.getNumber("pivot/dashboardTestDeployPositionDegrees", PivotConstants.DEPLOY_POSITION_DEGREES));


    // Shooter Influencer
    SmartDashboard.putNumber("shooter/influencer/kP", SmartDashboard.getNumber("shooter/influencer/kP", ShooterConstants.INFLUENCER_kP));
    SmartDashboard.putNumber("shooter/influencer/kI", SmartDashboard.getNumber("shooter/influencer/kI", ShooterConstants.INFLUENCER_kI));
    SmartDashboard.putNumber("shooter/influencer/kD", SmartDashboard.getNumber("shooter/influencer/kD", ShooterConstants.INFLUENCER_kD));
    SmartDashboard.putNumber("shooter/influencer/dashboardTestVelocityRotationsPerSecond",
        SmartDashboard.getNumber("shooter/influencer/dashboardTestVelocityRotationsPerSecond", ShooterConstants.SHOOTER_VELOCITY));
    SmartDashboard.putNumber("shooter/influencer/dashboardTestVoltage",
        SmartDashboard.getNumber("shooter/influencer/dashboardTestVoltage", ShooterConstants.SHOOTER_VOLTAGE));

    // Robot Command Factory Logging 
    SmartDashboard.putNumber("robotCommandFactory/distanceToHubMeters", m_robotCommandFactory.getDistanceToHub());
    SmartDashboard.putNumber("robotCommandFactory/angleToHubDegrees", m_robotCommandFactory.getAngleToHub());
    SmartDashboard.putNumber("robotCommandFactory/angleToHubRadians", Math.toRadians(m_robotCommandFactory.getAngleToHub()));
    SmartDashboard.putNumber("robotCommandFactory/dashboardTestShooterVelocityRotationsPerSecond",
        m_robotCommandFactory.getTargetShooterVelocity());
  }


  public void logShootByDistanceValues() {
    // Robot Command Factory Logging 
    SmartDashboard.putNumber("robotCommandFactory/distanceToHubMeters", m_robotCommandFactory.getDistanceToHub());
    SmartDashboard.putNumber("robotCommandFactory/angleToHubDegrees", m_robotCommandFactory.getAngleToHub());
    SmartDashboard.putNumber("robotCommandFactory/angleToHubRadians", Math.toRadians(m_robotCommandFactory.getAngleToHub()));
    SmartDashboard.putNumber("robotCommandFactory/calculatedShooterVelocityRotationsPerSecond",
        m_robotCommandFactory.getCalculatedShooterVelocity());

  }

  private void resetControllerConstantsSmartDashboard() {
    // Indexer
    IndexerConstants.kP = SmartDashboard.getNumber("indexer/kP", 0);
    IndexerConstants.kI = SmartDashboard.getNumber("indexer/kI", 0);
    IndexerConstants.kD = SmartDashboard.getNumber("indexer/kD", 0);
    // Column
    ColumnConstants.kP = SmartDashboard.getNumber("column/kP", 0);
    ColumnConstants.kI = SmartDashboard.getNumber("column/kI", 0);
    ColumnConstants.kD = SmartDashboard.getNumber("column/kD", 0);

    // Roller
    RollerConstants.kP = SmartDashboard.getNumber("roller/kP", 0);
    RollerConstants.kI = SmartDashboard.getNumber("roller/kI", 0);
    RollerConstants.kD = SmartDashboard.getNumber("roller/kD", 0);

    // Pivot
    PivotConstants.kP = SmartDashboard.getNumber("pivot/kP", 0);
    PivotConstants.kI = SmartDashboard.getNumber("pivot/kI", 0);
    PivotConstants.kD = SmartDashboard.getNumber("pivot/kD", 0);

    PivotConstants.kS = SmartDashboard.getNumber("pivot/kS", 0);
    PivotConstants.kV = SmartDashboard.getNumber("pivot/kV", 0);
    PivotConstants.kA = SmartDashboard.getNumber("pivot/kA", 0);

    PivotConstants.CRUISE_VELOCITY = SmartDashboard.getNumber("pivot/motionMagicCruiseVelocity", 0);
    PivotConstants.ACCELERATION = SmartDashboard.getNumber("pivot/motionMagicAcceleration", 0);
    PivotConstants.JERK = SmartDashboard.getNumber("pivot/motionMagicJerk", 0);

    PivotConstants.STORE_POSITION_DEGREES = SmartDashboard.getNumber("pivot/dashboardTestStorePositionDegrees", 0);
    PivotConstants.DEPLOY_POSITION_DEGREES = SmartDashboard.getNumber("pivot/dashboardTestDeployPositionDegrees", 0);

    // Shooter
    ShooterConstants.INFLUENCER_kP = SmartDashboard.getNumber("shooter/influencer/kP", 0);
    ShooterConstants.INFLUENCER_kI = SmartDashboard.getNumber("shooter/influencer/kI", 0);
    ShooterConstants.INFLUENCER_kD = SmartDashboard.getNumber("shooter/influencer/kD", 0);

    m_pivot.resetControllers();
    m_roller.resetControllers();
    m_indexer.resetControllers();
    m_column.resetControllers();
    m_shooter.resetControllers();
  }

  private Trigger driverTranslationInputTrigger() {
    return new Trigger(() -> m_driverTranslationJoystickValsSupplier.get().isMagnitudeGreaterThan(OperatorConstants.DRIVE_JOYSTICK_DEADBAND));
  }

  private Trigger driverRotationInputTrigger() {
    return new Trigger(() -> m_driverRotationJoystickValsSupplier.get().isMagnitudeGreaterThan(OperatorConstants.STEER_JOYSTICK_DEADBAND));
  }

  private Trigger driverInputTrigger() {
    return driverTranslationInputTrigger().or(driverRotationInputTrigger());
  }


  // ----- AUTONOMOUS -----
  /**
   * Define named commands for autonomous paths
   */
  private void createNamedCommands() {

    new EventTrigger("deploy intake trigger").onTrue(m_pivot.autoZeroPivotCommand()); 
    new EventTrigger("intake trigger").onTrue(m_robotCommandFactory.intakeModeCommand());
    new EventTrigger("shoot trigger").onTrue(m_robotCommandFactory.autoShootWithVisionCommand().withTimeout(AutoConstants.AUTO_SHOOT_TIMEOUT)); // TODO: tune timeout

    NamedCommands.registerCommand("intake command", 
      m_robotCommandFactory.intakeModeCommand()); // DOES NOT END 
     NamedCommands.registerCommand("deploy intake command", 
      m_pivot.autoZeroPivotCommand());
    NamedCommands.registerCommand("snap to hub command", 
      m_robotCommandFactory.snapToHubCommand(() -> new JoystickVals(0, 0)).withTimeout(AutoConstants.AUTO_SHOOT_TIMEOUT));
    NamedCommands.registerCommand("climb command", 
      new WaitCommand(1));
    NamedCommands.registerCommand("shoot command",
       m_robotCommandFactory.autoShootWithVisionCommand().withTimeout(AutoConstants.AUTO_SHOOT_TIMEOUT));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }


  // ----- POSE ESTIMATION -----

  public void updatePoseEst() {
    List<CameraReading> allReadings = m_vision.getValidCameraReadings();

    for (CameraReading reading : allReadings){
      EstimatedRobotPose robotPose = reading.robotPose();

      // Sample drivetrain fusedPose before updating
      SmartDashboard.putNumberArray("fusedVision/" + reading.cameraName() + "/drivetrainPoseBeforeUpdate", new double [] {
      m_drivetrain.getState().Pose.getX(), m_drivetrain.getState().Pose.getY(), m_drivetrain.getState().Pose.getRotation().getRadians()});
      SmartDashboard.putNumber("fusedVision/" + reading.cameraName() + "/drivetrainHeadingBeforeUpdateDegrees",
          m_drivetrain.getState().Pose.getRotation().getDegrees());
      SmartDashboard.putNumber("fusedVision/" + reading.cameraName() + "/drivetrainHeadingBeforeUpdateRadians",
          m_drivetrain.getState().Pose.getRotation().getRadians());

      // Update fusedPose
      m_drivetrain.setVisionMeasurementStdDevs(reading.stdDevs());
      m_drivetrain.addVisionMeasurement(robotPose.estimatedPose.toPose2d(), robotPose.timestampSeconds);

      // sample drivetrain fusedPose after updating
      SmartDashboard.putNumberArray("fusedVision/" + reading.cameraName() + "/drivetrainPoseAfterUpdate", new double [] {
        m_drivetrain.getState().Pose.getX(), m_drivetrain.getState().Pose.getY(), m_drivetrain.getState().Pose.getRotation().getRadians()});
      SmartDashboard.putNumber("fusedVision/" + reading.cameraName() + "/drivetrainHeadingAfterUpdateDegrees",
          m_drivetrain.getState().Pose.getRotation().getDegrees());
      SmartDashboard.putNumber("fusedVision/" + reading.cameraName() + "/drivetrainHeadingAfterUpdateRadians",
          m_drivetrain.getState().Pose.getRotation().getRadians());
    }
    
    m_actualField.setRobotPose(m_drivetrain.getState().Pose);
    SmartDashboard.putData("fusedVision/actualField", m_actualField);
  }
/*   public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
    // The pose by maplesim, including collisions with the field. 
    // See https://www.chiefdelphi.com/t/simulated-robot-goes-through-walls-with-maplesim/508663.

  } */
}
