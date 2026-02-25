// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrainSim;
import frc.robot.subsystems.drivetrain.DrivetrainCommandFactory;
import frc.robot.subsystems.intake.IntakeCommandFactory;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.intake.ColumnSubsystem;
import frc.robot.subsystems.intake.IndexerSubsystem;
import frc.robot.subsystems.scorer.ScorerCommandFactory;
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
import frc.robot.utils.HubCalculations;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.intake.RollerSubsystem;
import frc.robot.subsystems.scorer.ShooterSubsystem;
import frc.robot.subsystems.LaserCANSensorBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.VisionConstants;
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

import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix6.SignalLogger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static record JoystickVals(double x, double y) {}

  public enum RobotMode {
    NEUTRAL,
    INTAKE,
    SHOOT
  }

  private final SendableChooser<Command> m_autoChooser; // Sendable chooser that holds the autos
  private final Telemetry logger = new Telemetry(DrivetrainConstants.MAX_TRANSLATION_SPEED);
  private RobotMode m_robotMode;

  // Subsystems
  public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  public final RollerSubsystem m_roller = new RollerSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final VisionSubsystem m_vision = new VisionSubsystem();
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
  private final IntakeCommandFactory m_intakeCommandFactory = new IntakeCommandFactory(m_roller, m_intakeSensor, m_pivot, m_indexer, m_column);
  private final ScorerCommandFactory m_shooterCommandFactory = new ScorerCommandFactory(m_shooter, m_shooterSensor);

  private final CommandXboxController m_driverJoystick =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_testJoystick =
    new CommandXboxController(OperatorConstants.kTestControllerPort);

  private final Field2d m_actualField = new Field2d(); // field simulation

  /** The container for the robot. Contains subsystems and commands. */
  public RobotContainer() {

    // Configure trigger bindings
    if (Utils.isSimulation()) {
      configureBindingsSimulation();
    } else {
      configureBindingsTestingMechanisms();
      configureBindingsVision();
    }

    setRobotMode(RobotMode.NEUTRAL);

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
    // Default drive
    m_drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      m_drivetrainCommandFactory.defaultDrive(
        () -> new JoystickVals(m_driverJoystick.getLeftX(), m_driverJoystick.getLeftY()),
        () -> new JoystickVals(m_driverJoystick.getRightX(), m_driverJoystick.getRightY()),
        () -> false
      )
    );

    // Drive in slowmode while right bumper is pressed
    m_driverJoystick.rightBumper().whileTrue(
      m_drivetrainCommandFactory.defaultDrive(
        () -> new JoystickVals(m_driverJoystick.getLeftX(), m_driverJoystick.getLeftY()),
        () -> new JoystickVals(m_driverJoystick.getRightX(), m_driverJoystick.getRightY()),
        () -> true
      )
    );

    m_driverJoystick.leftBumper().and(m_driverJoystick.x()).onTrue(
      new InstantCommand(() -> setRobotMode(RobotMode.NEUTRAL))
    );
   
    // // TODO: change -- this is for testing
    // m_driverJoystick.y().and(m_driverJoystick.leftBumper().negate()).whileTrue(
    //   m_drivetrainCommandFactory.snapToAngle(
    //     () -> new JoystickVals(m_driverJoystick.getLeftX(), m_driverJoystick.getLeftY()),
    //     0
    //   )
    // );

    // // TODO: change -- this is for testing
    // m_driverJoystick.b().and(m_driverJoystick.leftBumper().negate()).onTrue(
    //   m_drivetrainCommandFactory.snapToTarget(
    //     () -> new JoystickVals(m_driverJoystick.getLeftX(), m_driverJoystick.getLeftY()),
    //     () -> new Pose2d(Units.inchesToMeters(182), Units.inchesToMeters(182), new Rotation2d())
    //   )
    // );

    m_driverJoystick.leftBumper().and(m_driverJoystick.y()).onTrue(
      new InstantCommand(() -> setRobotMode(RobotMode.INTAKE))
    );

    // reset the field-centric heading on a button press
    m_driverJoystick.leftBumper().and(m_driverJoystick.b()).onTrue(
      m_drivetrain.runOnce(() -> m_drivetrain.resetRotation(new Rotation2d(DriverStation.getAlliance().get().equals(Alliance.Blue) ? 0 : Math.PI)))
    );

    m_driverJoystick.leftBumper().and(m_driverJoystick.a()).onTrue(
      new InstantCommand(() -> setRobotMode(RobotMode.SHOOT))
    );

    m_driverJoystick.povCenter().negate().onTrue(new InstantCommand(() -> resetControllerConstantsSmartDashboard()));
    m_drivetrain.registerTelemetry(logger::telemeterize);

    configureDefaultCommandCompetition();
  }

  private void configureBindingsTestingMechanisms() {
    // m_testJoystick.a().whileTrue(m_intakeCommandFactory.runRoller());
    // m_testJoystick.y().whileTrue(m_intakeCommandFactory.runIndexer());
    // m_testJoystick.b().whileTrue(m_intakeCommandFactory.runColumn());

    // m_testJoystick.b().whileTrue(m_intakeCommandFactory.runPivotPID()); // commented for testing 2/15
    m_testJoystick.a().whileTrue(m_intakeCommandFactory.rollerVelocityCommand());
    m_testJoystick.x().whileTrue(m_intakeCommandFactory.indexerVelocityCommand());
    m_testJoystick.y().whileTrue(m_intakeCommandFactory.columnVelocityCommand());

    m_testJoystick.b().whileTrue(new ParallelCommandGroup(
      m_intakeCommandFactory.rollerVelocityCommand(),
      m_intakeCommandFactory.indexerVelocityCommand(),
      m_intakeCommandFactory.columnVelocityCommand()
    ));

   //m_testJoystick.rightBumper().whileTrue(m_intakeCommandFactory.storePivot());
    m_testJoystick.leftBumper().whileTrue(m_intakeCommandFactory.deployPivotCommand());
    //m_testJoystick.rightTrigger().whileTrue(m_shooterCommandFactory.runShooterSmartDashboard());
    m_testJoystick.leftTrigger().whileTrue(m_shooterCommandFactory.shooterBackVoltageCommand());


    //shooter testing bindings:
    m_testJoystick.rightTrigger().whileTrue(m_shooterCommandFactory.shooterSmartDashboardVelocityCommand("high speed", 80));
    m_testJoystick.rightBumper().and(m_testJoystick.rightTrigger().negate()).whileTrue(m_shooterCommandFactory.shooterSmartDashboardVelocityCommand("low speed",70));

    m_testJoystick.povCenter().negate().onTrue(new InstantCommand(() -> resetControllerConstantsSmartDashboard()));

    // // TODO: change -- this is for testing
    // m_testJoystick.y().and(m_testJoystick.leftBumper().negate()).whileTrue(
    //   m_drivetrainCommandFactory.snapToAngle(
    //     () -> new JoystickVals(m_testJoystick.getLeftX(), m_testJoystick.getLeftY()),
    //     0
    //   )
    // );

    // // TODO: change -- this is for testing
    // m_testJoystick.b().and(m_testJoystick.leftBumper().negate()).onTrue(
    //   m_drivetrainCommandFactory.snapToTarget(
    //     () -> new JoystickVals(m_testJoystick.getLeftX(), m_testJoystick.getLeftY()),
    //     () -> new Pose2d(Units.inchesToMeters(182), Units.inchesToMeters(182), new Rotation2d())
    //   )
    // );

    configureDefaultCommandTesting();
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
    localPipeline.addFilter("LocalCameraPoseConsistencyDistanceToFusedPose", new LocalCameraPoseConsistencyDistanceToFusedPoseFilter(m_drivetrain));

    m_vision.setGlobalFilterPipeline(globalPipeline);
    m_vision.setLocalFilteringPipeline(localPipeline);
  }

  private void configureBindingsSimulation() {
    // Default drive
    m_drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      m_drivetrainCommandFactory.defaultDrive(
        () -> new JoystickVals(m_driverJoystick.getLeftX(), m_driverJoystick.getLeftY()),
        () -> new JoystickVals(m_driverJoystick.getRightX(), m_driverJoystick.getRightY()),
        () -> false
      )
    );

    // Drive in slowmode while right bumper is pressed
    m_driverJoystick.rightBumper().whileTrue(
      m_drivetrainCommandFactory.defaultDrive(
        () -> new JoystickVals(m_driverJoystick.getLeftX(), m_driverJoystick.getLeftY()),
        () -> new JoystickVals(m_driverJoystick.getRightX(), m_driverJoystick.getRightY()),
        () -> true
      )
    );

    Consumer<SwerveDriveState> telemetry =  ((CommandSwerveDrivetrainSim) m_drivetrain)
      .getSimTelemetryConsumer().andThen(logger::telemeterize);
    m_drivetrain.registerTelemetry(telemetry);
  }

  private void configureDefaultCommandCompetition() {
    switch (m_robotMode) {
      case NEUTRAL:
        break;
      case INTAKE:
        break;
      case SHOOT:
        break;
    }
  }

  private void configureDefaultCommandTesting() {
    m_intakeCommandFactory.setDefaultCommand();
    m_shooterCommandFactory.setDefaultCommand();
  }

  public void setRobotMode(RobotMode newMode) {
    m_robotMode = newMode;
    configureDefaultCommandCompetition();
    SmartDashboard.putString("robot/mode", m_robotMode.toString());
  }


  // ----- LOGGING -----
  private void logToSmartDashboard() {
    // Indexer
    SmartDashboard.putNumber("indexer IO/kP", SmartDashboard.getNumber("indexer IO/kP", IndexerConstants.kP));
    SmartDashboard.putNumber("indexer IO/kI", SmartDashboard.getNumber("indexer IO/kI", IndexerConstants.kI));
    SmartDashboard.putNumber("indexer IO/kD", SmartDashboard.getNumber("indexer IO/kD", IndexerConstants.kD));
    SmartDashboard.putNumber("indexer IO/velocity", SmartDashboard.getNumber("indexer IO/velocity", IndexerConstants.INDEXER_VELOCITY));
    SmartDashboard.putNumber("indexer IO/voltage", SmartDashboard.getNumber("indexer/voltage", IndexerConstants.INDEXER_VOLTAGE));

    // Column
    SmartDashboard.putNumber("column IO/kP", SmartDashboard.getNumber("column IO/kP", ColumnConstants.kP));
    SmartDashboard.putNumber("column IO/kI", SmartDashboard.getNumber("column IO/kI", ColumnConstants.kI));
    SmartDashboard.putNumber("column IO/kD", SmartDashboard.getNumber("column IO/kD", ColumnConstants.kD));
    SmartDashboard.putNumber("column IO/velocity", SmartDashboard.getNumber("column IO/velocity", ColumnConstants.COLUMN_VELOCITY));
    SmartDashboard.putNumber("column IO/voltage", SmartDashboard.getNumber("column/voltage", ColumnConstants.COLUMN_VOLTAGE));

    // Roller
    SmartDashboard.putNumber("roller IO/kP", SmartDashboard.getNumber("roller IO/kP", RollerConstants.kP));
    SmartDashboard.putNumber("roller IO/kI", SmartDashboard.getNumber("roller IO/kI", RollerConstants.kI));
    SmartDashboard.putNumber("roller IO/kD", SmartDashboard.getNumber("roller IO/kD", RollerConstants.kD));
    SmartDashboard.putNumber("roller IO/velocity", SmartDashboard.getNumber("roller IO/velocity", RollerConstants.ROLLER_VELOCITY));
    SmartDashboard.putNumber("roller IO/voltage", SmartDashboard.getNumber("roller/voltage", RollerConstants.ROLLER_VOLTAGE));

    // Pivot
    SmartDashboard.putNumber("pivot IO/kP", SmartDashboard.getNumber("pivot IO/kP", PivotConstants.kP));
    SmartDashboard.putNumber("pivot IO/kI", SmartDashboard.getNumber("pivot IO/kI", PivotConstants.kI));
    SmartDashboard.putNumber("pivot IO/kD", SmartDashboard.getNumber("pivot IO/kD", PivotConstants.kD));
    SmartDashboard.putNumber("pivot IO/velocity", SmartDashboard.getNumber("pivot IO/velocity", PivotConstants.DEPLOY_VELOCITY));
    SmartDashboard.putNumber("pivot IO/voltage", SmartDashboard.getNumber("pivot/voltage", PivotConstants.DEPLOY_VOLTAGE));

    SmartDashboard.putNumber("pivot/store position", SmartDashboard.getNumber("pivot/store position", PivotConstants.STORE_POSITION_DEGREES));
    SmartDashboard.putNumber("pivot/deploy position", SmartDashboard.getNumber("pivot/deploy position", PivotConstants.DEPLOY_POSITION_DEGREES));

    // Shooter Influencer
    SmartDashboard.putNumber("shooter influencer IO/kP", SmartDashboard.getNumber("shooter influencer IO/kP", ShooterConstants.INFLUENCER_kP));
    SmartDashboard.putNumber("shooter influencer IO/kI", SmartDashboard.getNumber("shooter influencer IO/kI", ShooterConstants.INFLUENCER_kI));
    SmartDashboard.putNumber("shooter influencer IO/kD", SmartDashboard.getNumber("shooter influencer IO/kD", ShooterConstants.INFLUENCER_kD));
    SmartDashboard.putNumber("shooter influencer IO/velocity", SmartDashboard.getNumber("shooter influencer IO/velocity", ShooterConstants.OUTTAKE_MOTOR_VELOCITY));
    SmartDashboard.putNumber("shooter influencer IO/out voltage", SmartDashboard.getNumber("shooter/out voltage", ShooterConstants.OUTTAKE_MOTOR_VOLTAGE));
    SmartDashboard.putNumber("shooter influencer IO/back voltage", SmartDashboard.getNumber("shooter/back voltage", ShooterConstants.BACK_MOTOR_VOLTAGE));
  }

  private void resetControllerConstantsSmartDashboard() {
    // Indexer
    IndexerConstants.kP = SmartDashboard.getNumber("indexer IO/kP", 0);
    IndexerConstants.kI = SmartDashboard.getNumber("indexer IO/kI", 0);
    IndexerConstants.kD = SmartDashboard.getNumber("indexer IO/kD", 0);
    IndexerConstants.INDEXER_VELOCITY = SmartDashboard.getNumber("indexer IO/velocity", 0);
    IndexerConstants.INDEXER_VOLTAGE = SmartDashboard.getNumber("indexer IO/voltage", 0);

    // Column
    ColumnConstants.kP = SmartDashboard.getNumber("column IO/kP", 0);
    ColumnConstants.kI = SmartDashboard.getNumber("column IO/kI", 0);
    ColumnConstants.kD = SmartDashboard.getNumber("column IO/kD", 0);
    ColumnConstants.COLUMN_VELOCITY = SmartDashboard.getNumber("column IO/velocity", 0);
    ColumnConstants.COLUMN_VOLTAGE = SmartDashboard.getNumber("column IO/voltage", 0);

    // Roller
    RollerConstants.kP = SmartDashboard.getNumber("roller IO/kP", 0);
    RollerConstants.kI = SmartDashboard.getNumber("roller IO/kI", 0);
    RollerConstants.kD = SmartDashboard.getNumber("roller IO/kD", 0);
    RollerConstants.ROLLER_VELOCITY = SmartDashboard.getNumber("roller IO/velocity",0);
    RollerConstants.ROLLER_VOLTAGE = SmartDashboard.getNumber("roller IO/voltage", 0);

    // Pivot
    PivotConstants.kP = SmartDashboard.getNumber("pivot IO/kP", 0);
    PivotConstants.kI = SmartDashboard.getNumber("pivot IO/kI", 0);
    PivotConstants.kD = SmartDashboard.getNumber("pivot IO/kD", 0);
    PivotConstants.DEPLOY_VELOCITY = SmartDashboard.getNumber("pivot IO/velocity",0);
    PivotConstants.DEPLOY_VOLTAGE = SmartDashboard.getNumber("pivot IO/voltage", 0);
    PivotConstants.STORE_VELOCITY = -1.0*SmartDashboard.getNumber("pivot IO/velocity",0);
    PivotConstants.STORE_VOLTAGE = -1.0*SmartDashboard.getNumber("pivot IO/voltage", 0);

    PivotConstants.STORE_POSITION_DEGREES = SmartDashboard.getNumber("pivot/store position", 0);
    PivotConstants.DEPLOY_POSITION_DEGREES = SmartDashboard.getNumber("pivot/deploy position", 0);

    // Shooter
    ShooterConstants.INFLUENCER_kP = SmartDashboard.getNumber("shooter influencer IO/kP", 0);
    ShooterConstants.INFLUENCER_kI = SmartDashboard.getNumber("shooter influencer IO/kI", 0);
    ShooterConstants.INFLUENCER_kD = SmartDashboard.getNumber("shooter influencer IO/kD", 0);
    ShooterConstants.OUTTAKE_MOTOR_VELOCITY = SmartDashboard.getNumber("shooter influencer IO/velocity", 0);
    ShooterConstants.OUTTAKE_MOTOR_VOLTAGE = SmartDashboard.getNumber("shooter influencer IO/out voltage", 0);
    ShooterConstants.BACK_MOTOR_VOLTAGE = SmartDashboard.getNumber("shooter influencer IO/back voltage", 0);

    m_pivot.resetControllers();
    m_roller.resetControllers();
    m_indexer.resetControllers();
    m_column.resetControllers();
    m_shooter.resetControllers();
  }


  // ----- AUTONOMOUS -----
  /**
   * Define named commands for autonomous paths
   */
  private void createNamedCommands() {
    // TODO -- REPLACE WITH PROPER COMMAND ONCE IT HAS BEEN WRITTEN 
    NamedCommands.registerCommand("trigger intake", new WaitCommand(1));
    NamedCommands.registerCommand("snap to hub", m_drivetrainCommandFactory.snapToAngle(
      () -> new JoystickVals(0, 0), 
      HubCalculations.angleToHub(m_drivetrain.getState().Pose).getRadians())
      .withTimeout(1.5));
    NamedCommands.registerCommand("climb", new WaitCommand(1));
    NamedCommands.registerCommand("shoot", m_shooterCommandFactory.shooterSmartDashboardVelocityCommand("auto shoot", ShooterConstants.OUTTAKE_MOTOR_VELOCITY).withTimeout(5));
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
      SmartDashboard.putNumberArray("fusedVision/" + reading.cameraName() + "/drivetrainBeforeUpdate", new double [] {
      m_drivetrain.getState().Pose.getX(), m_drivetrain.getState().Pose.getY(), m_drivetrain.getState().Pose.getRotation().getRadians()});

      // Update fusedPose
      m_drivetrain.setVisionMeasurementStdDevs(reading.stdDevs());
      m_drivetrain.addVisionMeasurement(robotPose.estimatedPose.toPose2d(), robotPose.timestampSeconds);

      // sample drivetrain fusedPose after updating
      SmartDashboard.putNumberArray("fusedVision/" + reading.cameraName() + "/drivetrainAfterUpdate", new double [] {
        m_drivetrain.getState().Pose.getX(), m_drivetrain.getState().Pose.getY(), m_drivetrain.getState().Pose.getRotation().getRadians()});
    }
    
    m_actualField.setRobotPose(m_drivetrain.getState().Pose);
    SmartDashboard.putData("fusedVision/" + "actual field/", m_actualField);
  }
/*   public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
    // The pose by maplesim, including collisions with the field. 
    // See https://www.chiefdelphi.com/t/simulated-robot-goes-through-walls-with-maplesim/508663.

  } */
}
