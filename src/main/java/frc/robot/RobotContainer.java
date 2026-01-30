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
import frc.robot.subsystems.scorer.ScorerCommandFactory;
import frc.robot.subsystems.vision.LocalVisionFilterPipeline;
import frc.robot.subsystems.vision.GlobalVisionFilterPipeline;
import frc.robot.subsystems.vision.LocalizationCamera;
import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.filtering.GlobalCrossCameraConsensusFilter;
import frc.robot.subsystems.vision.filtering.LocalCameraPoseConsistencyFilter;
import frc.robot.subsystems.vision.filtering.LocalDistanceToFusedPoseFilter;
import frc.robot.subsystems.vision.filtering.LocalPoseZRollPitchFilter;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.intake.RollerSubsystem;
import frc.robot.subsystems.scorer.ShooterSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ScorerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static record JoystickVals(double x, double y) {}

  private final SendableChooser<Command> m_autoChooser; // Sendable chooser that holds the autos
  private final Telemetry logger = new Telemetry(DrivetrainConstants.MAX_TRANSLATION_SPEED);

  // Subsystems
  public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  public final RollerSubsystem m_roller = new RollerSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final VisionSubsystem m_vision = new VisionSubsystem();
  
  // Command factories
  private final DrivetrainCommandFactory m_drivetrainCommandFactory = new DrivetrainCommandFactory(m_drivetrain);
  private final IntakeCommandFactory m_intakeCommandFactory = new IntakeCommandFactory(m_roller);
  private final ScorerCommandFactory m_shooterCommandFactory = new ScorerCommandFactory(m_shooter);

  private final CommandXboxController m_driverJoystick =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Field2d m_actualField = new Field2d(); // field simulation

  /** The container for the robot. Contains subsystems and commands. */
  public RobotContainer() {
    // Configure trigger bindings
    configureBindings();


    // Configure auto builder
    createNamedCommands();
    m_autoChooser = AutoBuilder.buildAutoChooser("drive forward 1m");
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Data logging
    DataLogManager.start(); // Starts recording to data log
    DriverStation.startDataLog(DataLogManager.getLog()); // Record both DS control and joystick data
    DriverStation.silenceJoystickConnectionWarning(true); // Turn off unplugged joystick errors

    logToSmartDashboard();
  }

  /**
   * Define trigger -> command mappings
   */
  private void configureBindings() {
    // Default drive
    m_drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      m_drivetrainCommandFactory.defaultDrive(m_driverJoystick, () -> false)
    );

    // Drive in slowmode while right bumper is pressed
    m_driverJoystick.rightBumper().whileTrue(
      m_drivetrainCommandFactory.defaultDrive(m_driverJoystick, () -> true)
    );

    if (Utils.isSimulation()){
      Consumer<SwerveDriveState> telemetry =  ((CommandSwerveDrivetrainSim) m_drivetrain)
          .getSimTelemetryConsumer().andThen(logger::telemeterize);
      m_drivetrain.registerTelemetry(telemetry);
    }
    else{
       m_drivetrain.registerTelemetry(logger::telemeterize);
    }
   
    // TODO: change -- this is for testing 
    m_driverJoystick.y().whileTrue(
      m_drivetrainCommandFactory.snapToAngle(m_driverJoystick, 0)
    );

    // TODO: change -- this is for testing 
    m_driverJoystick.b().onTrue(m_drivetrainCommandFactory.snapToTarget(m_driverJoystick, () -> new Pose2d(Units.inchesToMeters(182), Units.inchesToMeters(182), new Rotation2d())));


    // reset the field-centric heading on a button press
    m_driverJoystick.a().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.resetRotation(new Rotation2d(DriverStation.getAlliance().equals(Alliance.Blue) ? 0 : Math.PI))));


    m_driverJoystick.povCenter().negate().onTrue(new InstantCommand(() -> resetControllerConstantsSmartDashboard()));


    // --- CONFIGURE VISION FILTERING ---
    GlobalVisionFilterPipeline globalPipeline = new GlobalVisionFilterPipeline();
    LocalVisionFilterPipeline localPipeline = new LocalVisionFilterPipeline();

    // Add global filters
    globalPipeline.addFilter("crossCameraConsensus", new GlobalCrossCameraConsensusFilter());

     // Add local filters
    localPipeline.addFilter("poseZRollPitch", new LocalPoseZRollPitchFilter());
    localPipeline.addFilter("cameraPoseConsistency", new LocalCameraPoseConsistencyFilter());
    localPipeline.addFilter("distanceToFusedPose", new LocalDistanceToFusedPoseFilter(m_drivetrain));

    m_vision.setGlobalFilterPipeline(globalPipeline);
    m_vision.setLocalFilteringPipeline(localPipeline);
  }

  private void logToSmartDashboard() {
    // Roller
    SmartDashboard.putNumber("roller IO/kP", SmartDashboard.getNumber("roller IO/kP", IntakeConstants.ROLLER_kP));
    SmartDashboard.putNumber("roller IO/kI", SmartDashboard.getNumber("roller IO/kI", IntakeConstants.ROLLER_kI));
    SmartDashboard.putNumber("roller IO/kD", SmartDashboard.getNumber("roller IO/kD", IntakeConstants.ROLLER_kD));
    SmartDashboard.putNumber("roller IO/velocity", SmartDashboard.getNumber("roller IO/velocity", IntakeConstants.OUTTAKE_MOTOR_VELOCITY));

    // Shooter Influencer 
    SmartDashboard.putNumber("shooter influencer IO/kP", SmartDashboard.getNumber("shooter influencer IO/kP", ScorerConstants.INFLUENCER_kP));
    SmartDashboard.putNumber("shooter influencer IO/kI", SmartDashboard.getNumber("shooter influencer IO/kI", ScorerConstants.INFLUENCER_kI));
    SmartDashboard.putNumber("shooter influencer IO/kD", SmartDashboard.getNumber("shooter influencer IO/kD", ScorerConstants.INFLUENCER_kD));
    SmartDashboard.putNumber("shooter influencer IO/velocity", SmartDashboard.getNumber("shooter influencer IO/velocity", ScorerConstants.INFLUENCER_OUTTAKE_MOTOR_VELOCITY));

    // Shooter Follower 
    SmartDashboard.putNumber("shooter follower IO/kP", SmartDashboard.getNumber("shooter follower IO/kP", ScorerConstants.FOLLOWER_kP));
    SmartDashboard.putNumber("shooter follower IO/kI", SmartDashboard.getNumber("shooter follower IO/kI", ScorerConstants.FOLLOWER_kI));
    SmartDashboard.putNumber("shooter follower IO/kD", SmartDashboard.getNumber("shooter follower IO/kD", ScorerConstants.FOLLOWER_kD));
    SmartDashboard.putNumber("shooter follower IO/velocity", SmartDashboard.getNumber("shooter follower IO/velocity", ScorerConstants.FOLLOWER_OUTTAKE_MOTOR_VELOCITY));
  }

  private void resetControllerConstantsSmartDashboard() {
    IntakeConstants.ROLLER_kP = SmartDashboard.getNumber("roller IO/kP", 0);
    IntakeConstants.ROLLER_kI = SmartDashboard.getNumber("roller IO/kI", 0);
    IntakeConstants.ROLLER_kD = SmartDashboard.getNumber("roller IO/kD", 0);
    IntakeConstants.OUTTAKE_MOTOR_VELOCITY = SmartDashboard.getNumber("roller IO/velocity", 0);

    ScorerConstants.INFLUENCER_kP = SmartDashboard.getNumber("shooter influencer IO/kP", 0);
    ScorerConstants.INFLUENCER_kI = SmartDashboard.getNumber("shooter influencer IO/kI", 0);
    ScorerConstants.INFLUENCER_kD = SmartDashboard.getNumber("shooter influencer IO/kD", 0);
    ScorerConstants.INFLUENCER_OUTTAKE_MOTOR_VELOCITY = SmartDashboard.getNumber("shooter influencer IO/velocity", 0);

    ScorerConstants.FOLLOWER_kP = SmartDashboard.getNumber("shooter follower IO/kP", 0);
    ScorerConstants.FOLLOWER_kI = SmartDashboard.getNumber("shooter follower IO/kI", 0);
    ScorerConstants.FOLLOWER_kD = SmartDashboard.getNumber("shooter follower IO/kD", 0);
    ScorerConstants.FOLLOWER_OUTTAKE_MOTOR_VELOCITY = SmartDashboard.getNumber("shooter follower IO/velocity", 0);

    m_roller.resetControllers();
    m_shooter.resetControllers();

  }

  /**
   * Define named commands for autonomous paths
   */
  private void createNamedCommands() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }


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
