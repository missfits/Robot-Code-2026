// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainCommandFactory;
import frc.robot.subsystems.intake.IntakeCommandFactory;
import frc.robot.subsystems.scorer.ScorerCommandFactory;
import frc.robot.subsystems.vision.LocalizationCamera;
import frc.robot.subsystems.vision.VisionSubsystem;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.WaitCommand;


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
    //Pathplanner register named commands
    //TO-DO -- REPLACE WITH PROPER COMMAND ONCE IT HAS BEEN WRITTEN 
    NamedCommands.registerCommand("trigger intake", new WaitCommand(1));
    NamedCommands.registerCommand("orient to hub", new WaitCommand(1));
    NamedCommands.registerCommand("climb", new WaitCommand(1));

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

    // Drive in slowmode while right trigger is pressed
    m_driverJoystick.rightTrigger().whileTrue(
      m_drivetrainCommandFactory.defaultDrive(m_driverJoystick, () -> true)
    );

    // TODO: change -- this is for testing 
    m_driverJoystick.a().whileTrue(
      m_drivetrainCommandFactory.snapToAngle(m_driverJoystick, 0)
    );

    m_driverJoystick.povCenter().negate().onTrue(new InstantCommand(() -> resetControllerConstantsSmartDashboard()));


    m_drivetrain.registerTelemetry(logger::telemeterize);
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
    List<LocalizationCamera> cameras = m_vision.getLocalizationCameras();

    for (LocalizationCamera cam : cameras){
      updatePoseEst(cam);
    }
    
    m_actualField.setRobotPose(m_drivetrain.getState().Pose);
    SmartDashboard.putData("fusedVision/" + "actual field/", m_actualField);
  }

  public void updatePoseEst(LocalizationCamera camera){
    var optionalReading = camera.getCameraReading();
    if (!optionalReading.isPresent()) {
      return;
    }
    var cameraReading = optionalReading.get();
    EstimatedRobotPose robotPose = cameraReading.robotPose();

    Pose3d estPose3d = robotPose.estimatedPose; // estimated robot pose of vision
    Pose2d estPose2d = estPose3d.toPose2d();

    // check if new estimated pose and previous pose are less than 2 meters apart (fused poseEst)
    double distance = estPose2d.getTranslation().getDistance(m_drivetrain.getState().Pose.getTranslation());

    SmartDashboard.putNumber("fusedVision/" + camera.getCameraName() + "/distanceBetweenVisionAndActualPose", distance);

    SmartDashboard.putBoolean("fusedVision/" + camera.getCameraName() + "/areRecentCameraPosesConsistent", camera.areRecentCameraPosesConsistent());

    SmartDashboard.putString("fusedVision/" + camera.getCameraName() + "/filterState", "distance-filtering");

    /*
     * Only accepts vision measurement from ONE CAMERA if distance between estimated vision pose
     *  and previous fused pose is less than MAX_VISION_POSE_DISTANCE
     * OR if the last three vision poses from ONE CAMERA are consistent with each other.
     */
    if (distance < VisionConstants.MAX_VISION_POSE_DISTANCE || camera.areRecentCameraPosesConsistent()) {
      m_drivetrain.setVisionMeasurementStdDevs(cameraReading.stdDevs());

      // sample drivetrain fusedPose before updating
      Optional<Pose2d> samplePose = m_drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(robotPose.timestampSeconds));

      if (samplePose.isPresent()){
        SmartDashboard.putNumberArray("fusedVision/" + camera.getCameraName() + "/samplePose",  new double [] {
          samplePose.get().getX(), samplePose.get().getY(), samplePose.get().getRotation().getRadians()});
      }
    
      SmartDashboard.putNumberArray("fusedVision/" + camera.getCameraName() + "/drivetrainBeforeUpdate", new double [] {
      m_drivetrain.getState().Pose.getX(), m_drivetrain.getState().Pose.getY(), m_drivetrain.getState().Pose.getRotation().getRadians()});

    
      m_drivetrain.addVisionMeasurement(estPose2d, robotPose.timestampSeconds);
      camera.updateField(estPose2d);

      // sample drivetrain fusedPose after updating
      SmartDashboard.putNumberArray("fusedVision/" + camera.getCameraName() + "/drivetrainAfterUpdate", new double [] {
        m_drivetrain.getState().Pose.getX(), m_drivetrain.getState().Pose.getY(), m_drivetrain.getState().Pose.getRotation().getRadians()});
        
      SmartDashboard.putNumberArray("fusedVision/" + camera.getCameraName() + "/visionPose2dFiltered" + camera.getCameraName(), new double[] {estPose2d.getX(), estPose2d.getY(), estPose2d.getRotation().getRadians()});

      SmartDashboard.putNumberArray("fusedVision/" + camera.getCameraName() + "/visionPose3D", new double[] {
        estPose3d.getX(),
        estPose3d.getY(),
        estPose3d.getZ(),
        estPose3d.getRotation().toRotation2d().getRadians()
      }); // post vision 3d to smartdashboard
      SmartDashboard.putString("fusedVision/" + camera.getCameraName() + "/filterState", "success");
    } else {
        SmartDashboard.putString("fusedVision/" + camera.getCameraName() + "/filterState", "failed-distance-filtering");
    }
  }
}
