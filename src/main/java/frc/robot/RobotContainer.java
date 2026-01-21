// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainCommandFactory;
import frc.robot.subsystems.vision.LocalizationCamera;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drivetrain.Telemetry;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
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
  public final VisionSubsystem m_vision = new VisionSubsystem();
  
  // Command factories
  private final DrivetrainCommandFactory m_drivetrainCommandFactory = new DrivetrainCommandFactory(m_drivetrain);

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

    m_drivetrain.registerTelemetry(logger::telemeterize);
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
