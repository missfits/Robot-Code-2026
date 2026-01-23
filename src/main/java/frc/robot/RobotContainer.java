// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainCommandFactory;
import frc.robot.subsystems.vision.LocalizationCamera;
import frc.robot.subsystems.vision.LocalizationCamera.CameraReading;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionUtils;
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

    // TODO: change -- this is for testing 
    m_driverJoystick.a().whileTrue(
      m_drivetrainCommandFactory.snapToAngle(m_driverJoystick, 0)
    );

    m_drivetrain.registerTelemetry(logger::telemeterize);

    // --- VISION FILTERING ---
    // sets global filtering logic for vision readings
    m_vision.setFilter((reading, camera) -> {
      return VisionUtils.poseIsSane(reading.robotPose().estimatedPose) && filterPoseEst(reading, camera);
    });
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
  
  public boolean filterPoseEst(CameraReading reading, LocalizationCamera camera){
    EstimatedRobotPose robotPose = reading.robotPose();

    Pose3d estPose3d = robotPose.estimatedPose; // estimated robot pose of vision
    Pose2d estPose2d = estPose3d.toPose2d();

    // check if new estimated pose and previous pose are less than 2 meters apart (fused poseEst)
    double distance = estPose2d.getTranslation().getDistance(m_drivetrain.getState().Pose.getTranslation());
    /*
     * Only accepts vision measurement from ONE CAMERA if distance between estimated vision pose
     *  and previous fused pose is less than MAX_VISION_POSE_DISTANCE
     * OR if the last three vision poses from ONE CAMERA are consistent with each other.
     * 
     */
    if (distance < VisionConstants.MAX_VISION_POSE_DISTANCE || camera.areRecentCameraPosesConsistent()) {
      // sample drivetrain fusedPose before updating
      Optional<Pose2d> samplePose = m_drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(robotPose.timestampSeconds));

      if (samplePose.isPresent()){
        SmartDashboard.putNumberArray("fusedVision/" + reading.cameraName() + "/samplePose",  new double [] {
          samplePose.get().getX(), samplePose.get().getY(), samplePose.get().getRotation().getRadians()});
      }
       
      // sample drivetrain fusedPose after updating
      SmartDashboard.putNumberArray("fusedVision/" + reading.cameraName() + "/drivetrainAfterUpdate", new double [] {
        m_drivetrain.getState().Pose.getX(), m_drivetrain.getState().Pose.getY(), m_drivetrain.getState().Pose.getRotation().getRadians()});
        
      SmartDashboard.putNumberArray("fusedVision/" + reading.cameraName() + "/visionPose2dFiltered" + camera.getCameraName(), new double[] {estPose2d.getX(), estPose2d.getY(), estPose2d.getRotation().getRadians()});

      SmartDashboard.putNumberArray("fusedVision/" + reading.cameraName() + "/visionPose3D", new double[] {
        estPose3d.getX(),
        estPose3d.getY(),
        estPose3d.getZ(),
        estPose3d.getRotation().toRotation2d().getRadians()
      }); // post vision 3d to smartdashboard
      SmartDashboard.putString("fusedVision/" + reading.cameraName() + "/filterState", "success");
      camera.updateField(estPose2d); // log immediately after filtering (CHANGE LATER!)
      return true;
    } else {
        SmartDashboard.putString("fusedVision/" + reading.cameraName() + "/filterState", "failed-distance-filtering");
        return false;
    }
  }

  public void updatePoseEst() {
    for (CameraReading reading : m_vision.getCameraReadings()) {
      m_drivetrain.setVisionMeasurementStdDevs(reading.stdDevs());

      SmartDashboard.putNumberArray("fusedVision/" + reading.cameraName() + "/drivetrainBeforeUpdate", new double [] {
        m_drivetrain.getState().Pose.getX(), m_drivetrain.getState().Pose.getY(), m_drivetrain.getState().Pose.getRotation().getRadians()});
      
      m_drivetrain.addVisionMeasurement(reading.robotPose().estimatedPose.toPose2d(), reading.robotPose().timestampSeconds);
    }

    m_actualField.setRobotPose(m_drivetrain.getState().Pose);
    SmartDashboard.putData("fusedVision/" + "actual field/", m_actualField);
  }
}
