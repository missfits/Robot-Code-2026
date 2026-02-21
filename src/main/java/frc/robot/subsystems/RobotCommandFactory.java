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
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer.JoystickVals;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainCommandFactory;
import frc.robot.subsystems.intake.ColumnSubsystem;
import frc.robot.subsystems.intake.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeCommandFactory;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.intake.RollerSubsystem;
import frc.robot.subsystems.scorer.ScorerCommandFactory;
import frc.robot.subsystems.scorer.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.HubCalculations;
import frc.robot.utils.ShooterLookupTable;

public class RobotCommandFactory {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final RollerSubsystem m_roller;
    private final ShooterSubsystem m_shooter;
    private final VisionSubsystem m_vision;
    private final IndexerSubsystem m_indexer;
    private final PivotSubsystem m_pivot;
    private final ColumnSubsystem m_column;
    private final LaserCANSensorBase m_intakeSensor;
    private final LaserCANSensorBase m_shooterSensor;
    private final DrivetrainCommandFactory m_drivetrainCommandFactory; 
    private final IntakeCommandFactory m_intakeCommandFactory;
    private final ScorerCommandFactory m_shooterCommandFactory;

    private final Supplier<Double> m_shooterVelocitySupplier = () -> calculateShooterVelocity(); 
    
        public RobotCommandFactory(CommandSwerveDrivetrain drivetrain, RollerSubsystem roller,
                ShooterSubsystem shooter, VisionSubsystem vision, IndexerSubsystem indexer, PivotSubsystem pivot,
                ColumnSubsystem column, LaserCANSensorBase intakeSensor, LaserCANSensorBase shooterSensor, 
                DrivetrainCommandFactory drivetrainCommandFactory, IntakeCommandFactory intakeCommandFactory, ScorerCommandFactory shooterCommandFactory) {
            m_drivetrain = drivetrain;
            m_roller = roller;
            m_shooter = shooter;
            m_vision = vision;
            m_indexer = indexer;
            m_pivot = pivot;
            m_column = column;
            m_intakeSensor = intakeSensor;
            m_shooterSensor = shooterSensor;
            m_drivetrainCommandFactory = drivetrainCommandFactory;
            m_intakeCommandFactory = intakeCommandFactory; 
            m_shooterCommandFactory = shooterCommandFactory;
        }

    private Double calculateShooterVelocity() {
        // Calculate distance from hub
        Pose2d robotPose = m_drivetrain.getState().Pose;
        double distanceToHub = HubCalculations.distanceToHub(robotPose, DriverStation.getAlliance().equals(Alliance.Blue));

        // Look up target velocity from distance
        Optional<Double> velocityOptional = ShooterLookupTable.getVelocityForDistance(distanceToHub);

        SmartDashboard.putNumber("robot/distanceToHub", distanceToHub);

        if (velocityOptional.isPresent()) {
            return velocityOptional.get();
        } else {
        return 0.0; // TODO: fix, based on robot mode?? 
        }
    }

  /**
   * Command that shoots based on distance to hub using vision
  */
  public Command shootByDistanceCommand(Supplier<JoystickVals> joystickValsSupplier) {
    return Commands.parallel(
      m_shooterCommandFactory.runShooterVelocity(m_shooterVelocitySupplier), // run shooter at velocity  
      Commands.sequence(
        m_column.runMechanismOff() // column: wait until 
          .until(m_shooter.atTargetVelocityTrigger() // shooter at target velocity 
            .and(m_drivetrainCommandFactory.atTargetAngleTrigger())), // and drivetrain at target angle
        m_column.runMechanismPID(ColumnConstants.COLUMN_VELOCITY)),
      m_drivetrainCommandFactory.snapToAngle( // drivetrain: snap to angle 
        joystickValsSupplier,
        () -> HubCalculations.angleToHub(m_drivetrain.getState().Pose, DriverStation.getAlliance().equals(Alliance.Blue)))
    ).withName("shootByDistance");
  }
}

