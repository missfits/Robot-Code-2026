// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // Joystick deadband values
    public static final double DRIVE_JOYSTICK_DEADBAND = 0.1;
    public static final double STEER_JOYSTICK_DEADBAND = 0.1;

    // Slowmode factor for reduced speed control
    public static final double SLOWMODE_FACTOR = 0.3;
  }

  public static class DrivetrainConstants {
    // Steer motor PID and feedforward gains
    public static double STEER_KP = 100;
    public static double STEER_KI = 0;
    public static double STEER_KD = 0.5;
    public static double STEER_KS = 0.1;
    public static double STEER_KV = 2.66;
    public static double STEER_KA = 0;

    // Drive motor PID and feedforward gains
    public static double DRIVE_KP = 0.1;
    public static double DRIVE_KI = 0;
    public static double DRIVE_KD = 0;
    public static double DRIVE_KS = 0;
    public static double DRIVE_KV = 0.124;
    public static double DRIVE_KA = 0;

    public static double WHEEL_RADIUS_FUDGE_FACTOR = 1.0;


    // Max speeds for drivetrain
    public static final double MAX_TRANSLATION_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MAX_ROTATION_SPEED = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Rotation heading controller PID gains
    public static double ROTATION_KP = 5.0;
    public static double ROTATION_KI = 0.0;
    public static double ROTATION_KD = 0.0;

    // PID constants for PathPlanner AutoBuilder
    public static double ROBOT_POSITION_P = 5.0;
    public static double ROBOT_POSITION_I = 0;
    public static double ROBOT_POSITION_D = 0;
    public static double ROBOT_ROTATION_P = 5.0;
    public static double ROBOT_ROTATION_I = 0;
    public static double ROBOT_ROTATION_D = 0;

    static {
      switch (RobotConfig.getRobot()) {
        case CLEO:
          STEER_KP = 100;
          STEER_KI = 0;
          STEER_KD = 0.5;
          STEER_KS = 0.1;
          STEER_KV = 2.66;
          STEER_KA = 0;

          DRIVE_KP = 0.1;
          DRIVE_KI = 0;
          DRIVE_KD = 0;
          DRIVE_KS = 0;
          DRIVE_KV = 0.124;
          DRIVE_KA = 0;

          WHEEL_RADIUS_FUDGE_FACTOR = 1.0;

          ROTATION_KP = 5.0;
          ROTATION_KI = 0.0;
          ROTATION_KD = 0.0;

          ROBOT_POSITION_P = 5.0;
          ROBOT_POSITION_I = 0;
          ROBOT_POSITION_D = 0;
          ROBOT_ROTATION_P = 5.0;
          ROBOT_ROTATION_I = 0;
          ROBOT_ROTATION_D = 0;
          break;

        case CERIDWEN:
          // TODO: Tune these values for Ceridwen
          STEER_KP = 100;
          STEER_KI = 0;
          STEER_KD = 0.5;
          STEER_KS = 0.1;
          STEER_KV = 2.66;
          STEER_KA = 0;

          DRIVE_KP = 0.1;
          DRIVE_KI = 0;
          DRIVE_KD = 0;
          DRIVE_KS = 0;
          DRIVE_KV = 0.124;
          DRIVE_KA = 0;

          WHEEL_RADIUS_FUDGE_FACTOR = 1.0;

          ROTATION_KP = 5.0;
          ROTATION_KI = 0.0;
          ROTATION_KD = 0.0;

          ROBOT_POSITION_P = 5.0;
          ROBOT_POSITION_I = 0;
          ROBOT_POSITION_D = 0;
          ROBOT_ROTATION_P = 5.0;
          ROBOT_ROTATION_I = 0;
          ROBOT_ROTATION_D = 0;
          break;

        default:
          throw new IllegalStateException("Unknown robot type: " + RobotConfig.getRobot());
      }
    }

    /* Universal drivetrain constants (not robot-dependent) */

    // Snap to target distance threshold (meters)
    // If robot is within this distance of target, maintain current heading
    public static final double SNAP_TO_TARGET_DISTANCE_THRESHOLD = 0.05; // 5cm

  }

  public static class AngularMechanismConstants {
    public static final int MECHANISM_MOTOR_ID = 0;
    public static final int MOTOR_STATOR_LIMIT = 0;

    public static final double DEGREES_PER_ROTATION = 0;
  }

  public static class LinearMechanismConstants {
    public static final int MECHANISM_MOTOR_ID = 0;
    public static final int MOTOR_STATOR_LIMIT = 0;

    public static final double METERS_PER_ROTATION = 0;
  }
  
  public static class VisionConstants {
    // --- vision utils ---
    public static final double MAX_VISION_POSE_DISTANCE = 1;
    public static final double MAX_VISION_POSE_Z = 0.1;
    public static final double MAX_VISION_POSE_ROLL = 0.05; // in radians
    public static final double MAX_VISION_POSE_PITCH = 0.05; // in radians

    // --- localization camera ---
    // default vision standard deviation
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(6, 6, 4);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 0.3);

    public static final double VISION_DISTANCE_DISCARD = 10; 
    public static final double MAX_POSE_AMBIGUITY = 0.2;
    public static final double MAX_AVG_DIST_BETWEEN_LAST_EST_POSES = 0.3; // in meters 
    public static final double MAX_AVG_SPEED_BETWEEN_LAST_EST_POSES = MAX_AVG_DIST_BETWEEN_LAST_EST_POSES * 50.;
    public static final int NUM_LAST_EST_POSES = 3;
    public static final double STD_DEV_SCALER = 30;

    // --- vision subsystem ---
    // (camera setup)
    public static final String CAMERA1_NAME;
    public static final String CAMERA2_NAME;

    // Camera 1 position - robot-specific because camera mounting may differ
    public static final double ROBOT_TO_CAM1_X;
    public static final double ROBOT_TO_CAM1_Y;
    public static final double ROBOT_TO_CAM1_Z;
    public static final double ROBOT_TO_CAM1_ROLL;
    public static final double ROBOT_TO_CAM1_PITCH;
    public static final double ROBOT_TO_CAM1_YAW;
    public static final Transform3d ROBOT_TO_CAM1_3D;

    // Camera 2 position - robot-specific
    public static final double ROBOT_TO_CAM2_X;
    public static final double ROBOT_TO_CAM2_Y;
    public static final double ROBOT_TO_CAM2_Z;
    public static final double ROBOT_TO_CAM2_ROLL;
    public static final double ROBOT_TO_CAM2_PITCH;
    public static final double ROBOT_TO_CAM2_YAW;
    public static final Transform3d ROBOT_TO_CAM2_3D;

    static {
      switch (RobotConfig.getRobot()) {
        case CLEO:
          // TODO: Measure and update these values for Cleo

          CAMERA1_NAME = "camera1";
          CAMERA2_NAME = "camera2";

          // Cleo camera positions
          ROBOT_TO_CAM1_X = 0;
          ROBOT_TO_CAM1_Y = 0;
          ROBOT_TO_CAM1_Z = 0;
          ROBOT_TO_CAM1_ROLL = 0;
          ROBOT_TO_CAM1_PITCH = 0;
          ROBOT_TO_CAM1_YAW = 0;

          ROBOT_TO_CAM2_X = 0;
          ROBOT_TO_CAM2_Y = 0;
          ROBOT_TO_CAM2_Z = 0;
          ROBOT_TO_CAM2_ROLL = 0;
          ROBOT_TO_CAM2_PITCH = 0;
          ROBOT_TO_CAM2_YAW = 0;
          break;

        case CERIDWEN:
          // Ceridwen camera positions

          CAMERA1_NAME = "camera1";
          CAMERA2_NAME = "camera2";

          ROBOT_TO_CAM1_X = Units.inchesToMeters(2);
          ROBOT_TO_CAM1_Y = Units.inchesToMeters(-7);
          ROBOT_TO_CAM1_Z = Units.inchesToMeters(8);
          ROBOT_TO_CAM1_ROLL = Units.degreesToRadians(-35.26);
          ROBOT_TO_CAM1_PITCH = Units.degreesToRadians(30);
          ROBOT_TO_CAM1_YAW = Units.degreesToRadians(45);

          ROBOT_TO_CAM2_X = 0;
          ROBOT_TO_CAM2_Y = 0;
          ROBOT_TO_CAM2_Z = 0;
          ROBOT_TO_CAM2_ROLL = 0;
          ROBOT_TO_CAM2_PITCH = 0;
          ROBOT_TO_CAM2_YAW = 0;
          break;

        default:
          throw new IllegalStateException("Unknown robot type: " + RobotConfig.getRobot());
      }

      // Compute Transform3d after switch (same for all robots)
      ROBOT_TO_CAM1_3D = new Transform3d(
        new Translation3d(ROBOT_TO_CAM1_X, ROBOT_TO_CAM1_Y, ROBOT_TO_CAM1_Z),
        new Rotation3d(ROBOT_TO_CAM1_ROLL, ROBOT_TO_CAM1_PITCH, ROBOT_TO_CAM1_YAW)
      );

      ROBOT_TO_CAM2_3D = new Transform3d(
        new Translation3d(ROBOT_TO_CAM2_X, ROBOT_TO_CAM2_Y, ROBOT_TO_CAM2_Z),
        new Rotation3d(ROBOT_TO_CAM2_ROLL, ROBOT_TO_CAM2_PITCH, ROBOT_TO_CAM2_YAW)
      );
    }
  }
  
  public static class LEDConstants { // placeholder constants
    public static final int KPORT = 0;
    public static final int KLENGTH = 60;

    public static final double BLINK_TIME = 1; // in seconds for after intake/outtake
  }
}
