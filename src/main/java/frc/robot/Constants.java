// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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
    public static final double STEER_KP = 100;
    public static final double STEER_KI = 0;
    public static final double STEER_KD = 0.5;
    public static final double STEER_KS = 0.1;
    public static final double STEER_KV = 2.66;
    public static final double STEER_KA = 0;

    // Drive motor PID and feedforward gains
    public static final double DRIVE_KP = 0.1;
    public static final double DRIVE_KI = 0;
    public static final double DRIVE_KD = 0;
    public static final double DRIVE_KS = 0;
    public static final double DRIVE_KV = 0.124;
    public static final double DRIVE_KA = 0;

    public static final double WHEEL_RADIUS_FUDGE_FACTOR = 1.0;

    // Max speeds for drivetrain
    public static final double MAX_TRANSLATION_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MAX_ROTATION_SPEED = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Rotation heading controller PID gains
    public static final double ROTATION_KP = 10.0;
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.0;
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
    public static final double MAX_VISION_POSE_DISTANCE = 1; // for vision (fusedpose) vs odometry
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
    public static final String CAMERA1_NAME = null;
    public static final String CAMERA2_NAME = null;
    public static final Transform3d ROBOT_TO_CAM1_3D = null;
    public static final Transform3d ROBOT_TO_CAM2_3D = null;


    public static final int MIN_NUM_CAMERA_READINGS = 2; // min number of diff camera readings for "jumpy" check
    public static final double MAX_VISION_READING_DISTANCE = 0.5; // max distance between two camera readings
  }
  
  public static class LEDConstants { // placeholder constants
    public static final int KPORT = 0;
    public static final int KLENGTH = 60;

    public static final double BLINK_TIME = 1; // in seconds for after intake/outtake
  }
}
