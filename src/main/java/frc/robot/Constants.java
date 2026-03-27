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
    public static final int kOperatorControllerPort = 1;
    public static final int kTestControllerPort = 2;

    // Joystick deadband values
    public static final double DRIVE_JOYSTICK_DEADBAND = 0.1;
    public static final double STEER_JOYSTICK_DEADBAND = 0.1;

    // Slowmode factor for reduced speed control
    public static final double SLOWMODE_FACTOR = 0.6;
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

    public static double WHEEL_RADIUS_FUDGE_FACTOR = 1.0/1.05;

    // Max speeds for drivetrain
    public static final double MAX_TRANSLATION_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MAX_ROTATION_SPEED = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a revolution per second max angular velocity

    // Rotation heading controller PID gains
    public static double ROTATION_KP = 7.0;
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

    // angle tolerance (in radians) for atTargetAngle()
    public static final double ANGLE_TOLERANCE = Math.toRadians(5);

  }

  public static class PivotConstants {
    // Velocity voltage constants
    public static final boolean ENABLE_FOC = false;
    public static final double FEED_FORWARD = 0.0;
    public static final int SLOT = 0;
    public static final boolean OVERRIDE_BRAKE_DUR_NEUTRAL = false;
    public static final double CURRENT_THRESHOLD = 40;
    public static final double AUTO_CURRENT_THRESHOLD = 60;


    // Motor ID
    public static final int MOTOR_ID = 20;

    // Motor limits
    public static final int MOTOR_STATOR_LIMIT = 40;

    public static final boolean IS_INVERTED = true;

    // Duty cycle limits
    public static final double PEAK_FORWARD_DUTY_CYCLE = 1;
    public static final double PEAK_REVERSE_DUTY_CYCLE = -1;

    // Conversions
    public static final double DEGREES_PER_REVOLUTION = 360./25.*18./30.;

    // Pivot volts
    public static double DEPLOY_VOLTAGE = 3.0;
    public static double STORE_VOLTAGE = -1.0*DEPLOY_VOLTAGE;
    public static double MAX_VOLTAGE = 6.0;
    public static double ZERO_PIVOT_VOLTAGE = 1.5;
    public static double AUTO_ZERO_PIVOT_VOLTAGE = 1.5;


    // Pivot motor velocities
    public static double DEPLOY_VELOCITY = 1.0;
    public static double STORE_VELOCITY = -1.0*DEPLOY_VELOCITY;
    public static final double MAX_VELOCITY = 15.0;

    // Testing velocities
    public static double TESTING_VELOCITY = 0.0;

    // Pivot positions
    public static double STORE_POSITION_DEGREES = 10;
    public static double DISPLACE_FUEL_POSITION_DEGREES = 10;
    public static double DEPLOY_POSITION_DEGREES = 55;
    public static double RESET_DEPLOY_POSITION_DEGREES = DEPLOY_POSITION_DEGREES + 10;
    public static double AUTO_RESET_DEPLOY_POSITION_DEGREES = DEPLOY_POSITION_DEGREES;

    // Timing values for displace fuel command 
    public static final double DISPLACE_FUEL_UP_TIMEOUT = 0.75; // TODO: tune
    public static final double DISPLACE_FUEL_DOWN_TIMEOUT = 0.75; 
    public static final double DISPLACE_FUEL_DELAY = 0; // time between repeats of displace fuel command

    // Max manual volts
    
    // PID gains
    public static double kP = 0.3;
    public static double kI = 0;
    public static double kD = 0;

    // Feed forward values
    public static double kS = 0.25;
    public static double kG = 0.25;
    public static double kV = 0.12;
    public static double kA = 0.0;

    // Motion Magic values
    public static double CRUISE_VELOCITY = 0.4;
    public static double ACCELERATION = 2;
    public static double JERK = 100;

    public static final double GRAVITY_FEEDFORWARD_OFFSET = -55; // offset in degrees. 0 should be horizontal

  }

  public static class RollerConstants {
    // Velocity voltage constants
    public static final boolean ENABLE_FOC = false;
    public static final double FEED_FORWARD = 0.0;
    public static final int SLOT = 0;
    public static final boolean OVERRIDE_BRAKE_DUR_NEUTRAL = false;

    // Motor ID
    public static final int MOTOR_ID = 21;

    // Motor limits
    public static final int MOTOR_STATOR_LIMIT = 60;

    public static final boolean IS_INVERTED = true;

    // Duty cycle limits
    public static final double PEAK_FORWARD_DUTY_CYCLE = 1;
    public static final double PEAK_REVERSE_DUTY_CYCLE = -1;

    // Conversions
    public static final double DEGREES_PER_REVOLUTION = 360.;

    // Intake volts
    public static double ROLLER_VOLTAGE = 1.0;
    public static double ROLLER_BACK_VOLTAGE = -1.0;

    // Intake motor velocities
    public static double ROLLER_VELOCITY = 100.0;
    public static double MANUAL_VELOCITY = 80.0;
    public static double MANUAL_BACK_VELOCITY = -80.0;
    public static double OUTTAKE_VELOCITY = -80.0;
    public static double RECYCLE_VELOCITY = 70.0;
    public static double INTAKE_VELOCITY = 80.0;
    public static double SHOOT_VELOCITY = 70.0;

    // Testing velocities
    public static double TESTING_VELOCITY = 0.0;

    // Max manual volts
    public static double ROLLER_MAX_MANUAL_VOLTS = 6.0;

    // PID gains
    public static double kP = 0.25;
    public static double kI = 0;
    public static double kD = 0;

    // Feed forward values
    public static double kS = 0;
    public static double kV = 0.12;
    public static double kA = 0;

    // Timing
    public static final double RUN_INTAKE_TIME = 2.0;
  }

  public static class IndexerConstants {
    // Velocity voltage constants
    public static final boolean ENABLE_FOC = false;
    public static final double FEED_FORWARD = 0.0;
    public static final int SLOT = 0;
    public static final boolean OVERRIDE_BRAKE_DUR_NEUTRAL = false;

    // Motor ID
    public static final int MOTOR_ID = 22;

    // Motor limits
    public static final int MOTOR_STATOR_LIMIT = 60;

    public static final boolean IS_INVERTED = true;

    // Duty cycle limits
    public static final double PEAK_FORWARD_DUTY_CYCLE = 1;
    public static final double PEAK_REVERSE_DUTY_CYCLE = -1;

    // Conversions
    public static final double DEGREES_PER_REVOLUTION = 360;

    // Indexer volts
    public static double INDEXER_VOLTAGE = 1.0;

    // Indexer motor velocity
    public static double INDEXER_VELOCITY = 70.0;

    public static double SHOOT_VELOCITY = 100.0;
    public static double MANUAL_VELOCITY = 80.0;
    public static double MANUAL_BACK_VELOCITY = -80.0;
    public static double OUTTAKE_VELOCITY = -100.0;
    public static double RECYCLE_VELOCITY = 100.0;
    public static double SHUTTLE_VELOCITY = 100.0;
    public static double INTAKE_VELOCITY = 100.0;

    // Testing velocities
    public static double TESTING_VELOCITY = 0.0;

    // Indexer PID/FF gains
    // Tuned in shop 2/12
    public static double kP = 0.3;
    public static double kI = 0.3;
    public static double kD = 0;
    public static double kS = 0;
    public static double kV = 0.15;
    public static double kA = 0;
  }

  public static class ColumnConstants {
    // Velocity voltage constants
    public static final boolean ENABLE_FOC = false;
    public static final double FEED_FORWARD = 0.0;
    public static final int SLOT = 0;
    public static final boolean OVERRIDE_BRAKE_DUR_NEUTRAL = false;

    // Motor ID
    public static final int INFLUENCER_ID = 23;
    public static final int FOLLOWER_ID = 24;

    // Motor limits
    public static final int INFLUENCER_STATOR_LIMIT = 40;
    public static final int FOLLOWER_STATOR_LIMIT = 40;

    public static final boolean IS_INFLUENCER_INVERTED = false;

    // Duty cycle limits
    public static final double PEAK_FORWARD_DUTY_CYCLE = 1;
    public static final double PEAK_REVERSE_DUTY_CYCLE = -1;

    // Conversions
    public static final double DEGREES_PER_REVOLUTION = 360;

    // Column volts
    public static double COLUMN_VOLTAGE = 1.0;

    // Column motor velocities
    public static double COLUMN_VELOCITY = 100.0;
    
    public static double SHOOT_VELOCITY = 100.0;
    public static double MANUAL_VELOCITY = 80.0;
    public static double MANUAL_BACK_VELOCITY = -80.0;
    public static double OUTTAKE_VELOCITY = -100.0;
    public static double RECYCLE_VELOCITY = 100.0;
    public static double SHUTTLE_VELOCITY = 100.0;
    public static double INTAKE_VELOCITY = -100.0;

    // Testing velocities
    public static double TESTING_VELOCITY = 0.0;

    public static final double AT_VELOCITY_DETECTION_PERCENTAGE = 0.50;

    // PID gains
    // Tuned in shop 2/12
    public static double INFLUENCER_kP = 0.3;
    public static double INFLUENCER_kI = 0.4;
    public static double INFLUENCER_kD = 0;

    // Feed forward values
    // Tuned in shop 2/12
    public static double INFLUENCER_kS = 0;
    public static double INFLUENCER_kV = 0.13;
    public static double INFLUENCER_kA = 0;

     // PID gains
    // Tuned in shop 2/12
    public static double FOLLOWER_kP = INFLUENCER_kP;
    public static double FOLLOWER_kI = INFLUENCER_kI;
    public static double FOLLOWER_kD = INFLUENCER_kD;

    // Feed forward values
    // Tuned in shop 2/12
    public static double FOLLOWER_kS = INFLUENCER_kS;
    public static double FOLLOWER_kV = INFLUENCER_kV;
    public static double FOLLOWER_kA = INFLUENCER_kA;
  }

  public static class ShooterConstants {
    // Velocity voltage constants - Influencer
    public static final boolean INFLUENCER_ENABLE_FOC = false;
    public static final double INFLUENCER_FEED_FORWARD = 0.0;
    public static final int INFLUENCER_SLOT = 0;
    public static final boolean INFLUENCER_OVERRIDE_BRAKE_DUR_NEUTRAL = false;

    // Velocity voltage constants - Follower
    public static final boolean FOLLOWER_ENABLE_FOC = false;
    public static final double FOLLOWER_FEED_FORWARD = 0.0;
    public static final int FOLLOWER_SLOT = 0;
    public static final boolean FOLLOWER_OVERRIDE_BRAKE_DUR_NEUTRAL = false;

    // Velocity voltage constants - Third
    public static final boolean THIRD_ENABLE_FOC = false;
    public static final double THIRD_FEED_FORWARD = 0.0;
    public static final int THIRD_SLOT = 0;
    public static final boolean THIRD_OVERRIDE_BRAKE_DUR_NEUTRAL = false;


    // Motor IDs
    public static final int INFLUENCER_MOTOR_ID = 25;
    public static final int FOLLOWER_MOTOR_ID = 26;
    public static final int THIRD_MOTOR_ID = 27;

    // Motor limits
    public static final int INFLUENCER_MOTOR_STATOR_LIMIT = 80;
    public static final int FOLLOWER_MOTOR_STATOR_LIMIT = 80;
    public static final int THIRD_MOTOR_STATOR_LIMIT = 80;

    public static final boolean IS_INFLUENCER_INVERTED = false;
    
    // Duty cycle limits
    public static final double PEAK_FORWARD_DUTY_CYCLE = 1;
    public static final double PEAK_REVERSE_DUTY_CYCLE = -1;

    // Conversions
    public static final double SHOOTER_DEGREES_PER_REVOLUTION = 360;

    // Motor velocities
    public static double SHOOTER_VELOCITY = 10.0;
    public static double SHOOTER_BACK_VELOCITY = -1.0;

    public static double SHOOTER_DISTANCE1_VELOCITY = 45.0;
    public static double SHOOTER_DISTANCE2_VELOCITY = 50.0;
    public static double SHOOTER_DISTANCE3_VELOCITY = 55.0;

    // Testing velocities
    public static double TESTING_VELOCITY = 40.0;

    public static double RECYCLE_VELOCITY = 10.0;
    public static double SHUTTLE_VELOCITY = 50.0;
    public static double OUTTAKE_VOLTAGE = -3.0;
    public static double INTIAL_ADDITIONAL_VELOCITY = 15.0; // TODO: tune

    public static double SHOOTER_VOLTAGE = 1.0;
    public static double SHOOTER_BACK_VOLTAGE = -1.0;

    // Influencer PID/FF gains
    public static double INFLUENCER_kP = 0.3;
    public static double INFLUENCER_kI = 0.3;
    public static double INFLUENCER_kD = 0;
    public static double INFLUENCER_kS = 0.2;
    public static double INFLUENCER_kV = 0.115;
    public static double INFLUENCER_kA = 0;

    // Follower PID/FF gains
    public static double FOLLOWER_kP = INFLUENCER_kP;
    public static double FOLLOWER_kI = INFLUENCER_kI;
    public static double FOLLOWER_kD = INFLUENCER_kD;
    public static double FOLLOWER_kS = INFLUENCER_kS;
    public static double FOLLOWER_kV = INFLUENCER_kV;
    public static double FOLLOWER_kA = INFLUENCER_kA;

    // Third PID/FF gains
    public static double THIRD_kP = INFLUENCER_kP;
    public static double THIRD_kI = INFLUENCER_kI;
    public static double THIRD_kD = INFLUENCER_kD;
    public static double THIRD_kS = INFLUENCER_kS;
    public static double THIRD_kV = INFLUENCER_kV;
    public static double THIRD_kA = INFLUENCER_kA;

    // Timing
    public static final double RUN_SHOOTER_TIME = 2.0;

    // Velocity tolerance for checking if shooter is at target (rotations per second)
    public static final double VELOCITY_TOLERANCE = 4;

    // Current spike threshold for checking if fuel is shot (amps)
    public static final double CURRENT_SPIKE_THRESHOLD = 20;

    // % tolerance for shooter velocity check (isFuelShot check against targetVelocity)
    public static final double FUEL_SHOT_DETECTION_PERCENTAGE = 0.95;

    public static final double FUEL_SHOT_TIMEOUT = 1.5; // in seconds

    public static final double WAIT_FOR_SHOOTER_TIMEOUT = 1.5;

    // SOTM constants
    public static final double SHOOTER_RPS_TO_MPS = Units.inchesToMeters(2*Math.PI*4);
    public static final double SHOOTER_SLIP_FACTOR = 0.8; // fudge factor for slippage between shooter and fuel; TODO: tune
    public static final double SHOOTER_ANGLE_DEGREES = 71.2393048349; // measured in CAD
  }

  public static class ClimberConstants {
    //Velocity voltage constants
    public static final boolean CLIMBER_ENABLE_FOC = false;
    public static final double CLIMBER_FEED_FORWARD = 0.0;
    public static final int CLIMBER_SLOT = 0;
    public static final boolean CLIMBER_OVERRIDE_BRAKE_DUR_NEUTRAL = false;

    public static final int CLIMBER_MOTOR_ID = 40;
    public static final int CLIMBER_MOTOR_STATOR_LIMIT = 40;

    // Duty cycle limits
    public static final double PEAK_FORWARD_DUTY_CYCLE = 1;
    public static final double PEAK_REVERSE_DUTY_CYCLE = -1;

    public static final double CLIMBER_METERS_PER_REVOLUTION = 360./10.;
  }
  
  public static class VisionConstants {
    // --- vision utils ---
    public static final double MAX_VISION_POSE_DISTANCE = 1;
    public static final double MAX_VISION_POSE_Z = 0.1;
    public static final double MAX_VISION_POSE_ROLL = 0.05; // in radians
    public static final double MAX_VISION_POSE_PITCH = 0.05; // in radians

    // --- filtering constants ---
    // max average distance and speed to use for local filters
    public static final double MAX_AVG_DIST_BETWEEN_LAST_EST_POSES = 0.3; // in meters 
    public static final double MAX_AVG_SPEED_BETWEEN_LAST_EST_POSES = MAX_AVG_DIST_BETWEEN_LAST_EST_POSES * 50.;
    
    public static final double MAX_VISION_READING_DISTANCE = 0.5; // in meters


    // min number of camera readings to use for global filters
    public static final int MIN_NUM_CAMERA_READINGS = 2; // NEEDS TO BE CONFIRMED W/ LOGIC 1/24

    // --- localization camera ---
    // default vision standard deviation
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(6, 6, 4);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 0.3);

    public static final double VISION_DISTANCE_DISCARD = 10; 
    public static final double MAX_POSE_AMBIGUITY = 0.2;
    public static final int NUM_LAST_EST_POSES = 3;
    public static final double STD_DEV_SCALAR = 30;

    // --- vision subsystem ---
    // (camera setup)
    // DEFAULT CONSTANTS (robot specific constants are below) 
    public static final String CAMERA1_NAME;
    public static final String CAMERA2_NAME;
    public static final String CAMERA3_NAME;
    public static final String CAMERA4_NAME;

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

    // Camera 3 position - robot-specific
    public static final double ROBOT_TO_CAM3_X;
    public static final double ROBOT_TO_CAM3_Y;
    public static final double ROBOT_TO_CAM3_Z;
    public static final double ROBOT_TO_CAM3_ROLL;
    public static final double ROBOT_TO_CAM3_PITCH;
    public static final double ROBOT_TO_CAM3_YAW;
    public static final Transform3d ROBOT_TO_CAM3_3D;

    // Camera 4 position - robot-specific
    public static final double ROBOT_TO_CAM4_X;
    public static final double ROBOT_TO_CAM4_Y;
    public static final double ROBOT_TO_CAM4_Z;
    public static final double ROBOT_TO_CAM4_ROLL;
    public static final double ROBOT_TO_CAM4_PITCH;
    public static final double ROBOT_TO_CAM4_YAW;
    public static final Transform3d ROBOT_TO_CAM4_3D;

    static {
      switch (RobotConfig.getRobot()) {
        case CLEO:
          // TODO: Measure and update these values for Cleo

          CAMERA1_NAME = "right_camera";
          CAMERA2_NAME = "left_camera";
          CAMERA3_NAME = "front_right_camera";
          CAMERA4_NAME = "front_left_camera"; 

          // Cleo camera positions
          ROBOT_TO_CAM1_X = Units.inchesToMeters(-21.0/2+2.0);
          ROBOT_TO_CAM1_Y = Units.inchesToMeters(-33.0/2+6.5);
          ROBOT_TO_CAM1_Z = Units.inchesToMeters(8.3);
          ROBOT_TO_CAM1_ROLL = 0;
          ROBOT_TO_CAM1_PITCH = Units.degreesToRadians(-20);
          ROBOT_TO_CAM1_YAW = Units.degreesToRadians(135)-0.12-0.02;

          ROBOT_TO_CAM2_X = Units.inchesToMeters(-21.0/2+2.25);
          ROBOT_TO_CAM2_Y = Units.inchesToMeters(33.0/2-6.5);
          ROBOT_TO_CAM2_Z = Units.inchesToMeters(8.5);
          ROBOT_TO_CAM2_ROLL = 0;
          ROBOT_TO_CAM2_PITCH = Units.degreesToRadians(-20);
          ROBOT_TO_CAM2_YAW = Units.degreesToRadians(-135)+0.14-0.01;
 
          ROBOT_TO_CAM3_X = Units.inchesToMeters(-21./2+3); 
          ROBOT_TO_CAM3_Y = Units.inchesToMeters(-33.0/2+(6.5-1.25)); 
          ROBOT_TO_CAM3_Z = Units.inchesToMeters(27); 
          ROBOT_TO_CAM3_ROLL = 0;
          ROBOT_TO_CAM3_PITCH = Units.degreesToRadians(-25); 
          ROBOT_TO_CAM3_YAW = Units.degreesToRadians(0); 

          ROBOT_TO_CAM4_X = Units.inchesToMeters(-21./2+3); 
          ROBOT_TO_CAM4_Y = Units.inchesToMeters(33.0/2-(6.5-1.25)); 
          ROBOT_TO_CAM4_Z = Units.inchesToMeters(26.75); 
          ROBOT_TO_CAM4_ROLL = 0;
          ROBOT_TO_CAM4_PITCH = Units.degreesToRadians(-25); 
          ROBOT_TO_CAM4_YAW = Units.degreesToRadians(0); 

          break;

        case CERIDWEN:
          // Ceridwen camera positions

          CAMERA1_NAME = "camera1";
          CAMERA2_NAME = "camera2";
          CAMERA3_NAME = "camera3";
          CAMERA4_NAME = "camera4"; 

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

          ROBOT_TO_CAM3_X = 0;
          ROBOT_TO_CAM3_Y = 0;
          ROBOT_TO_CAM3_Z = 0;
          ROBOT_TO_CAM3_ROLL = 0;
          ROBOT_TO_CAM3_PITCH = 0;
          ROBOT_TO_CAM3_YAW = 0;

          ROBOT_TO_CAM4_X = 0;
          ROBOT_TO_CAM4_Y = 0;
          ROBOT_TO_CAM4_Z = 0;
          ROBOT_TO_CAM4_ROLL = 0;
          ROBOT_TO_CAM4_PITCH = 0;
          ROBOT_TO_CAM4_YAW = 0;

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

      ROBOT_TO_CAM3_3D = new Transform3d(
        new Translation3d(ROBOT_TO_CAM3_X, ROBOT_TO_CAM3_Y, ROBOT_TO_CAM3_Z),
        new Rotation3d(ROBOT_TO_CAM3_ROLL, ROBOT_TO_CAM3_PITCH, ROBOT_TO_CAM3_YAW)
      );

      ROBOT_TO_CAM4_3D = new Transform3d(
        new Translation3d(ROBOT_TO_CAM4_X, ROBOT_TO_CAM4_Y, ROBOT_TO_CAM4_Z),
        new Rotation3d(ROBOT_TO_CAM4_ROLL, ROBOT_TO_CAM4_PITCH, ROBOT_TO_CAM4_YAW)
      );
    }
  }
  
  public static class LEDConstants { // placeholder constants
    public static final int KPORT = 0;
    public static final int KLENGTH = 60;

    public static final double BLINK_TIME = 1; // in seconds for after intake/outtake
  }

  public static class SensorConstants { // placeholder constants 
    // LaserCAN sensor CAN IDs
    public static final int INTAKE_SENSOR_CAN_ID = 60;
    public static final int FEEDER_SENSOR_CAN_ID = 61;

    // LaserCAN sensor beam break distances (in mm)
    public static final double INTAKE_SENSOR_MIN_DISTANCE = 100.0;
    public static final double FEEDER_SENSOR_MIN_DISTANCE = 100.0;
  }

  public static class AutoConstants {
    public static final double AUTO_SHOOT_TIMEOUT = 5; // in seconds; TODO: tune
  }

  public static class TeleopConstants {
    public static final double LATENCY_COMPENSATION = 0.1; // compensate for shooter spin up + code processing time, etc. TODO: tune
  }
}
