# Robot-Code-2026

Robot code for FRC Team 6418 (The Missfits), 2026 season. Java, WPILib command-based,
CTRE Phoenix 6 swerve, PhotonVision for AprilTags.

The goal of 2026's game is to shoot 6-inch foam balls into a hub. 

## Hardware

- CTRE swerve: TalonFX drive and steer, CANcoders (fused), Pigeon 2 gyro. 
- Four Arducam OV9281 cameras with two Orange Pi 5s running PhotonVision (multi-tag PnP). 
- Two robots are supported: Cleo (competition) and Ceridwen (practice). 
- Grapple LaserCAN for game-piece detection in the intake path.
- All mechanism motors are TalonFX.

## Dependencies

- WPILib / GradleRIO 2026.2.1
- CTRE Phoenix 6 26.1.0
- PhotonLib v2026.1.1-rc-2
- PathPlannerLib 2026.1.2
- libgrapplefrc 2026.0.0 (LaserCAN)
- maple-sim 0.4.0-beta (simulation only)

## Localization

PhotonVision runs on the coprocessors and gives us a robot pose per camera per frame.
Those poses are noisy and sometimes wrong, so we added a filtering layer before the estimates
reach the drivetrain's pose estimator.

Per camera:
- If PhotonVision reports a high-ambiguity single-tag result, we reconstruct both
  candidate poses and keep the one that agrees with the gyro heading
  (`resolveHighAmbiguityPose`). 
- Filter out poses with unreasonable z / roll / pitch (`LocalPoseZRollPitchFilter`).
- Check that the last few readings from this camera are consistent with each other
  (implied speed between readings), OR that the reading is within some distance of the
  current fused pose. We'd like to recover after a collision -- when the camera is right and the
  odometry is wrong. 
  (`LocalCameraPoseConsistencyDistanceToFusedPoseFilter`).
- Compute a standard deviation for the measurement based on number of tags seen and
  average tag distance, so far/single-tag readings get less weight
  (`calculateEstimationStdDevs`).

Across cameras:
- Average all remaining readings and filter out any that are too far from the average
  (`GlobalCrossCameraConsensusFilter`).

The readings that are left are fed to the swerve pose estimator alongside std devs and timestamp.

## Shoot-on-the-move

Our shooter has a fixed angle, so in order to automatically shoot, we have to determine 
the shooter wheel speed and robot drivetrain heading. `HubCalculations.calculateShootOnTheFlyAngle`:

1. Project the robot's position forward by a fixed latency constant.
2. Get distance to the hub, look up the wheel speed for that distance
   (`ShooterLookupTable`, tuned by hand at practice).
3. Convert to a horizontal exit velocity.
4. Subtract the robot's field-relative velocity from the desired ball velocity and use the
   result's angle as the drivetrain heading.

A similar function is used for shuttling to the near corners of the field.

## Teleop Control Scheme

Teleop is organized as modes rather than a bunch of button bindings:
neutral, intake, score, and shuttle (`RobotCommandFactory`). 
Each mode is one command that operates the whole robot, so the driver switches modes and 
the code handles sequencing (spin up, wait for velocity, feed, etc). 
Note that score mode is "dynamic": it keeps re-aiming while the robot moves. 

## Tuning and Telemetry

Most values determining robot state are pushed to SmartDashboard. Using Elastic, we can view real-time values 
as the robot is running. With `DataLogManager`, we record these values, which can be replayed after the fact. 

Through our base mechanism classes, (`MechanismsIOHardwareBase`), the state of every mechanism is 
comprehensively and consistently logged. 
We also implemented a system to tune robot constants through SmartDashboard and Elastic without needing to redeploy code. 

## Autos

We use PathPlanner. Paths and autos are in `src/main/deploy/pathplanner/`; named commands are
registered in `RobotContainer`. We ran 2-pass auto to grab fuel from the center on both sides, and a special
"flex" 1-pass auto that plays well as a third scoring routine running in offset with two other center-going robots.
Our pathplanner folder also contains some other quirky autos. 

## Project Structure

```
src/main/java/frc/robot/
├── Robot.java                  # Main robot class
├── RobotContainer.java         # Robot initialization and command binding
├── Constants.java              # All robot configuration constants
├── FieldConstants.java         # Field-specific constants
├── subsystems/
│   ├── drivetrain/            # Swerve drivetrain implementation
│   ├── intake/                # Roller, Pivot, Indexer, Column subsystems
│   ├── scorer/                # Shooter subsystem
│   ├── vision/                # Vision and pose estimation
│   ├── MechanismsIOHardwareBase.java  # Base class for motor IO
│   ├── MechanismsSubsystemBase.java   # Base class for mechanisms
│   └── RobotCommandFactory.java       # Factory for creating robot-wide commands
└── utils/                     # Utility classes

src/main/deploy/
├── pathplanner/                # PathPlanner paths and autos
```

## Running the Project

Standard WPILib project: open in VS Code with the WPILib extension, deploy to the roboRIO.
`./gradlew test` runs the unit tests.