# FRC Robot Code 2026

This is the robot code for **FRC Team 6418 - The Missfits** 2026 season competition robot, written in Java using WPILib and CTRE Phoenix 6 libraries.

## Project Overview

This is a command-based robot project for the 2026 FRC season with the following key subsystems and features:

### Subsystems

**Drivetrain** :car::dash:
- Swerve drivetrain using CTRE Phoenix 6
- Full 360-degree movement with translation and rotation
- Operator perspective support for Red/Blue alliance
- Telemetry logging for debugging

**Intake System** (2-stage intake) :open_mouth::poultry_leg:
- **Pivot**: Arm that deploys/stores the intake mechanism
- **Roller**: Primary intake mechanism with velocity control
- **Indexer**: Transports game pieces from roller to column, also serves as an agitator
- **Column**: Triple rollers that feed game pieces to shooter

**Shooter** :gun: pewpew
- Three independent motors (Influencer, Follower, Third) with real-time velocity control
- **Shoot on the Move**: Dynamic velocity adjustment while translating
- Distance/angle lookup table for consistent accuracy across the field
- Independent PID tuning per motor with velocity ramping and timeout protection

**Vision** :eyes:
- **Pose Estimation**: PhotonVision AprilTag detection with Kalman filter fusion
- **Multi-Camera System**: Local and global camera pipelines with advanced filtering
- Pose consistency validation, cross-camera consensus, and distance-based filtering
- Real-time pose updates correct odometry drift during matches

### Key :key: Features 

- **Dynamic Tuning**: SmartDashboard integration for real-time tuning of velocities, angles, and other parameters
- **Motor Safety**: Dual current limiting (stator + supply) on all motors to protect mechanisms and battery
- **Telemetry**: Comprehensive logging of mechanism states (position, velocity, current, voltage)
- **Command-Based Architecture**: Uses WPILib command-based programming for autonomous and teleop control
- **Hardware Abstraction**: IO hardware classes separate motor control logic from subsystem interface

## Project Structure :building_construction:

```
src/main/java/frc/robot/
├── Robot.java                  # Main robot class
├── RobotContainer.java         # Robot initialization and command binding
├── RobotCommandFactory.java    # Factory for creating robot-wide commands
├── Constants.java              # All robot configuration constants
├── FieldConstants.java         # Field-specific constants
├── subsystems/
│   ├── drivetrain/            # Swerve drivetrain implementation
│   ├── intake/                # Roller, Pivot, Indexer, Column subsystems
│   ├── scorer/                # Shooter subsystem
│   ├── vision/                # Vision and pose estimation
│   ├── MechanismsIOHardwareBase.java  # Base class for motor IO
│   ├── MechanismsSubsystemBase.java   # Base class for mechanisms
│   └── RobotCommandFactory.java       # Robot command factory
├── utils/                     # Utility classes
└── commands/                  # Command implementations
```

## Constants Configuration

All mechanism constants are defined in `Constants.java` organized by subsystem:
- Motor IDs, current limits, velocity targets
- PID gains (kP, kI, kD)
- Feed-forward coefficients (kS, kV, kA)
- Duty cycle limits
- Motion magic parameters (for mechanisms with profiled motion)

Each subsystem has its own constants class (e.g., `RollerConstants`, `ShooterConstants`) for organization.

## Motor Hardware Abstraction :hammer_and_wrench:

### MechanismsIOHardwareBase
Base class for all motor IO implementations that handles:
- Motor configuration and initialization
- Status signal polling (position, velocity, voltage, current)
- Position and velocity getters/setters
- Inversion and neutral mode control

### Subsystem-Specific IO Classes
Each mechanism has its own IO hardware class that extends `MechanismsIOHardwareBase`:
- `RollerIOHardware`, `PivotIOHardware`, `IndexerIOHardware`
- `ColumnIOHardware`, `ShooterIOHardware`

These classes implement mechanism-specific behavior like gravity feedforward for Pivot.

## Command Factory :factory: Pattern 

The `RobotCommandFactory` centralizes command creation for complex robot behaviors:
- Intake, scoring, and neutral mode sequences
- Hub and shuttle target alignment
- Dynamic velocity and angle commands via SmartDashboard
- Coordinated subsystem control

This keeps command logic organized and reusable across teleop and autonomous modes.

## Autonomous Routines

Autonomous commands are defined in `Autos.java` and integrated with **PathPlanner** for trajectory generation:

- **Path Following**: Uses PathPlanner for smooth, optimized trajectories
- **Named Commands**: Registered commands allow easy event-based actions during auto (intake, shoot, etc.)
- **Dynamic Tuning**: Auto parameters (velocities, angles, timeouts) configurable via Constants.java
- **Vision-Aided**: Pose estimation corrects path tracking in real-time

Autonomous routines combine:
- Swerve drivetrain path following
- Intake/indexer synchronization
- Shooter velocity ramping and sequencing
- Vision-based target alignment

See **PathPlanner Documentation** (https://pathplanner.dev/) for trajectory creation and tuning.

## Building :house_buildings: and Running :woman-running:

### Prerequisites
- JDK 17 or later
- Gradle (included via wrapper)
- WPILib dependencies (downloaded automatically)

### Build
```bash
./gradlew compileJava
```

### Test
```bash
./gradlew test
```

### Deploy to RoboRIO
```bash
./gradlew deploy
```

## Technology Stack

- **Language**: Java 17
- **Build System**: Gradle with GradleRIO
- **Robot Framework**: WPILib 2026
- **Motor Controllers**: CTRE Phoenix 6
- **Vision**: PhotonVision
- **Simulation**: MapleSimSwervePhysics

## Documentation References

- [WPILib Documentation](https://docs.wpilib.org/)
- [Phoenix 6 Documentation](https://docs.ctre-phoenix.com/en/stable/index)
- [PhotonVision Documentation](https://docs.photonvision.org/)
- [PathPlanner Documentation](https://pathplanner.dev/)

## Team Information :spiral_note_pad::do_not_litter:

**Team Name**: The Missfits
**Team Number**: FRC 6418
**Email**: missfitsrobotics@gmail.com

For questions about this codebase, the robot design, or collaboration opportunities, please reach out via email!
