# Robot Configuration System

This codebase supports multiple robots with different hardware configurations. This document explains how to switch between robots and configure robot-specific constants.

## Quick Start: Switching Robots

**To switch between robots, change ONE line in `RobotConfig.java`:**

```java
private static RobotType ROBOT = RobotType.ROBOT_2026_1;  // Change this line
```

Available options:
- `RobotType.ROBOT_2026_1` - First competition robot
- `RobotType.ROBOT_2026_2` - Second competition robot (practice bot)

## Architecture

The robot differentiation system is based on the Mechanical Advantage pattern and consists of:

### 1. `RobotConfig.java` - Robot Selection
- **Single source of truth** for which robot is active
- Contains safety checks to prevent deploying SIMBOT to real hardware
- Provides `getRobot()` method used throughout the codebase

### 2. `Constants.java` - Robot-Specific Constants
Robot-specific constants are defined using static initializer blocks:

```java
public static class DrivetrainConstants {
    public static final double STEER_KP;
    public static final double STEER_KI;
    // ... more constants
    
    static {
        switch (RobotConfig.getRobot()) {
            case ROBOT_2026_1:
                STEER_KP = 100;
                STEER_KI = 0;
                // ... Robot 1 values
                break;
                
            case ROBOT_2026_2:
                STEER_KP = 95;  // Different value for Robot 2
                STEER_KI = 0;
                // ... Robot 2 values
                break;
                
            case SIMBOT:
            default:
                // Simulation values (usually same as Robot 1)
                break;
        }
    }
}
```

### 3. `TunerConstants.java` - Hardware Configuration
Hardware-specific values (motor IDs, encoder offsets) are also robot-specific:

```java
static {
    switch (RobotConfig.getRobot()) {
        case ROBOT_2026_1:
            kFrontLeftDriveMotorId = 8;
            kFrontLeftEncoderOffset = Rotations.of(-0.036865234375);
            // ... Robot 1 hardware config
            break;
            
        case ROBOT_2026_2:
            kFrontLeftDriveMotorId = 8;
            kFrontLeftEncoderOffset = Rotations.of(0.0);  // Different offset
            // ... Robot 2 hardware config
            break;
    }
}
```

## Workflow

### Setting Up a New Robot

1. **Change robot selection** in `RobotConfig.java`:
   ```java
   private static RobotType ROBOT = RobotType.ROBOT_2026_2;
   ```

2. **Update hardware configuration** in `TunerConstants.java`:
   - Run CTRE Tuner X to get encoder offsets
   - Update the `ROBOT_2026_2` case with the new values
   - Verify motor IDs match your hardware

3. **Tune constants** in `Constants.java`:
   - Update PID gains in the `ROBOT_2026_2` case
   - Tune drivetrain constants
   - Update any mechanism-specific constants

4. **Test and iterate**

### Deploying to Competition

1. **Verify robot selection** matches the physical robot
2. **Build and deploy**:
   ```bash
   ./gradlew deploy
   ```
3. The system will automatically prevent deploying SIMBOT to real hardware

### Safety Checks

The system includes validation mechanisms:

#### CheckDeploy (runs before deployment)
```bash
./gradlew CheckDeploy
```
- Validates robot configuration before deployment
- Run automatically by deployment scripts

#### CheckPullRequest (for CI/CD)
```bash
./gradlew CheckPullRequest
```
- Validates robot configuration for pull requests
- Can be used in CI/CD pipelines to enforce standards

## Best Practices

### ✅ DO:
- Change `RobotConfig.ROBOT` to switch robots
- Keep robot-specific constants in the switch statements
- Use descriptive comments for robot-specific tuning
- Test on simulation before deploying to hardware
- Run `CheckDeploy` before competition

### ❌ DON'T:
- Hardcode robot-specific values outside the switch statements
- Deploy without verifying robot selection matches the physical robot
- Forget to update encoder offsets for new robots

## Adding More Robots

To add a third robot:

1. Add to `RobotConfig.java`:
   ```java
   public enum RobotType {
       ROBOT_2026_1,
       ROBOT_2026_2,
       ROBOT_2026_3  // New robot
   }
   ```

2. Add cases in `Constants.java` and `TunerConstants.java`:
   ```java
   case ROBOT_2026_3:
       // Robot 3 constants
       break;
   ```

## Troubleshooting

**Problem**: Robot behaves incorrectly
- **Solution**: Verify `RobotConfig.ROBOT` matches the physical robot

**Problem**: Encoder offsets are wrong
- **Solution**: Re-run Tuner X and update the appropriate robot case

**Problem**: Constants seem to be ignored
- **Solution**: Ensure constants are in the correct switch case, not hardcoded

**Problem**: Build fails with "Unknown robot type" error
- **Solution**: Make sure all switch statements in Constants.java and TunerConstants.java have cases for both ROBOT_2026_1 and ROBOT_2026_2

