# Robot Configuration Quick Reference

## 🚀 How to Switch Robots

**Edit ONE line in `src/main/java/frc/robot/RobotConfig.java`:**

```java
private static RobotType ROBOT = RobotType.ROBOT_2026_1;  // ← Change this
```

Options:
- `ROBOT_2026_1` - Competition robot
- `ROBOT_2026_2` - Practice robot

## 📝 Where to Add Robot-Specific Values

### Constants.java - Tuning Values
Add values in the switch statement:

```java
static {
    switch (RobotConfig.getRobot()) {
        case ROBOT_2026_1:
            STEER_KP = 100;  // ← Robot 1 value
            break;
        case ROBOT_2026_2:
            STEER_KP = 95;   // ← Robot 2 value
            break;
    }
}
```

### TunerConstants.java - Hardware Config
Add hardware IDs and encoder offsets:

```java
static {
    switch (RobotConfig.getRobot()) {
        case ROBOT_2026_1:
            kFrontLeftEncoderOffset = Rotations.of(-0.036865234375);  // ← Robot 1
            break;
        case ROBOT_2026_2:
            kFrontLeftEncoderOffset = Rotations.of(0.123456789);      // ← Robot 2
            break;
    }
}
```

## ✅ Pre-Deployment Checklist

1. ☑️ Verify `RobotConfig.ROBOT` matches physical robot
2. ☑️ Run `./gradlew build` to check for errors
3. ☑️ Run `./gradlew CheckDeploy` to verify deployment safety
4. ☑️ Deploy: `./gradlew deploy`

## 🔧 Setting Up Robot 2

1. **Select Robot 2** in `RobotConfig.java`:
   ```java
   private static RobotType ROBOT = RobotType.ROBOT_2026_2;
   ```

2. **Run Tuner X** on Robot 2 to get encoder offsets

3. **Update `TunerConstants.java`** with Robot 2 encoder offsets:
   ```java
   case ROBOT_2026_2:
       kFrontLeftEncoderOffset = Rotations.of(YOUR_VALUE_HERE);
       kFrontRightEncoderOffset = Rotations.of(YOUR_VALUE_HERE);
       kBackLeftEncoderOffset = Rotations.of(YOUR_VALUE_HERE);
       kBackRightEncoderOffset = Rotations.of(YOUR_VALUE_HERE);
       break;
   ```

4. **Tune PID values** in `Constants.java` for Robot 2

5. **Test and iterate**

## 🛡️ Safety Features

| Feature | Purpose | When It Runs |
|---------|---------|--------------|
| `CheckDeploy` | Validates robot configuration | Before deployment |
| `CheckPullRequest` | Validates configuration for CI/CD | In CI/CD pipelines |

## 🐛 Common Issues

| Problem | Solution |
|---------|----------|
| Robot drives wrong | Check `RobotConfig.ROBOT` matches physical robot |
| Modules don't align | Update encoder offsets for the correct robot |
| Constants ignored | Ensure values are in switch statement, not hardcoded |
| Build fails | Make sure all switch statements have cases for both robots |

## 📚 Files Modified

- ✅ `RobotConfig.java` - **NEW** - Robot selection and safety checks
- ✅ `Constants.java` - Modified to support robot-specific constants
- ✅ `TunerConstants.java` - Modified to support robot-specific hardware config
- ✅ `ROBOT_CONFIGURATION.md` - Full documentation
- ✅ `QUICK_REFERENCE.md` - This file

## 🎯 Example: Switching from Robot 1 to Robot 2

**Before deployment to Robot 2:**

```java
// In RobotConfig.java
private static RobotType ROBOT = RobotType.ROBOT_2026_2;  // Changed from ROBOT_2026_1
```

**That's it!** All constants automatically switch to Robot 2 values.

## 💡 Pro Tips

- Use descriptive comments for robot-specific tuning decisions
- Test in simulation before deploying to hardware
- Document why values differ between robots
- Run `CheckDeploy` before every competition
- Keep both robot configurations up to date even if you primarily use one

---

**Need more details?** See `ROBOT_CONFIGURATION.md` for complete documentation.

