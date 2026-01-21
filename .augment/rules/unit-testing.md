---
type: "agent_requested"
description: "Guidelines for writing unit tests in FRC robot projects using JUnit 5 and WPILib simulation"
---

# Unit Testing Guidelines for FRC Robot Code

Apply these guidelines when writing, modifying, or discussing unit tests for robot code.

## Test Framework

This project uses **JUnit 5** for unit testing. Tests run in simulation mode on the desktop.

## Test File Location

Place all test files in: `src/test/java/`

The test directory structure should mirror the main source structure. For example:
- Main code: `src/main/java/frc/robot/subsystems/Intake.java`
- Test code: `src/test/java/frc/robot/subsystems/IntakeTest.java`

## Writing Testable Code

### Implement AutoCloseable

Subsystems should implement `AutoCloseable` to allow proper cleanup between tests:

```java
public class MySubsystem implements AutoCloseable {
    private final PWMSparkMax m_motor;
    
    @Override
    public void close() {
        m_motor.close();
    }
}
```

## Test Class Structure

Use the following structure for test classes:

```java
import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class MySubsystemTest {
    static final double DELTA = 1e-2; // acceptable deviation for floating-point comparison
    
    MySubsystem m_subsystem;
    // Simulation objects (PWMSim, DoubleSolenoidSim, etc.)
    
    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL
        m_subsystem = new MySubsystem();
        // Initialize simulation objects
    }
    
    @AfterEach
    void shutdown() throws Exception {
        m_subsystem.close(); // cleanup
    }
    
    @Test
    void testSomething() {
        // Test implementation with assertions
        assertEquals(expected, actual, DELTA);
    }
}
```

## Simulation Classes

Use WPILib simulation classes to verify hardware states:
- `edu.wpi.first.wpilibj.simulation.PWMSim` - for PWM motor controllers
- `edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim` - for double solenoids
- `edu.wpi.first.wpilibj.simulation.EncoderSim` - for encoders
- `edu.wpi.first.wpilibj.simulation.AnalogInputSim` - for analog inputs

## Assertions

Use JUnit 5 assertions from `org.junit.jupiter.api.Assertions`:
- `assertEquals(expected, actual)` - exact equality
- `assertEquals(expected, actual, delta)` - floating-point comparison with tolerance
- `assertTrue(condition)` / `assertFalse(condition)`
- `assertNull(object)` / `assertNotNull(object)`

## Running Tests

Run tests using the "Test Robot Code" command from the VS Code Command Palette, or:

```bash
./gradlew test
```

Test results are available in `build/reports/tests/test/index.html`.

## build.gradle Requirements

Ensure `build.gradle` contains the test configuration:

```groovy
test {
    useJUnitPlatform()
    systemProperty 'junit.jupiter.extensions.autodetection.enabled', 'true'
}
```

## Best Practices

- Each test method should test one specific behavior
- Use descriptive test method names that explain what is being tested
- Initialize HAL before creating any WPILib objects: `assert HAL.initialize(500, 0);`
- Always close resources in `@AfterEach` to prevent hardware allocation conflicts
- Use `DELTA` constant for floating-point comparisons to account for precision issues

### Further documentation:

For further updated information, reference the WPILib unit testing documentation:

https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html

