/* Blatantly ~~stole from~~ inspired by 254
https://github.com/Team254/FRC-2025-Public

MIT License

Copyright (c) 2025 Team 254

Copyright (c) 2026 The Missfits

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.simulation.MapleSimSwervePhysics;

public class CommandSwerveDrivetrainSim extends CommandSwerveDrivetrain {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier simNotifier = null;
    public MapleSimSwervePhysics mapleSimSwervePhysics = null;

    Pose2d lastConsumedPose = null;
    Consumer<SwerveDriveState> simTelemetryConsumer =
            swerveDriveState -> {

    
                swerveDriveState.Pose =
                mapleSimSwervePhysics.mapleSimDrive.getSimulatedDriveTrainPose();
        
            
                //telemetryConsumer_.accept(swerveDriveState);
            };
    public CommandSwerveDrivetrainSim(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        startSimThread();
    }

    public Consumer<SwerveDriveState> getSimTelemetryConsumer() {
        return simTelemetryConsumer;
    }

    @SuppressWarnings("unchecked")
    public void startSimThread() {
            mapleSimSwervePhysics =
                    new MapleSimSwervePhysics(
                            Seconds.of(kSimLoopPeriod),
                            Pounds.of(100),
                            Inches.of(31+6.5),
                            Inches.of(23),
                            DCMotor.getKrakenX60(1),
                            DCMotor.getKrakenX60(1),
                            1.2,
                            getModuleLocations(),
                            getPigeon2(),
                            getModules(),
                            TunerConstants.FrontLeft,
                            TunerConstants.FrontRight,
                            TunerConstants.BackLeft,
                            TunerConstants.BackRight);
            simNotifier = new Notifier(mapleSimSwervePhysics::update);

        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwervePhysics != null)
            mapleSimSwervePhysics.mapleSimDrive.setSimulationWorldPose(pose);
        Timer.delay(0.05); // Wait for simulation to update
        super.resetPose(pose);
    }  

    
}