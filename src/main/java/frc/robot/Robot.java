// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.PhoenixSignals;

/**
 * Runs tasks on Roborio in this file.
 */
public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;

    /**
     * Robnot Run type
     */
    public static enum RobotRunType {
        /** Real Robot. */
        kReal,
        /** Simulation runtime. */
        kSimulation,
        /** Replay runtime. */
        kReplay;
    }

    public Robot() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        RobotRunType robotRunType;

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
            Logger.addDataReceiver(new NT4Publisher());
            setUseTiming(true);
            robotRunType = RobotRunType.kReal;
        } else {
            Logger.addDataReceiver(new NT4Publisher());
            setUseTiming(true);
            robotRunType = RobotRunType.kSimulation;
        }
        Logger.start();

        robotContainer = new RobotContainer(robotRunType);
    }

    @Override
    public void robotPeriodic() {
        PhoenixSignals.refreshAll();
        CommandScheduler.getInstance().run();
        robotContainer.periodic();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}
