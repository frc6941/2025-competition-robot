package frc.robot;

import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.Swerve;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import java.io.IOException;

// FIXME: Too many things here!
public class Robot extends LoggedRobot {
    Swerve swerve = Swerve.getInstance();
    private Command m_autonomousCommand;
    private RobotContainer robotContainer;

    // Initializes the robot at the start of operation
    @Override
    public void robotInit() {
        Logger.addDataReceiver(new NT4Publisher());
        // Logger.addDataReceiver(new WPILOGWriter());
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.start();
        robotContainer = new RobotContainer();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Runs periodically while the robot is powered on
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.getUpdateManager().runEnableSingle();
    }

    // Initializes the robot in disabled mode
    @Override
    public void disabledInit() {
    }

    // Runs periodically while the robot is in disabled mode
    @Override
    public void disabledPeriodic() {
    }

    // Called when the robot exits disabled mode
    @Override
    public void disabledExit() {
    }

    // Initializes the robot in autonomous mode
    @Override
    public void autonomousInit() {
        m_autonomousCommand = robotContainer.getAutonomousCommand();
        robotContainer.getUpdateManager().runEnableSingle();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        robotContainer.getUpdateManager().invokeStart();
        swerve.auto();
    }

    // Runs periodically during autonomous mode
    @Override
    public void autonomousPeriodic() {
    }

    // Called when the robot exits autonomous mode
    @Override
    public void autonomousExit() {
        robotContainer.getUpdateManager().invokeStop();
        swerve.normal();
        swerve.cancelFollow();
    }

    // Initializes the robot in teleoperated mode
    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        swerve.normal();
        robotContainer.getUpdateManager().invokeStart();

    }

    // Runs periodically during teleoperated mode
    @Override
    public void teleopPeriodic() {

    }

    // Called when the robot exits teleoperated mode
    @Override
    public void teleopExit() {
        robotContainer.getUpdateManager().invokeStop();

    }

    // Initializes the robot in test mode
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    // Runs periodically during test mode
    @Override
    public void testPeriodic() {

    }

    // Called when the robot exits test mode
    @Override
    public void testExit() {
    }

    // Runs periodically during simulation mode
    @Override
    public void simulationPeriodic() {
        robotContainer.getUpdateManager().runSimulateSingle();
    }
}