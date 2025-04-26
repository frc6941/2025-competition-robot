package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.Swerve;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.wpilog.WPILOGWriter.AdvantageScopeOpenBehavior;

import static frc.robot.RobotConstants.DriverCamera;

public class Robot extends LoggedRobot {
    private final Swerve swerve = Swerve.getInstance();
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        if (! RobotConstants.useReplay) {
        // logger initialization
            Logger.addDataReceiver(new NT4Publisher());
            Logger.addDataReceiver(new WPILOGWriter());
        }else{
            // Replaying a log, set up replay source
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"), AdvantageScopeOpenBehavior.ALWAYS));
        }


        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.start();
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        // early-stage initialization
        DriverStation.silenceJoystickConnectionWarning(true);
        PowerDistribution PDP = new PowerDistribution();
        PDP.clearStickyFaults();
        PDP.close();

        // Camera used by driver to help aiming
        // Remember to adjust fps & resolution in elastic (5fps, 300*200)
        // If network is bad or rio is in high cpu usage, disable it
        if (DriverCamera) {
            CameraServer.startAutomaticCapture("Driver Camera", "/dev/video0");
        }
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.getUpdateManager().runEnableSingle();
    }

    @Override
    public void disabledInit() {
        robotContainer.setMegaTag2(false);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        try {
            autonomousCommand = robotContainer.getAutonomousCommand();
        } catch (Exception e) {
            System.out.println("Autonomous command failed: " + e);
            e.printStackTrace();
            autonomousCommand = null;
        }
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
        swerve.auto();
        robotContainer.getUpdateManager().invokeStart();
        robotContainer.setMegaTag2(true);
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        robotContainer.getUpdateManager().invokeStop();
        swerve.normal();
        swerve.cancelFollow();
    }

    @Override
    public void teleopInit() {
        swerve.normal();
        robotContainer.getUpdateManager().invokeStart();
        robotContainer.setMegaTag2(true);
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {
        robotContainer.getUpdateManager().invokeStop();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
        robotContainer.getUpdateManager().runSimulateSingle();
    }
}
