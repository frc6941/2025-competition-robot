package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import static frc.robot.RobotConstants.DriverCamera;
import static frc.robot.RobotConstants.VisionCamera;

public class Robot extends LoggedRobot {
    private final Swerve swerve = Swerve.getInstance();
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        // logger initialization
        if (Robot.isSimulation())
            Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter());
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
            //JSONObject config = new JSONObject();
            //config.put("absolute_exposure", 100);
            //CameraServer.getServer("Driver Camera").setConfigJson(config.toJSONString());
        }
        if (VisionCamera) {
            //TODO add vision camera
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
        DestinationSupplier.getInstance().setUseVision(true);
    }

    @Override
    public void teleopInit() {
        swerve.normal();
        robotContainer.getUpdateManager().invokeStart();
        DestinationSupplier.getInstance().setUseVision(true);
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
//        Pose3d cam1Pose = new Pose3d(
//                0.20556,
//                0.33419,
//                0.31560,
//                new Rotation3d(0.0, 0.0, Units.degreesToRadians(-21)));
//        Transform3d cam1TransR = new Transform3d(
//                0, 0, 0,
//                new Rotation3d(0.0, 0.0, Units.degreesToRadians(21)));
//        Transform3d cam1TransT = new Transform3d(
//                -0.20556,
//                -0.33419,
//                -0.31560,
//                new Rotation3d(0.0, 0.0, Units.degreesToRadians(0)));
//        Transform3d cam1Trans111 = new Transform3d(0.20556,
//                0.33419,
//                0.31560,
//                new Rotation3d(0.0, 0.0, Units.degreesToRadians(-21))).inverse();
//        Pose3d cam1PoseE = cam1Pose.transformBy(cam1TransR);
//        Pose3d cam1PoseF = cam1PoseE.transformBy(cam1TransT);
//
//        Transform3d cam2TransR = new Transform3d(0, 0, 0,
//                new Rotation3d(-0.0064, -0.7093, 0.0239));
//        Transform3d cam2TransT = new Transform3d(0.211601527157513, -0.582579438752884, 0.00456979252483178, new Rotation3d(0.0, 0.0, Units.degreesToRadians(0)));
//        Pose3d cam2PoseE = cam1PoseF.transformBy(cam2TransT);
//        Pose3d cam2PoseF = cam2PoseE.transformBy(cam2TransR);
//        Pose3d cam2PoseFF = cam2PoseF.transformBy(cam1TransT.inverse());
//        Pose3d cam2PoseFFF = cam2PoseFF.transformBy(cam1TransR.inverse());
//        Pose3d cam2PoseTest = cam2PoseF.transformBy(cam1Trans111);
//        System.out.println(cam2PoseFFF.getTranslation());
//        System.out.println(cam2PoseFFF.getRotation().toString());
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
        robotContainer.getUpdateManager().runSimulateSingle();
    }
}