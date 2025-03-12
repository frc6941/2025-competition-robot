package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants;
import frc.robot.display.Display;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

import static frc.robot.RobotConstants.ReefAimConstants;

public class ReefAimCommand extends Command {
    private final Swerve swerve = Swerve.getInstance();
    private final ProfiledPIDController xPID = new ProfiledPIDController(
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get(),
            new TrapezoidProfile.Constraints(
                    ReefAimConstants.MAX_AIMING_SPEED.magnitude(),
                    ReefAimConstants.MAX_AIMING_ACCELERATION.magnitude()));
    private final ProfiledPIDController yPID = new ProfiledPIDController(
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get(),
            new TrapezoidProfile.Constraints(
                    ReefAimConstants.MAX_AIMING_SPEED.magnitude(),
                    ReefAimConstants.MAX_AIMING_ACCELERATION.magnitude()));
    private final BooleanSupplier stop;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CommandXboxController driverController;
    private final IndicatorSubsystem indicatorSubsystem;
    private boolean rightReef; // true if shooting right reef
    private boolean xFinished = false;
    private boolean yFinished = false;
    private boolean omegaFinished = false;
    private Pose2d robotPose, tagPose, destinationPose, finalDestinationPose;
    private Translation2d translationalVelocity, controllerVelocity;


    public ReefAimCommand(BooleanSupplier stop, ElevatorSubsystem elevatorSubsystem,
                          CommandXboxController driverController, IndicatorSubsystem indicatorSubsystem) {
        addRequirements(swerve);
        this.stop = stop;
        this.elevatorSubsystem = elevatorSubsystem;
        this.driverController = driverController;
        this.indicatorSubsystem = indicatorSubsystem;
    }

    @Override
    public void initialize() {
        // Calculate destination
        robotPose = swerve.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp());
        DestinationSupplier.getInstance();
        tagPose = DestinationSupplier.getNearestTag(robotPose);
        // PID init
        xPID.reset(robotPose.getX(), swerve.getLocalizer().getMeasuredVelocity().getX());
        yPID.reset(robotPose.getY(), swerve.getLocalizer().getMeasuredVelocity().getY());
        rightReef = DestinationSupplier.getInstance().getCurrentBranch();
        DestinationSupplier.getInstance();
        finalDestinationPose = DestinationSupplier.getFinalDriveTarget(tagPose, rightReef);
        indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);

    }

    @Override
    public void execute() {
        xPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        ReefAimConstants.MAX_AIMING_SPEED.magnitude() * 0.6 / elevatorSubsystem.getIo().getElevatorHeight(),
                        ReefAimConstants.MAX_AIMING_ACCELERATION.magnitude() * 0.6 / elevatorSubsystem.getIo().getElevatorHeight()));
        yPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        ReefAimConstants.MAX_AIMING_SPEED.magnitude() * 0.6 / elevatorSubsystem.getIo().getElevatorHeight(),
                        ReefAimConstants.MAX_AIMING_ACCELERATION.magnitude() * 0.6 / elevatorSubsystem.getIo().getElevatorHeight()));
        if (RobotConstants.TUNING) {
            xPID.setPID(RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
            yPID.setPID(RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
        }

        robotPose = swerve.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp());
        destinationPose = DestinationSupplier.getDriveTarget(robotPose, tagPose, rightReef);

        xPID.setGoal(destinationPose.getTranslation().getX());
        yPID.setGoal(destinationPose.getTranslation().getY());
        swerve.setLockHeading(true);
        swerve.setHeadingTarget(destinationPose.getRotation().getDegrees() - 180.0);
        translationalVelocity = AllianceFlipUtil.shouldFlip() ?
                new Translation2d(-xPID.calculate(robotPose.getX()), -yPID.calculate(robotPose.getY())) :
                new Translation2d(xPID.calculate(robotPose.getX()), yPID.calculate(robotPose.getY()));
        controllerVelocity = new Translation2d(
                Math.abs(driverController.getLeftY()) < RobotConstants.SwerveConstants.deadband ?
                        0 : -driverController.getLeftY() * RobotConstants.SwerveConstants.maxSpeed.magnitude(),
                Math.abs(driverController.getLeftX()) < RobotConstants.SwerveConstants.deadband ?
                        0 : -driverController.getLeftX() * RobotConstants.SwerveConstants.maxSpeed.magnitude());
        swerve.drive(translationalVelocity, 0.0, true, false);
        Display.getInstance().setAimingTarget(destinationPose);
        Logger.recordOutput("ReefAimCommand/tagPose", tagPose);
        Logger.recordOutput("ReefAimCommand/destinationPose", destinationPose);
        Logger.recordOutput("ReefAimCommand/finalDestinationPose", finalDestinationPose);
        Logger.recordOutput("ReefAimCommand/translationalVelocity", translationalVelocity);
        Logger.recordOutput("ReefAimCommand/controllerVelocity", controllerVelocity);
    }

    @Override
    public boolean isFinished() {
        xFinished = Math.abs(robotPose.getX() - finalDestinationPose.getX()) < ReefAimConstants.X_TOLERANCE_METERS.get();
        yFinished = Math.abs(robotPose.getY() - finalDestinationPose.getY()) < ReefAimConstants.Y_TOLERANCE_METERS.get();
        omegaFinished = Swerve.getInstance().aimingReady(ReefAimConstants.OMEGA_TOLERANCE_DEGREES.get(), 5);
        Logger.recordOutput("ReefAimCommand/xFinished", xFinished);
        Logger.recordOutput("ReefAimCommand/yFinished", yFinished);
        Logger.recordOutput("ReefAimCommand/omegaFinished", omegaFinished);
        Logger.recordOutput("ReefAimCommand/emergencyStopped", stop.getAsBoolean());
        return (xFinished && yFinished && omegaFinished) || stop.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0.0, true, false);
        swerve.setLockHeading(false);
        if (!interrupted) indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMED);
        else indicatorSubsystem.setPattern(IndicatorIO.Patterns.NORMAL);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}