package frc.robot.auto.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.display.Display;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.ReefAimConstants;

public class ReefAimAlgaeAutoCommand extends Command {
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
    private final ElevatorSubsystem elevatorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private boolean xFinished = false;
    private boolean yFinished = false;
    private boolean omegaFinished = false;
    private Pose2d robotPose, tagPose, destinationPose, finalDestinationPose;
    private Translation2d translationalVelocity;


    public ReefAimAlgaeAutoCommand(ElevatorSubsystem elevatorSubsystem, IndicatorSubsystem indicatorSubsystem) {
        addRequirements(swerve);
        this.elevatorSubsystem = elevatorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
    }

    @Override
    public void initialize() {
        // Calculate destination
        robotPose = swerve.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp());
        tagPose = DestinationSupplier.getNearestTag(robotPose);
        // PID init
        xPID.reset(robotPose.getX(), swerve.getLocalizer().getMeasuredVelocity().getX());
        yPID.reset(robotPose.getY(), swerve.getLocalizer().getMeasuredVelocity().getY());
        // Choose algae target
        finalDestinationPose = DestinationSupplier.getFinalAlgaeTarget(tagPose);
        indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
    }

    @Override
    public void execute() {
        // TODO: combine ReefAimAlgaeAutoCommand ReefAimCoralAutoCommand ReefAimCommand by using extend
        xPID.setConstraints(
                new TrapezoidProfile.Constraints(
                            ReefAimConstants.MAX_AIMING_SPEED.magnitude() * 0.6 / elevatorSubsystem.getElevatorPosition(),
                        ReefAimConstants.MAX_AIMING_ACCELERATION.magnitude() * 0.6 / elevatorSubsystem.getElevatorPosition()));
        yPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        ReefAimConstants.MAX_AIMING_SPEED.magnitude() * 0.6 / elevatorSubsystem.getElevatorPosition(),
                        ReefAimConstants.MAX_AIMING_ACCELERATION.magnitude() * 0.6 / elevatorSubsystem.getElevatorPosition()));
        if (RobotConstants.TUNING) {
            xPID.setPID(RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
            yPID.setPID(RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
        }

        robotPose = swerve.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp());
        destinationPose = DestinationSupplier.getDriveTarget(robotPose, finalDestinationPose);

        xPID.setGoal(destinationPose.getTranslation().getX());
        yPID.setGoal(destinationPose.getTranslation().getY());
        swerve.setLockHeading(true);
        swerve.setHeadingTarget(destinationPose.getRotation().getDegrees());
        translationalVelocity = new Translation2d(xPID.calculate(robotPose.getX()), yPID.calculate(robotPose.getY()));
        swerve.drive(translationalVelocity, 0.0, true, false);
        Display.getInstance().setAimingTarget(destinationPose);
        Logger.recordOutput("ReefAimAlgaeAutoCommand/tagPose", tagPose);
        Logger.recordOutput("ReefAimAlgaeAutoCommand/destinationPose", destinationPose);
        Logger.recordOutput("ReefAimAlgaeAutoCommand/finalDestinationPose", finalDestinationPose);
        Logger.recordOutput("ReefAimAlgaeAutoCommand/translationalVelocity", translationalVelocity);
    }

    @Override
    public boolean isFinished() {
        xFinished = Math.abs(robotPose.getX() - finalDestinationPose.getX()) < ReefAimConstants.X_TOLERANCE_METERS.get();
        yFinished = Math.abs(robotPose.getY() - finalDestinationPose.getY()) < ReefAimConstants.Y_TOLERANCE_METERS.get();
        omegaFinished = Swerve.getInstance().aimingReady(ReefAimConstants.OMEGA_TOLERANCE_DEGREES.get(), 5);
        Logger.recordOutput("ReefAimAlgaeAutoCommand/xFinished", xFinished);
        Logger.recordOutput("ReefAimAlgaeAutoCommand/yFinished", yFinished);
        Logger.recordOutput("ReefAimAlgaeAutoCommand/omegaFinished", omegaFinished);
        return xFinished && yFinished && omegaFinished;
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