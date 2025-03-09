package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.RobotConstants;
import frc.robot.display.Display;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.ReefAimConstants;

public class ReefAimAutoCommand extends Command {
    private static final int[] FLIP_TAG_NUMBERS = {7, 8, 9, 10, 11, 6};
    private static final int[] NON_FLIP_TAG_NUMBERS = {18, 17, 22, 21, 20, 19};
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
    private final boolean rightReef; // true if shooting right reef
    private final int tagNumber;
    private boolean xFinished = false;
    private boolean yFinished = false;
    private boolean omegaFinished = false;
    private Translation2d translationalVelocity;
    private Pose2d robotPose, tagPose, destinationPose, finalDestinationPose;

    public ReefAimAutoCommand(ElevatorSubsystem elevatorSubsystem, char tagChar) {
        addRequirements(swerve);
        this.elevatorSubsystem = elevatorSubsystem;
        rightReef = (tagChar % 2) == 0;
        tagNumber = AllianceFlipUtil.shouldFlip()
                ? FLIP_TAG_NUMBERS[(tagChar - 'A') / 2]
                : NON_FLIP_TAG_NUMBERS[(tagChar - 'A') / 2];
    }

    @Override
    public void initialize() {
        // Calculate destination
        robotPose = swerve.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp());
        tagPose = FieldConstants.officialAprilTagType.getLayout().getTagPose(tagNumber).get().toPose2d();
        // PID init
        xPID.reset(robotPose.getX(), swerve.getLocalizer().getMeasuredVelocity().getX());
        yPID.reset(robotPose.getY(), swerve.getLocalizer().getMeasuredVelocity().getY());
        finalDestinationPose = DestinationSupplier.getFinalDriveTarget(tagPose, rightReef);
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
        translationalVelocity = new Translation2d(xPID.calculate(robotPose.getX()), yPID.calculate(robotPose.getY()));
        swerve.drive(translationalVelocity, 0.0, true, false);
        Display.getInstance().setAimingTarget(destinationPose);
        Logger.recordOutput("ReefAimCommand/tagPose", tagPose);
        Logger.recordOutput("ReefAimCommand/destinationPose", destinationPose);
        Logger.recordOutput("ReefAimCommand/finalDestinationPose", finalDestinationPose);
        Logger.recordOutput("ReefAimCommand/translationalVelocity", translationalVelocity);
    }

    @Override
    public boolean isFinished() {
        xFinished = Math.abs(robotPose.getX() - finalDestinationPose.getX()) < 0.02;
        yFinished = Math.abs(robotPose.getY() - finalDestinationPose.getY()) < 0.02;
        omegaFinished = Swerve.getInstance().aimingReady(ReefAimConstants.OMEGA_TOLERANCE_DEGREES.get(), 5);
        Logger.recordOutput("ReefAimCommandAuto/xFinished", xFinished);
        Logger.recordOutput("ReefAimCommandAuto/yFinished", yFinished);
        Logger.recordOutput("ReefAimCommandAuto/omegaFinished", omegaFinished);
        return xFinished && yFinished && omegaFinished;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0.0, true, false);
        swerve.setLockHeading(false);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}