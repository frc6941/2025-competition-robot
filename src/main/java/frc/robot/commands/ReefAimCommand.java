package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.RobotConstants;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO.Patterns;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.AllianceFlipUtil;

import java.util.function.BooleanSupplier;

public class ReefAimCommand extends Command {
    private final Swerve swerve = Swerve.getInstance();
    private final IndicatorSubsystem indicatorSubsystem;
    private final int tagID;
    private final boolean rightReef; // true if shooting right reef
    private final PIDController xPID = new PIDController(
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
    private final PIDController yPID = new PIDController(
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
    private final BooleanSupplier stop;
    private Pose2d robotPose;
    private Pose2d tagPose;
    private Pose2d destinationPose;
    private Translation2d translationalVelocity;

    // Constructor for ReefAimCommand
    public ReefAimCommand(
        int tagID, 
        boolean rightReef, 
        IndicatorSubsystem indicatorSubsystem,
        BooleanSupplier stop) {
        addRequirements(this.swerve);
        this.tagID = tagID;
        this.rightReef = rightReef;
        this.stop = stop;
        this.indicatorSubsystem = indicatorSubsystem;
        SmartDashboard.putNumber("ReefAimCommand/tagID", tagID);
        SmartDashboard.putBoolean("ReefAimCommand/rightReef", rightReef);
    }

    @Override
    public void initialize() {
        indicatorSubsystem.setPattern(Patterns.AIMING);
        tagPose = FieldConstants.defaultAprilTagType.getLayout().getTagPose(tagID).get().toPose2d();
        if (rightReef) {
            destinationPose = tagPose.transformBy(RobotConstants.ReefAimConstants.tagRightToRobot);
        } else {
            destinationPose = tagPose.transformBy(RobotConstants.ReefAimConstants.tagLeftToRobot);
        }
        xPID.setSetpoint(destinationPose.getTranslation().getX());
        xPID.setTolerance(0.02);
        yPID.setSetpoint(destinationPose.getTranslation().getY());
        yPID.setTolerance(0.02);
        SmartDashboard.putString("ReefAimCommand/destinationPose", destinationPose.toString());
        swerve.setLockHeading(true);
        swerve.setHeadingTarget(destinationPose.getRotation().getDegrees() - 180.0);
    }

    @Override
    public void execute() {
        if (RobotConstants.TUNING) {
            xPID.setP(RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get());
            xPID.setI(RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get());
            xPID.setD(RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
            yPID.setP(RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get());
            yPID.setI(RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get());
            yPID.setD(RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
        }
        robotPose = swerve.getLocalizer().getCoarseFieldPose(0);
        translationalVelocity = new Translation2d(xPID.calculate(robotPose.getX() - 0.45), yPID.calculate(robotPose.getY()));
        SmartDashboard.putString("ReefAimCommand/RobotPose", robotPose.toString());
        SmartDashboard.putString("ReefAimCommand/translationalVelocity", translationalVelocity.toString());
        swerve.drive(AllianceFlipUtil.shouldFlip() ? translationalVelocity.unaryMinus() : translationalVelocity, 0.0, true, false);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("ReefAimCommand/xFinished", xPID.atSetpoint());
        SmartDashboard.putBoolean("ReefAimCommand/yFinished", yPID.atSetpoint());
        SmartDashboard.putBoolean("ReefAimCommand/omegaFinished", Swerve.getInstance().aimingReady(1));
        SmartDashboard.putBoolean("ReefAimCommand/emergencyStopped", stop.getAsBoolean());
        return (xPID.atSetpoint() && yPID.atSetpoint() && Swerve.getInstance().aimingReady(1)) || stop.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setLockHeading(false);
        indicatorSubsystem.setPattern(Patterns.NORMAL);
    }

    @Override

    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}