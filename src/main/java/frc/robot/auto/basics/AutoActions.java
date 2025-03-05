package frc.robot.auto.basics;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.commands.*;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.BooleanSupplier;

public class AutoActions {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final AprilTagVision aprilTagVision;
    private final Swerve swerve;

    public AutoActions(IndicatorSubsystem indicatorSubsystem, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, IntakeSubsystem intakeSubsystem, AprilTagVision aprilTagVision) {
        this.intakeSubsystem = intakeSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.aprilTagVision = aprilTagVision;
        this.swerve = Swerve.getInstance();
    }

    public Command shootCoralAtSetpoint() {
        return new AutoShootCoralCommand(elevatorSubsystem, endEffectorSubsystem);
    }

    // invoke event marker
    public void invokeCommand(String name, BooleanSupplier stopSupplier) {
        switch (name) {
            case "DEPLOY-INTAKE":
                deployIntake().until(stopSupplier).schedule();
                break;
            case "PRESHOOT":
                preShoot().until(stopSupplier).schedule();
                break;
            case "ZEROELEVATOR":
                zeroElevator().until(stopSupplier).schedule();
                break;
            case "DEPLOY-INTAKE-INIT":
                //zeroAndIntake().until(stopSupplier).schedule();
                deployIntake().until(stopSupplier).schedule();
                break;
        }
    }

    public Command initializeVision() {
        return Commands.runOnce(() -> aprilTagVision.setMeasureCnt(0));
    }

    public Command followPath(PathPlannerPath path, boolean angleLock, boolean requiredOnTarget, boolean resetOdometry) {
        return new FollowPath(this, swerve, path, angleLock, requiredOnTarget, resetOdometry);
    }

    public Command waitFor(double seconds) {
        return new WaitCommand(seconds);
    }

    public Command setL4() {
        return Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L4));
    }

    public Command zeroElevator() {
        return new ZeroElevatorCommand(elevatorSubsystem, intakeSubsystem, endEffectorSubsystem);
    }

    public Command zeroAndIntake() {
        return Commands.sequence(
                new ZeroCommand(elevatorSubsystem, intakeSubsystem, endEffectorSubsystem),
                new WaitUntilCommand(() -> (elevatorSubsystem.hasReachedNearZero && intakeSubsystem.hasHomed)),
                new WaitUntilCommand(() -> (!elevatorSubsystem.hasReachedNearZero && !intakeSubsystem.hasHomed)),
                new GroundIntakeCommand(indicatorSubsystem, intakeSubsystem, endEffectorSubsystem, elevatorSubsystem));
    }

    public Command deployIntake() {
        return new GroundIntakeCommand(indicatorSubsystem, intakeSubsystem, endEffectorSubsystem, elevatorSubsystem);
    }

    public Command preShoot() {
        return new PreShootCommand(indicatorSubsystem, endEffectorSubsystem, intakeSubsystem, elevatorSubsystem);
    }

    public Command shootCoral() {
        return new ShootCommand(indicatorSubsystem, endEffectorSubsystem);
    }

    public Command putCoral() {
        return Commands.race(preShoot(), shootCoralAtSetpoint());
    }

    public Command setLevel(DestinationSupplier.elevatorSetpoint setpoint) {
        return Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(setpoint));
    }

    public Command AutoAimShoot(DestinationSupplier.elevatorSetpoint setpoint, char tagChar) {
        return Commands.sequence(
                Commands.parallel(
                        setLevel(setpoint),
                        new ReefAimAutoCommand(elevatorSubsystem, tagChar),
                        new AutoPreShootCommand(indicatorSubsystem, endEffectorSubsystem, intakeSubsystem, elevatorSubsystem)
                ),
                new ShootCommand(indicatorSubsystem, endEffectorSubsystem),
                Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(
                        RobotConstants.ElevatorConstants.IDLE_EXTENSION_METERS.get())));
    }

    public Command ReverseEndEffector() {
        return new ReverseEndEffectorCommand(endEffectorSubsystem);
    }
}
