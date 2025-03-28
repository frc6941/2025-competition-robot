package frc.robot.auto.fullAutos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.auto.basics.FollowPath;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.coral.AutoPreShootCoralCommand;
import frc.robot.commands.coral.AutoReefAimCoralCommand;
import frc.robot.commands.coral.ManualPreShootCoralCommand;
import frc.robot.commands.coral.ShootCoralCommand;
import frc.robot.drivers.DestinationSupplier;
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
    private final Swerve swerve;
    private final DestinationSupplier destinationSupplier = DestinationSupplier.getInstance();

    public AutoActions(IndicatorSubsystem indicatorSubsystem, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.swerve = Swerve.getInstance();
    }

    public Command shootCoralAtSetpoint() {
        return new AutonomousShootCoralCommand(elevatorSubsystem, endEffectorSubsystem);
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
                zeroAndIntake().until(stopSupplier).schedule();
                //deployIntake().until(stopSupplier).schedule();
                break;
        }
    }

    public Command followPath(PathPlannerPath path, boolean angleLock, boolean requiredOnTarget, boolean resetOdometry) {
        return new FollowPath(this, swerve, path, angleLock, requiredOnTarget, resetOdometry);
    }

    public Command waitFor(double seconds) {
        return new WaitCommand(seconds);
    }

    public Command setL4() {
        return Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L4));
    }

    public Command zeroElevator() {
        return new ZeroElevatorCommand(elevatorSubsystem, intakeSubsystem, endEffectorSubsystem);
    }

    public Command zeroAndIntake() {
        return Commands.sequence(
                new ZeroElevatorCommand(elevatorSubsystem, intakeSubsystem, endEffectorSubsystem),
                new WaitUntilCommand(() -> (elevatorSubsystem.getSystemState() != ElevatorSubsystem.SystemState.ZEROING &&
                        intakeSubsystem.getSystemState() == IntakeSubsystem.SystemState.AVOIDING)),
                new AutonomousGroundIntakeCommand(indicatorSubsystem, intakeSubsystem, endEffectorSubsystem, elevatorSubsystem));
    }

    public Command deployIntake() {
        return new AutonomousGroundIntakeCommand(indicatorSubsystem, intakeSubsystem, endEffectorSubsystem, elevatorSubsystem);
    }

    public Command preShoot() {
        return new ManualPreShootCoralCommand(indicatorSubsystem, endEffectorSubsystem, intakeSubsystem, elevatorSubsystem);
    }

    public Command shootCoral() {
        return new ShootCoralCommand(indicatorSubsystem, endEffectorSubsystem);
    }

    public Command putCoral() {
        return Commands.race(preShoot(), shootCoralAtSetpoint());
    }

    public Command setLevel(DestinationSupplier.elevatorSetpoint setpoint) {
        return Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(setpoint));
    }

    public Command AutoAimShoot(DestinationSupplier.elevatorSetpoint setpoint, char tagChar) {
        return Commands.sequence(
                Commands.parallel(
                        setLevel(setpoint),
                        new AutonomousReefAimCommand(elevatorSubsystem, tagChar),
                        new AutoPreShootCoralCommand(indicatorSubsystem, endEffectorSubsystem, intakeSubsystem, elevatorSubsystem)
                ),
                new WaitCommand(0.05),
                new ShootCoralCommand(indicatorSubsystem, endEffectorSubsystem),
                new WaitCommand(0.05),
                Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(
                        RobotConstants.ElevatorConstants.IDLE_EXTENSION_METERS.get())));
    }

//    public Command ReverseEndEffector() {
//        return new ReverseEndEffectorCommand(endEffectorSubsystem);
//    }

    public Command homeEverything() {
        return Commands.parallel(Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME)),
                Commands.runOnce(() -> elevatorSubsystem.setElevatorState(ElevatorSubsystem.WantedState.IDLE)));
    }

    public boolean intakerHasCoral() {
        return intakeSubsystem.hasCoralBB();
    }

    public EndEffectorSubsystem.SystemState getEESystemState() {
        return endEffectorSubsystem.getSystemState();
    }

    public boolean isIntakeFinished() {
        return endEffectorSubsystem.isIntakeFinished();
    }
}
