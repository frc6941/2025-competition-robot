package frc.robot.auto.basics;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.GroundIntakeCommand;
import frc.robot.commands.PreShootCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.DestinationSupplier;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import static frc.robot.RobotConstants.ElevatorConstants.HOME_EXTENSION_METERS;
import static frc.robot.RobotConstants.ElevatorConstants.L4_EXTENSION_METERS;

public class AutoActions {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final Swerve swerve;
    public boolean initialized = false;
    private Map<String, Command> autoCommands = new HashMap<>();

    public AutoActions(IndicatorSubsystem indicatorSubsystem, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.swerve = Swerve.getInstance();
        initializeAutoCommands();
    }

    public void initializeAutoCommands() {
        if (initialized) {
            throw new IllegalStateException("AutoActions already initialized");
        }
        // FIXME: do not put commands into a hashmap (crash)
        autoCommands = Map.ofEntries(
                Map.entry(
                        "EE-GROUND-INTAKE", Commands.runOnce(() -> endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.GROUND_INTAKE))
                ),
                Map.entry(
                        "EE-PRE-SHOOT", Commands.runOnce(() -> endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.PRE_SHOOT))
                ),
                Map.entry(
                        "EE-IDLE", Commands.runOnce(() -> endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.IDLE))
                ),
                Map.entry(
                        "EE-SHOOT", Commands.runOnce(() -> endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.SHOOT))
                ),
                Map.entry(
                        "ELE-ZERO", Commands.runOnce(() -> elevatorSubsystem.setElevatorState(ElevatorSubsystem.WantedState.ZERO))
                ),
                Map.entry(
                        "ELE-L1", Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L1)).andThen(preShoot())
                ),
                Map.entry(
                        "ELE-L2", Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L2)).andThen(preShoot())
                ),
                Map.entry(
                        "ELE-L3", Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L3)).andThen(preShoot())
                ),
                Map.entry(
                        "ELE-L4", Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L4)).andThen(preShoot())
                ),
                Map.entry(
                        "ELE-HOME", Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(HOME_EXTENSION_METERS.get()))
                ),
                Map.entry(
                        "INTAKE-DEPLOY", deployIntake()
                ),
                Map.entry(
                        "INTAKE-HOME", Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME))
                ),
                Map.entry(
                        "INTAKE-ZERO", Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.GROUNDZERO))
                )
        );
    }

    public void invokeCommand(String name, BooleanSupplier stopSupplier) {
        if (autoCommands.get(name).isScheduled()) {
            System.out.println(name + " already scheduled.");
            return;
        }
        assert autoCommands.containsKey(name);
        switch (name) {
            // FIXME: do not do that it will crash.
            case "INTAKE-DEPLOY":
                deployIntake().until(stopSupplier).schedule();
                break;
            default:
                autoCommands.get(name).until(stopSupplier).schedule();
        }
    }

    public Command followPath(PathPlannerPath path, boolean angleLock, boolean requiredOnTarget, boolean resetOdometry) {
        return new FollowPath(this, swerve, path, angleLock, requiredOnTarget, resetOdometry);
    }

    public Command waitFor(double seconds) {
        return new WaitCommand(seconds);
    }

    public Command raiseElevator() {
        return Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(L4_EXTENSION_METERS.get()));
    }

    public Command deployIntake() {
        return new GroundIntakeCommand(indicatorSubsystem, intakeSubsystem, endEffectorSubsystem, elevatorSubsystem);
    }

    public Command preShoot() {
        return new PreShootCommand(indicatorSubsystem, endEffectorSubsystem, intakeSubsystem, elevatorSubsystem);
    }

    public Command shootCoral() {
        return Commands.runOnce(() -> endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.SHOOT));
    }

    public Command shootCoralAtSetpoint() {
        return shootCoral().onlyIf(() -> elevatorSubsystem.getIo().isNearExtension(elevatorSubsystem.getWantedPosition()))
                .andThen(
                        new InstantCommand(() -> preShoot().cancel())
                );
    }
}