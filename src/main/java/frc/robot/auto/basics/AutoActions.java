package frc.robot.auto.basics;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import lombok.Synchronized;

import java.util.Map;

import static frc.robot.RobotConstants.ElevatorConstants.*;

public class AutoActions {
    public static boolean initialized = false;
    private final RobotContainer m_container;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    AutoActions(RobotContainer container) {
        this.m_container = container;
        this.intakeSubsystem = m_container.getIntakeSubsystem();
        this.endEffectorSubsystem = m_container.getEndEffectorSubsystem();
        this.elevatorSubsystem = m_container.getElevatorSubsystem();
        initializeAutoCommands();
    }

    public void initializeAutoCommands() {
        if (initialized) {
            throw new IllegalStateException("AutoActions already initialized");
        }
        Map<String, Command> autoCommands = Map.ofEntries(
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
                        "ELE-L1", Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(L1_EXTENSION_METERS.get()))
                ),
                Map.entry(
                        "ELE-L2", Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(L2_EXTENSION_METERS.get()))
                ),
                Map.entry(
                        "ELE-L3", Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(L3_EXTENSION_METERS.get()))
                ),
                Map.entry(
                        "ELE-L4", Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(L4_EXTENSION_METERS.get()))
                ),
                Map.entry(
                        "ELE-HOME", Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(HOME_EXTENSION_METERS.get()))
                ),
                Map.entry(
                        "INTAKE-DEPLOY", Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_INTAKE))
                ),
                Map.entry(
                        "INTAKE-HOME", Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME))
                ),
                Map.entry(
                        "INTAKE-ZERO", Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.GROUNDZERO))
                )
        );
        NamedCommands.registerCommands(autoCommands);

    }



    public Command setElevator(double meters) {
        return Commands.runOnce(()-> elevatorSubsystem.setElevatorPosition(meters));
    }

    public Command waitFor(double seconds) {
        return new WaitCommand(seconds);
    }
}