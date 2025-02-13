package frc.robot.commands.factory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorCommandFactory {
    public static Command setElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, double position) {
        return new InstantCommand(() -> elevatorSubsystem.setElevatorPosition(position)).until(() -> elevatorSubsystem.getIo().isNearExtension(position));
    }

    public static Command setElevatorStateCommand(ElevatorSubsystem elevatorSubsystem, ElevatorSubsystem.WantedState wantedState) {
        return new InstantCommand(() -> elevatorSubsystem.setElevatorState(wantedState));
    }
}
