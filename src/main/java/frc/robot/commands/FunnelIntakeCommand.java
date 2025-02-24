package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.WantedState;

import static frc.robot.RobotConstants.ElevatorConstants.FUNNEL_INTAKE_EXTENSION_METERS;
import static frc.robot.RobotConstants.ElevatorConstants.IDLE_EXTENSION_METERS;

public class FunnelIntakeCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public FunnelIntakeCommand(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem, endEffectorSubsystem, elevatorSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setWantedState(WantedState.FUNNEL_AVOID);
        endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.FUNNEL_INTAKE);
        elevatorSubsystem.setElevatorPosition(FUNNEL_INTAKE_EXTENSION_METERS.get());
    }

    @Override
    public boolean isFinished() {
        return endEffectorSubsystem.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setWantedState(WantedState.HOME);
        elevatorSubsystem.setElevatorPosition(IDLE_EXTENSION_METERS.get());
        if(interrupted){
            endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.IDLE);
        }
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
