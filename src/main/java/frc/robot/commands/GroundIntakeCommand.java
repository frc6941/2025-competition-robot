package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

import static frc.robot.RobotConstants.ElevatorConstants.HOME_EXTENSION_METERS;
import static frc.robot.RobotConstants.ElevatorConstants.IDLE_EXTENSION_METERS;

public class GroundIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public GroundIntakeCommand(IntakeSubsystem intakeSubsystem, EndEffectorSubsystem endEffectorSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(intakeSubsystem, endEffectorSubsystem, elevatorSubsystem);
    }

    @Override
    public void execute() {
        if(elevatorSubsystem.getIo().isNearExtension(RobotConstants.ElevatorConstants.HOME_EXTENSION_METERS.get())){
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_INTAKE);
        }
        else{
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_WITHOUT_ROLL);
        }
        endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.GROUND_INTAKE);
        elevatorSubsystem.setElevatorPosition(HOME_EXTENSION_METERS.get());
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME);
        elevatorSubsystem.setElevatorPosition(IDLE_EXTENSION_METERS.get());
        if (interrupted) {
            endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.IDLE);
        }
    }

    @Override
    public boolean isFinished() {
        return endEffectorSubsystem.hasCoral();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}