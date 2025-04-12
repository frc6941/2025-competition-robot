package frc.robot.commands.manualSequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem.WantedState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

import static frc.robot.RobotConstants.ElevatorConstants.HOLD_EXTENSION_METERS;

public class FixIntakeCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final EndEffectorArmSubsystem endEffectorArmSubsystem;

    public FixIntakeCommand(ElevatorSubsystem elevatorSubsystem,
                        IntakeSubsystem intakeSubsystem, EndEffectorArmSubsystem endEffectorArmSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorArmSubsystem = endEffectorArmSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(elevatorSubsystem, endEffectorArmSubsystem, intakeSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setElevatorPosition(HOLD_EXTENSION_METERS.get());
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_WITHOUT_ROLL);
        endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.CORAL_OUTTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setElevatorPosition(RobotConstants.ElevatorConstants.HOME_EXTENSION_METERS.get());
        endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.CORAL_INTAKE);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}