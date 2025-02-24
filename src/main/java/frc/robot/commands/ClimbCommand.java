package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem.WantedState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

import static frc.robot.RobotConstants.ElevatorConstants.IDLE_EXTENSION_METERS;

public class ClimbCommand extends Command{
    private final ClimberSubsystem climberSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    public ClimbCommand(ClimberSubsystem climberSubsystem,ElevatorSubsystem elevatorSubsystem,
    IntakeSubsystem intakeSubsystem,EndEffectorSubsystem endEffectorSubsystem){
        this.climberSubsystem = climberSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(climberSubsystem, elevatorSubsystem, endEffectorSubsystem, intakeSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setElevatorPosition(IDLE_EXTENSION_METERS.get());
        climberSubsystem.setWantedState(WantedState.CLIMB);
        intakeSubsystem.setWantedState(frc.robot.subsystems.intake.IntakeSubsystem.WantedState.HOME);
        endEffectorSubsystem.setWantedState(frc.robot.subsystems.endeffector.EndEffectorSubsystem.WantedState.IDLE);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setWantedState(WantedState.DEPLOY);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
