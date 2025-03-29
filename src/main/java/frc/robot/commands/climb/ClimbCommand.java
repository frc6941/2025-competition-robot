package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem.WantedState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;


import static frc.robot.RobotConstants.ElevatorConstants.HOME_EXTENSION_METERS;

public class ClimbCommand extends Command{
    private final ClimberSubsystem climberSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final EndEffectorArmSubsystem endEffectorArmSubsystem;
    public ClimbCommand(ClimberSubsystem climberSubsystem,ElevatorSubsystem elevatorSubsystem,
                        IntakeSubsystem intakeSubsystem,EndEffectorArmSubsystem endEffectorArmSubsystem){
        this.climberSubsystem = climberSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorArmSubsystem = endEffectorArmSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(climberSubsystem, elevatorSubsystem, endEffectorArmSubsystem, intakeSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setElevatorPosition(HOME_EXTENSION_METERS.get());
        climberSubsystem.setWantedState(WantedState.CLIMB);
        intakeSubsystem.setWantedState(frc.robot.subsystems.intake.IntakeSubsystem.WantedState.HOME);
        endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.HOME);
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
