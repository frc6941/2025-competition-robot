package frc.robot.commands.groundintaker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;

import static frc.robot.RobotConstants.ElevatorConstants.HOME_EXTENSION_METERS;
import static frc.robot.RobotConstants.ElevatorConstants.IDLE_EXTENSION_METERS;

public class CoralPreIntakeCommand extends Command {
    public final IndicatorSubsystem indicatorSubsystem;
    public final EndEffectorArmSubsystem endEffectorArmSubsystem;
    public final ElevatorSubsystem elevatorSubsystem;

    public CoralPreIntakeCommand(IndicatorSubsystem indicatorSubsystem, EndEffectorArmSubsystem endEffectorArmSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.indicatorSubsystem = indicatorSubsystem;
        this.endEffectorArmSubsystem = endEffectorArmSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(endEffectorArmSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.CORAL_INTAKE);
        elevatorSubsystem.setElevatorPosition(HOME_EXTENSION_METERS.get());
    }

    @Override
    public void end(boolean interrupted) {
        endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.HOME);
        elevatorSubsystem.setElevatorPosition(IDLE_EXTENSION_METERS.get());
    }

    @Override
    public boolean isFinished() {
        return endEffectorArmSubsystem.hasCoral();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
