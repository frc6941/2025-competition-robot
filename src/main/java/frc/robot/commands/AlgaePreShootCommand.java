package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.ElevatorConstants.IDLE_EXTENSION_METERS;


public class AlgaePreShootCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final EndEffectorArmSubsystem endEffectorArmSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;

    public AlgaePreShootCommand(IndicatorSubsystem indicatorSubsystem, IntakeSubsystem intakeSubsystem, EndEffectorArmSubsystem endEffectorSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.endEffectorArmSubsystem = endEffectorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        addRequirements(intakeSubsystem, endEffectorSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Commands/AlgaePreshoot", "initialize");
    }


    @Override
    public void execute() {
        Logger.recordOutput("Commands/AlgaePreshoot", "execute");
        elevatorSubsystem.setElevatorPosition(DestinationSupplier.getInstance().getElevatorSetpoint(false));
        endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.ALGAE_PRESHOOT);
    }


    @Override
    public boolean isFinished() {
        if (!endEffectorArmSubsystem.hasAlgae()) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Commands/AlgaePreshoot", "end");
        elevatorSubsystem.setElevatorPosition(IDLE_EXTENSION_METERS.get());
    }
}
