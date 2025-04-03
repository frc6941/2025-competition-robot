package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.ElevatorConstants.IDLE_EXTENSION_METERS;

public class PreShootCommand extends Command {
    private final EndEffectorArmSubsystem endEffectorArmSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;

    public PreShootCommand(IndicatorSubsystem indicatorSubsystem, EndEffectorArmSubsystem endEffectorArmSubsystem, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.endEffectorArmSubsystem = endEffectorArmSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Commands/Preshoot", "initialize");
//        indicatorSubsystem.setPattern(IndicatorIO.Patterns.PRE_SHOOT);
        endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.CORAL_PRESHOOT);
    }

    @Override
    public void execute() {
        Logger.recordOutput("Commands/Preshoot", "execute");
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME);
        elevatorSubsystem.setElevatorPosition(DestinationSupplier.getInstance().getElevatorSetpoint(true));
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Commands/Preshoot", "end");
        elevatorSubsystem.setElevatorPosition(IDLE_EXTENSION_METERS.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
