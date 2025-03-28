package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.ElevatorConstants.IDLE_EXTENSION_METERS;


public class AlgaeIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final EndEffectorArmSubsystem endEffectorArmSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;

    public AlgaeIntakeCommand(IndicatorSubsystem indicatorSubsystem, IntakeSubsystem intakeSubsystem, EndEffectorArmSubsystem endEffectorSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.endEffectorArmSubsystem = endEffectorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        addRequirements(intakeSubsystem, endEffectorSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
       Logger.recordOutput("Commands/AlgaeIntake", "initialize");

    }

    @Override
    public void execute() {
       Logger.recordOutput("Commands/AlgaeIntake", "execute");

       elevatorSubsystem.setElevatorPosition(DestinationSupplier.getInstance().getElevatorSetpoint(true));

       endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.ALGAE_INTAKE);

    }

    @Override
    public boolean isFinished() {
        if (endEffectorArmSubsystem.hasAlgae()) {
           return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
       Logger.recordOutput("Commands/AlgaeIntake", "end");
       elevatorSubsystem.setElevatorPosition(IDLE_EXTENSION_METERS.get());
    }
}
