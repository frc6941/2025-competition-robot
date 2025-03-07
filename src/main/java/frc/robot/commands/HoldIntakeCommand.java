package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

import static frc.robot.RobotConstants.ElevatorConstants.HOLD_INTAKE_METERS;
import static frc.robot.RobotConstants.ElevatorConstants.IDLE_EXTENSION_METERS;

public class HoldIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private boolean hasCoral = false;

    public HoldIntakeCommand(IndicatorSubsystem indicatorSubsystem, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        addRequirements(intakeSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        indicatorSubsystem.setPattern(IndicatorIO.Patterns.INTAKE);
    }

    @Override
    public void execute() {
        if (elevatorSubsystem.getIo().isNearExtension(HOLD_INTAKE_METERS.get())) {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_INTAKE_HOLD);
        } else {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_WITHOUT_ROLL);
        }
        elevatorSubsystem.setElevatorPosition(HOLD_INTAKE_METERS.get());
        hasCoral = hasCoral || intakeSubsystem.hasCoralBB();
        if (hasCoral) {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOLD_OUTTAKE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME);
        elevatorSubsystem.setElevatorPosition(IDLE_EXTENSION_METERS.get());
        indicatorSubsystem.setPattern(IndicatorIO.Patterns.AFTER_INTAKE);
        hasCoral = false;
        if (!interrupted) RobotContainer.intakeHasCoral = true;
    }

    @Override
    public boolean isFinished() {
        return !intakeSubsystem.hasCoralBB() && hasCoral;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}