package frc.robot.commands.groundintaker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;

public class CoralIntakeCommand extends Command {
    public final IntakeSubsystem intakeSubsystem;
    public final IndicatorSubsystem indicatorSubsystem;
    public final EndEffectorArmSubsystem endEffectorArmSubsystem;
    public final ElevatorSubsystem elevatorSubsystem;

    public CoralIntakeCommand(IntakeSubsystem intakeSubsystem, IndicatorSubsystem indicatorSubsystem, EndEffectorArmSubsystem endEffectorArmSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.endEffectorArmSubsystem = endEffectorArmSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (endEffectorArmSubsystem.hasCoral() || endEffectorArmSubsystem.hasAlgae()) {
            return;
        }
        if (elevatorSubsystem.getIo().isNearExtension(RobotConstants.ElevatorConstants.HOME_EXTENSION_METERS.get())) {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_INTAKE);
        } else {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_WITHOUT_ROLL);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME);

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
