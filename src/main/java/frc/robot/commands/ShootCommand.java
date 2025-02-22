package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO.Patterns;

public class ShootCommand extends Command {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;

    public ShootCommand(
        EndEffectorSubsystem endEffectorSubsystem,
        IndicatorSubsystem indicatorSubsystem) {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
    }

    @Override
    public void initialize() {
        indicatorSubsystem.setPattern(Patterns.SHOOTING);
    }

    @Override
    public void execute() {
        endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.SHOOT);
    }

    @Override
    public boolean isFinished() {
            return endEffectorSubsystem.isShootFinished();
        }

    @Override
    public void end(boolean interrupted){
        endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.IDLE);
        indicatorSubsystem.setPattern(Patterns.SHOOT_FINISH);
    }
}
