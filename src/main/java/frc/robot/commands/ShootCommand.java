package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;

public class ShootCommand extends Command {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;

    public ShootCommand(IndicatorSubsystem indicatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
    }

    @Override
    public void execute() {
        endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.SHOOT);
    }

    @Override
    public boolean isFinished() {
        return Robot.isSimulation() || endEffectorSubsystem.isShootFinished();
    }

    @Override
    public void end(boolean interrupted) {
        endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.IDLE);
//        indicatorSubsystem.setPattern(IndicatorIO.Patterns.SHOOT);
    }
}
