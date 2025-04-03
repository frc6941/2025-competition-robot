package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;

public class ShootCommand extends Command {
    private final EndEffectorArmSubsystem endEffectorArmSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;

    public ShootCommand(IndicatorSubsystem indicatorSubsystem, EndEffectorArmSubsystem endEffectorArmSubsystem) {
        this.endEffectorArmSubsystem = endEffectorArmSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
    }

    @Override
    public void execute() {
        endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.CORAL_SHOOT);
    }

    @Override
    public boolean isFinished() {
        return Robot.isSimulation() || endEffectorArmSubsystem.isShootFinished();
    }

    @Override
    public void end(boolean interrupted) {
//        endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.HOME);
//        indicatorSubsystem.setPattern(IndicatorIO.Patterns.SHOOT);
    }
}
