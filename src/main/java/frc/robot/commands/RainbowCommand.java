package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;

public class RainbowCommand extends Command {
	private final IndicatorSubsystem indicatorSubsystem;

	private final Timer timer = new Timer();

	public RainbowCommand(IndicatorSubsystem indicatorSubsystem) {
		this.indicatorSubsystem = indicatorSubsystem;
	}

	@Override
	public void initialize() {
		timer.restart();
		indicatorSubsystem.setPattern(IndicatorIO.Patterns.SHOOTING);
	}

	@Override
	public void end(boolean interrupted) {
		indicatorSubsystem.resetToLastPattern();
	}

	@Override
	public boolean isFinished() {
		return timer.hasElapsed(2);
	}
}
