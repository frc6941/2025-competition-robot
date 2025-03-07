package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeSubsystem;

import java.util.function.BooleanSupplier;

public class ShootHoldCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final Timer timer = new Timer();
    private final BooleanSupplier shoot;
    private boolean hasShoot = false;

    public ShootHoldCommand(IntakeSubsystem intakeSubsystem, BooleanSupplier shoot) {
        this.intakeSubsystem = intakeSubsystem;
        this.shoot = shoot;
    }

    @Override
    public void initialize() {
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_SHOOT);
    }

    @Override
    public void execute() {
        hasShoot = shoot.getAsBoolean() || hasShoot;
        if (hasShoot) {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.SHOOT);
            if (!intakeSubsystem.hasCoralBB() && !timer.isRunning()) {
                timer.start();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME);
        hasShoot = false;
        timer.stop();
        timer.reset();
        RobotContainer.intakeHasCoral = false;
    }
}