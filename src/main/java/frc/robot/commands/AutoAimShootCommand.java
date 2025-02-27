package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

import java.util.function.BooleanSupplier;

public class AutoAimShootCommand extends ParallelCommandGroup {

    public AutoAimShootCommand(IndicatorSubsystem indicatorSubsystem, EndEffectorSubsystem endeffectorSubsystem,
                               ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, BooleanSupplier stop) {
        addRequirements(endeffectorSubsystem, elevatorSubsystem, intakeSubsystem);
        addCommands(
                Commands.sequence(
                        Commands.race(
                            new WaitUntilCommand(stop),
                            Commands.sequence(
                                Commands.parallel(
                                        new ReefAimCommand(stop, elevatorSubsystem),
                                        new AutoPreShootCommand(indicatorSubsystem, endeffectorSubsystem, intakeSubsystem, elevatorSubsystem)
                                ),
                                new ShootCommand(indicatorSubsystem, endeffectorSubsystem)
                            )
                        ),
                        Commands.runOnce(() ->
                                elevatorSubsystem.setElevatorPosition(
                                        RobotConstants.ElevatorConstants.IDLE_EXTENSION_METERS.get())
                        )
                )
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
