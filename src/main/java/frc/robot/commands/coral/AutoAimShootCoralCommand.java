package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.commands.coral.ShootCoralCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.commands.*;

import java.util.function.BooleanSupplier;

public class AutoAimShootCoralCommand extends ParallelCommandGroup {
    public AutoAimShootCoralCommand(IndicatorSubsystem indicatorSubsystem, EndEffectorSubsystem endeffectorSubsystem,
                               ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, BooleanSupplier stop, CommandXboxController driverController) {
        addRequirements(endeffectorSubsystem, elevatorSubsystem, intakeSubsystem);
        addCommands(
                Commands.race(
                        new WaitUntilCommand(stop),
                        Commands.sequence(
                                Commands.parallel(
                                        new AutoReefAimCoralCommand(stop, elevatorSubsystem, driverController, indicatorSubsystem),
                                        new AutoPreShootCoralCommand(indicatorSubsystem, endeffectorSubsystem, intakeSubsystem, elevatorSubsystem)
                                ),
                                new ShootCoralCommand(indicatorSubsystem, endeffectorSubsystem)
                        ),
                        Commands.sequence(
                                new WaitUntilCommand(() -> (driverController.rightTrigger().getAsBoolean() && Robot.isReal())),
                                new ShootCoralCommand(indicatorSubsystem, endeffectorSubsystem)
                        )

                ).finallyDo(() -> elevatorSubsystem.setElevatorPosition(
                        RobotConstants.ElevatorConstants.IDLE_EXTENSION_METERS.get()))
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
