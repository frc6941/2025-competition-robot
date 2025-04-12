package frc.robot.commands.aimSequences;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.drivers.DestinationSupplier.GamePiece;
import frc.robot.drivers.GamepieceTracker;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem.WantedState;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

import java.util.function.BooleanSupplier;

// TODO: Test if Neutral state is needed when intaking Algae
public class  AutoAimShootCommand extends SequentialCommandGroup {
    public AutoAimShootCommand(
            IndicatorSubsystem indicatorSubsystem,
            EndEffectorArmSubsystem endeffectorArmSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            IntakeSubsystem intakeSubsystem,
            BooleanSupplier stop,
            CommandXboxController driverController) {

        addRequirements(endeffectorArmSubsystem, elevatorSubsystem, intakeSubsystem);

        addCommands(
                Commands.either(
                        Commands.sequence(
                                //shoot coral
                                Commands.race(
                                        new WaitUntilCommand(stop),
                                        Commands.sequence(
                                                Commands.parallel(
                                                        Commands.runOnce(() ->
                                                                DestinationSupplier.getInstance().setCurrentGamePiece(GamePiece.CORAL_SCORING)),
                                                        new ReefAimCommand(stop, elevatorSubsystem, driverController, indicatorSubsystem),
                                                        new AutoPreShootCommand(indicatorSubsystem, endeffectorArmSubsystem, intakeSubsystem, elevatorSubsystem)
                                                ),
                                                new ShootCommand(indicatorSubsystem, endeffectorArmSubsystem)
                                        ),
                                        Commands.sequence(
                                                new WaitUntilCommand(() ->
                                                        (driverController.rightTrigger().getAsBoolean() && Robot.isReal())),
                                                new ShootCommand(indicatorSubsystem, endeffectorArmSubsystem)
                                        )
                                ),
                                // and then intake algae if using super cycle
                                Commands.parallel(
                                        Commands.runOnce(() ->
                                                DestinationSupplier.getInstance().setCurrentGamePiece(GamePiece.ALGAE_INTAKING)),
                                        new AutoPreShootCommand(indicatorSubsystem, endeffectorArmSubsystem, intakeSubsystem, elevatorSubsystem),
                                        new ReefAimCommand(stop, elevatorSubsystem, driverController, indicatorSubsystem)
                                ).onlyIf(() -> DestinationSupplier.getInstance().useSuperCycle)
                        ),
                        //if dont have coral then just intake algae
                        Commands.parallel(
                                Commands.runOnce(() ->
                                        DestinationSupplier.getInstance().setCurrentGamePiece(GamePiece.ALGAE_INTAKING)),
                                new AutoPreShootCommand(indicatorSubsystem, endeffectorArmSubsystem, intakeSubsystem, elevatorSubsystem),
                                new ReefAimCommand(stop, elevatorSubsystem, driverController, indicatorSubsystem)
                        ),
                        endeffectorArmSubsystem::hasCoral
                ).finallyDo(() -> {
                    endeffectorArmSubsystem.setWantedState(WantedState.HOLD);
                    if (!GamepieceTracker.getInstance().isEndeffectorHasCoral() && !GamepieceTracker.getInstance().isEndeffectorHasAlgae()) {
                        elevatorSubsystem.setElevatorPosition(RobotConstants.ElevatorConstants.HOLD_EXTENSION_METERS.get());
                    } else {
                        elevatorSubsystem.setElevatorPosition(RobotConstants.ElevatorConstants.HOLD_EXTENSION_METERS.get());
                    }
                })
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
