package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.drivers.DestinationSupplier.GamePiece;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.Swerve;

public class AutoPreShootCommand extends Command {
    // Subsystems
    private final EndEffectorArmSubsystem endEffectorArmSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;

    // State variables
    private final Timer timer = new Timer();
    private boolean safeToRaise = false;

    public AutoPreShootCommand(
            IndicatorSubsystem indicatorSubsystem,
            EndEffectorArmSubsystem endEffectorArmSubsystem,
            IntakeSubsystem intakeSubsystem,
            ElevatorSubsystem elevatorSubsystem) {
        this.endEffectorArmSubsystem = endEffectorArmSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
    }

    @Override
    public void initialize() {
        timer.start();
        DestinationSupplier.getInstance().updatePokeSetpointByTag(
                DestinationSupplier.getNearestTagID(
                        Swerve.getInstance().getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp())));
        // indicatorSubsystem.setPattern(IndicatorIO.Patterns.PRE_SHOOT);
    }

    @Override
    public void execute() {
        safeToRaise = DestinationSupplier.isSafeToRaise(
                Swerve.getInstance().getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp()),
                DestinationSupplier.getInstance().getCurrentBranch());

        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME);

        if (DestinationSupplier.getInstance().getCurrentGamePiece() == GamePiece.CORAL_SCORING) {
            if (safeToRaise) {
                elevatorSubsystem.setElevatorPosition(DestinationSupplier.getInstance().getElevatorSetpoint(true));
                endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.CORAL_PRESHOOT);
            }
        } else {
            if (safeToRaise) {
                elevatorSubsystem.setElevatorPosition(DestinationSupplier.getInstance().getElevatorSetpoint(false));
                endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.ALGAE_INTAKE);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // No cleanup needed
    }

    @Override
    public boolean isFinished() {
        if (DestinationSupplier.getInstance().getCurrentGamePiece() == GamePiece.CORAL_SCORING) {
            return timer.hasElapsed(0.1)
                    && elevatorSubsystem.elevatorReady(0.005)
                    && endEffectorArmSubsystem.isShootReady()
                    && safeToRaise;
        } else {
            return endEffectorArmSubsystem.hasAlgae();
        }
    }
}