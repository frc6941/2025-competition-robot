package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.Swerve;

public class AutoPreShootCommand extends Command {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    Timer timer = new Timer();
    private boolean safeToRaise = false;
    private double ControllerX;
    private double ControllerY;

    public AutoPreShootCommand(IndicatorSubsystem indicatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, double ControllerX, double ControllerY) {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.ControllerX = ControllerX;
        this.ControllerY = ControllerY;
    }

    @Override
    public void initialize() {
        timer.start();
//        indicatorSubsystem.setPattern(IndicatorIO.Patterns.PRE_SHOOT);
    }

    @Override
    public void execute() {
        safeToRaise = DestinationSupplier.isSafeToRaise(Swerve.getInstance().getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp()), DestinationSupplier.getInstance().getCurrentBranch(), ControllerX, ControllerY);
        if (safeToRaise) {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME);
            elevatorSubsystem.setElevatorPosition(DestinationSupplier.getInstance().getElevatorSetpoint(true));
            endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.PRE_SHOOT);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.1) &&
                elevatorSubsystem.elevatorReady(0.005) &&
                //endEffectorSubsystem.isShootReady() &&
                safeToRaise;
    }
}