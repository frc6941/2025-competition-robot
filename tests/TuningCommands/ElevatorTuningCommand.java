package frc.robot.commands.TuningCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import static edu.wpi.first.units.Units.Volts;

/*
Used for tuning the elevator constants.
Bind to a key whileTrue, prints out the absolute starting position on press, prints out how much position is changed on release
 */

public class ElevatorTuningCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final boolean isMovingUp;
    private double initialPosition;

    public ElevatorTuningCommand(ElevatorSubsystem elevatorSubsystem, boolean isMovingUp) {
        this.elevatorSubsystem = elevatorSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.elevatorSubsystem);
        this.isMovingUp = isMovingUp;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        this.initialPosition = this.elevatorSubsystem.getIo().getElevatorPosition();
        System.out.println(this.initialPosition);
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        this.elevatorSubsystem.getIo().setElevatorVelocity(-RobotConstants.ElevatorConstants.elevatorMotorRPS * 15 * (this.isMovingUp ? 1 : -1));
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        this.elevatorSubsystem.getIo().setElevatorDirectVoltage(Volts.of(0));
        System.out.println(this.elevatorSubsystem.getIo().getElevatorPosition() - this.initialPosition);
    }
}
