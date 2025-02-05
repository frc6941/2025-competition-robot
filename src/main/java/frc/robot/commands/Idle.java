package frc.robot.commands;

import java.security.PublicKey;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.RobotConstants.intakeConstants.intakePivotGainsClass;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.statemachine.StateMachine;

public class Idle extends Command{
    StateMachine stateMachine;
    ElevatorSubsystem elevatorSubsystem;
    public Idle(StateMachine stateMachine,
                ElevatorSubsystem elevatorSubsystem){
        this.stateMachine = stateMachine;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(stateMachine);
    }
    @Override
    public void initialize() {
        if (stateMachine == null || elevatorSubsystem == null) {
            throw new IllegalStateException("State machine or subsystems are not initialized");
        }
        stateMachine.setRobotState(StateMachine.RobotState.IDLE); // Ensure state is updated
        elevatorSubsystem.setPosition(ElevatorConstants.IDLE_EXTENSION_METERS.get());
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println(stateMachine.getRobotState().toString());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
