package frc.robot.commands;

import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.EndEffectorConstants;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.statemachine.StateMachine;

public class PlaceCoral extends Command{
    StateMachine stateMachine;
    // EndEffectorSubsystem endEffectorSubsystem;

    public PlaceCoral(StateMachine stateMachine
                    //   EndEffectorSubsystem endEffectorSubsystem
                    ){
        this.stateMachine = stateMachine;
        // this.endEffectorSubsystem = endEffectorSubsystem;
        addRequirements(stateMachine);
    }
    @Override
    public void initialize() {
        stateMachine.setRobotState(StateMachine.RobotState.SCORING_CORAL);
        // endEffectorSubsystem.setVelocity(1000);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    //     endEffectorSubsystem.setVelocity(1000);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // endEffectorSubsystem.setVelocity(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}