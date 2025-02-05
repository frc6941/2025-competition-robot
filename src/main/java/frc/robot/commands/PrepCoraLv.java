package frc.robot.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.statemachine.StateMachine;

public class PrepCoraLv extends Command{
    double height;
    StateMachine stateMachine;
    ElevatorSubsystem elevatorSubsystem;
    public PrepCoraLv(StateMachine stateMachine,ElevatorSubsystem elevatorSubsystem,double height){
        this.elevatorSubsystem = elevatorSubsystem;
        this.stateMachine = stateMachine;
        this.height = height;
        addRequirements(elevatorSubsystem);
        addRequirements(stateMachine);
    }
    @Override
    public void initialize(){
        if (stateMachine == null || elevatorSubsystem == null) {
            throw new IllegalStateException("State machine or subsystems are not initialized");
        }
        else if (height == RobotConstants.ElevatorConstants.L1_EXTENSION_METERS.get()) {
            stateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L1);
        }
        else if (height == RobotConstants.ElevatorConstants.L2_EXTENSION_METERS.get()) {
            stateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L2);
        }
        else if (height == RobotConstants.ElevatorConstants.L3_EXTENSION_METERS.get()) {
            stateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L3);
        }
        else if (height == RobotConstants.ElevatorConstants.L4_EXTENSION_METERS.get()) {
            stateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L4);
        }
        
    }
    @Override
    public void execute(){
        elevatorSubsystem.setPosition(height);
        System.out.println("executed");
        
    }
    @Override
    public boolean isFinished(){
        return elevatorSubsystem.isAtSetpoint(height);
    }
}
