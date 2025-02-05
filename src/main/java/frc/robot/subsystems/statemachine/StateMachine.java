package frc.robot.subsystems.statemachine;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.commands.Idle;
import frc.robot.commands.PlaceCoral;
import frc.robot.commands.PrepCoraLv;
import frc.robot.commands.PrepCoralZero;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.swerve.Swerve;

@Logged
public class StateMachine extends SubsystemBase{
    public static RobotState currentRobotState; // Initialize here
    @NotLogged
    ElevatorSubsystem elevatorSubsystem;
    @NotLogged
    EndEffectorSubsystem endEffectorSubsystem;
    @NotLogged
    IntakePivotSubsystem intakePivotSubsystem;
    @NotLogged
    Swerve swerve;

    public StateMachine(ElevatorSubsystem elevatorSubsystem,
                        EndEffectorSubsystem endEffectorSubsystem,
                        IntakePivotSubsystem intakePivotSubsystem,
                        Swerve swerve){
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.intakePivotSubsystem = intakePivotSubsystem;
        this.swerve = swerve; 
    }
    public void setRobotState(RobotState robotState) {
        currentRobotState = robotState;
    }
    
    
    public RobotState getRobotState() {
        return currentRobotState;
    }

    public Command tryState(RobotState desiredState){
        if (elevatorSubsystem == null) {
            System.out.println("Elevator subsystem is null!");
            return Commands.print("Elevator subsystem is not available.");
        }
        switch (desiredState) {
            case IDLE:
                return new Idle(this, 
                elevatorSubsystem);
            case PREP_CORAL_L1:
                return new PrepCoraLv(this, 
                                      elevatorSubsystem, 
                                      RobotConstants.ElevatorConstants.L1_EXTENSION_METERS.get()
                );
            case PREP_CORAL_L2:
                return new PrepCoraLv(this, 
                                      elevatorSubsystem, 
                                      RobotConstants.ElevatorConstants.L2_EXTENSION_METERS.get()
                );
            case PREP_CORAL_L3:
                return new PrepCoraLv(this, 
                                      elevatorSubsystem, 
                                      RobotConstants.ElevatorConstants.L3_EXTENSION_METERS.get()
                );
            case PREP_CORAL_L4:
                return new PrepCoraLv(this, 
                                      elevatorSubsystem, 
                                      RobotConstants.ElevatorConstants.L4_EXTENSION_METERS.get()
                );
            case PREP_CORAL_ZERO:
                return new PrepCoralZero(this, elevatorSubsystem);
            case SCORING_CORAL:
                return new PlaceCoral(this);
            default:
                System.out.println("This Case is undefined");
                break;
        }
        return Commands.print("ITS SO OVER D: Invalid State Provided, Blame Eli");
            
    }    

    public static enum RobotState {
        IDLE,
        HAS_CORAL,
        PREP_CORAL_L1,
        PREP_CORAL_L2,
        PREP_CORAL_L3,
        PREP_CORAL_L4,
        PREP_CORAL_ZERO,
        SCORING_CORAL,
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
