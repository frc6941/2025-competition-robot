package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    // private EndEffectorSubsystem endEffector;
    private ClimberSubsystem climber;

    public enum WantedSuperState {
        STOPPED,
        INTAKE_CORAL_FUNNEL,
        INTAKE_CORAL_GROUND,
        SHOOT_CORAL,
        CLIMB,
        GO_DOWN,
        HOLD,
    }

    public enum CurrentSuperState {
        STOPPED,
        INTAKE_CORAL_FUNNEL,
        INTAKE_CORAL_GROUND,
        SHOOT_CORAL,
        CLIMB,
        GO_DOWN,
        HOLD,
    }

    WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
    CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
    CurrentSuperState previousSuperState;

    public Superstructure(ClimberSubsystem climber){
            // EndEffectorSubsystem endEffector) {
        // this.endEffector = endEffector;
        this.climber = climber;
    }

    @Override
    public void periodic() {
        currentSuperState = handleStateTransition();
        applyStates();

        Logger.recordOutput("DesiredSuperstate", wantedSuperState);
        Logger.recordOutput("CurrentSuperstate", currentSuperState);
    }

    private CurrentSuperState handleStateTransition() {
        previousSuperState = currentSuperState;
        switch(wantedSuperState) {
            // case INTAKE_CORAL_FUNNEL:
            //     currentSuperState = CurrentSuperState.INTAKE_CORAL_FUNNEL;
            //     break;
            // case INTAKE_CORAL_GROUND:
            //     currentSuperState = CurrentSuperState.INTAKE_CORAL_GROUND;
            //     break;
            // case SHOOT_CORAL:
            //     currentSuperState = CurrentSuperState.SHOOT_CORAL;
            //     break;
            case CLIMB:
                currentSuperState = CurrentSuperState.CLIMB;
                break;  
            case GO_DOWN:
                currentSuperState = CurrentSuperState.GO_DOWN;
                break;
            case HOLD:
                currentSuperState = CurrentSuperState.HOLD;
                break;
            case STOPPED:
            default:
                currentSuperState = CurrentSuperState.STOPPED;
                break;
        }
        return currentSuperState;
    }

    private void applyStates() {
        switch (currentSuperState) {
            // case INTAKE_CORAL_FUNNEL:
            //     intakeCoralFunnel();
            //     break;
            // case INTAKE_CORAL_GROUND:
            //     intakeCoralGround();
            //     break;
            // case SHOOT_CORAL:
            //     shootCoral();
            //     break;
            case CLIMB:
                climber.setWantedState(ClimberSubsystem.WantedState.CLIMB);
                break;
            case GO_DOWN:
                climber.setWantedState(ClimberSubsystem.WantedState.GO_DOWN);
                break;
            case HOLD:
                climber.setWantedState(ClimberSubsystem.WantedState.HOLD);
                break;
            case STOPPED:
            default:
                handleStopped();
                break;
        }
    }

    public void setWantedSuperState(WantedSuperState wantedSuperState) {
        this.wantedSuperState = wantedSuperState;
    }

    public Command setWantedSuperStateCommand(WantedSuperState wantedSuperState) {
        return new InstantCommand(() -> setWantedSuperState(wantedSuperState));
    }

    // private void intakeCoralFunnel() {
    //     if (endEffector.isIntakeFinished()) {
    //         endEffector.setWantedState(EndEffectorSubsystem.WantedState.TRANSFER);
    //     } else {
    //         endEffector.setWantedState(EndEffectorSubsystem.WantedState.FUNNEL_INTAKE);
    //     }
    // }

    // private void intakeCoralGround() {
    //     if (endEffector.isIntakeFinished()) {
    //         endEffector.setWantedState(EndEffectorSubsystem.WantedState.TRANSFER);
    //     } else {
    //         endEffector.setWantedState(EndEffectorSubsystem.WantedState.GROUND_INTAKE);
    //     }
    // }

    // private void shootCoral() {
    //     if (endEffector.isCoralReady()) {endEffector.setWantedState(EndEffectorSubsystem.WantedState.SHOOT);}
    // }

    private void handleStopped() {
        climber.setWantedState(ClimberSubsystem.WantedState.IDLE);
        // endEffector.setWantedState(EndEffectorSubsystem.WantedState.IDLE);
    }

}