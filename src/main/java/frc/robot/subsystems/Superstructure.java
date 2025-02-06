package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    private EndEffectorSubsystem endEffector;

    public enum WantedSuperState {
        STOPPED,
        INTAKE_CORAL_FUNNEL,
        SHOOT_CORAL
    }

    public enum CurrentSuperState {
        STOPPED,
        INTAKE_CORAL_FUNNEL,
        SHOOT_CORAL
    }

    WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
    CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
    CurrentSuperState previousSuperState;

    public Superstructure(
            EndEffectorSubsystem endEffector) {
        this.endEffector = endEffector;
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
            case INTAKE_CORAL_FUNNEL:
                currentSuperState = CurrentSuperState.INTAKE_CORAL_FUNNEL;
                break;
            case SHOOT_CORAL:
                currentSuperState = CurrentSuperState.SHOOT_CORAL;
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
            case INTAKE_CORAL_FUNNEL:
                intakeCoralFunnel();
                break;
            case SHOOT_CORAL:
                shootCoral();
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

    private void intakeCoralFunnel() {
        if (endEffector.isFunnelIntakeFinished()) {
            endEffector.setWantedState(EndEffectorSubsystem.WantedState.FUNNEL_TRANSFER);
        } else {
            endEffector.setWantedState(EndEffectorSubsystem.WantedState.FUNNEL_INTAKE);
        }
    }

    private void shootCoral() {
        endEffector.setWantedState(EndEffectorSubsystem.WantedState.SHOOT);
    }

    private void handleStopped() {
        endEffector.setWantedState(EndEffectorSubsystem.WantedState.IDLE);
    }

}