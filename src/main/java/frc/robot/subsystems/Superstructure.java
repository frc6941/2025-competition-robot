package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
// import frc.robot.subsystems.intake.IntakerSubsystem;

import java.util.logging.Handler;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    private EndEffectorSubsystem endEffector;
    // private IntakerSubsystem intakerSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ClimberSubsystem climberSubsystem;

    public enum WantedSuperState {
        STOPPED,
        INTAKE_CORAL_FUNNEL,
        GROUND_INTAKE,
        SHOOT_CORAL,
        OUTTAKE,
        L1,
        L2,
        L3,
        L4,
        CLIMB,
        HOLD_POSITION,
        GO_DOWN
    }

    public enum CurrentSuperState {
        STOPPED,
        INTAKE_CORAL_FUNNEL,
        GROUND_INTAKE,
        SHOOT_CORAL,
        OUTTAKE,
        L1,
        L2,
        L3,
        L4,
        CLIMB,
        HOLD_POSITION,
        GO_DOWN
    }

    WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
    WantedSuperState previosWantedSuperState;
    CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
    CurrentSuperState previousSuperState;

    public Superstructure(
            EndEffectorSubsystem endEffector, /*IntakerSubsystem intakerSubsystem, */ElevatorSubsystem elevatorSubsystem,ClimberSubsystem climberSubsystem) {
        this.endEffector = endEffector;
        // this.intakerSubsystem = intakerSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.climberSubsystem = climberSubsystem;
    }

    @Override
    public void periodic() {
        currentSuperState = handleStateTransition();
        applyStates();

        Logger.recordOutput("DesiredSuperstate", wantedSuperState);
        Logger.recordOutput("CurrentSuperstate", currentSuperState);
        Logger.recordOutput("previosWantedSuperState",previosWantedSuperState);

        if (DriverStation.isDisabled()) {
            wantedSuperState = WantedSuperState.STOPPED;
        }
    }

    private CurrentSuperState handleStateTransition() {
        previousSuperState = currentSuperState;
        switch(wantedSuperState) {
            case INTAKE_CORAL_FUNNEL:
                currentSuperState = CurrentSuperState.INTAKE_CORAL_FUNNEL;
                break;
            case SHOOT_CORAL:
                if(endEffector.isEndEffectorIntaking()){
                    wantedSuperState = previosWantedSuperState;
                    break;
                }
                currentSuperState = CurrentSuperState.SHOOT_CORAL;
                break;
            case GROUND_INTAKE:
                currentSuperState = CurrentSuperState.GROUND_INTAKE;
                break;
            case OUTTAKE:
                currentSuperState = CurrentSuperState.OUTTAKE;
                break;
            case L1:
                currentSuperState = CurrentSuperState.L1;
                break;
            case L2:
                currentSuperState = CurrentSuperState.L2;
                break;
            case L3:
                currentSuperState = CurrentSuperState.L3;
                break;
            case L4:
                currentSuperState = CurrentSuperState.L4;
                break;
            case CLIMB:
                currentSuperState = CurrentSuperState.CLIMB;
                break;
            case HOLD_POSITION:
                currentSuperState = CurrentSuperState.HOLD_POSITION;
                break;
            case GO_DOWN:
                currentSuperState = CurrentSuperState.GO_DOWN;
                break;
            case STOPPED:
                currentSuperState = CurrentSuperState.STOPPED;
                break;
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
            case GROUND_INTAKE:
                groundIntake();
                break;
            case OUTTAKE:
                outtake();
                break;
            case L1:
                l1();
                break;
            case L2:
                l2();
                break;
            case L3:
                l3();
                break;
            case L4:
                l4();
                break;
            case CLIMB:
                climb();
                break;
            case HOLD_POSITION:
                holdPosition();
                break;
            case GO_DOWN:
                goDown();
                break;
            case STOPPED:
                handleStopped();
                break;
            default:
                handleStopped();
                break;
        }
    }

    public void setWantedSuperState(WantedSuperState wantedSuperState) {
        previosWantedSuperState = this.wantedSuperState;
        this.wantedSuperState = wantedSuperState;
    }

    public Command setWantedSuperStateCommand(WantedSuperState wantedSuperState) {
        return new InstantCommand(() -> setWantedSuperState(wantedSuperState));
    }

    private void intakeCoralFunnel() {
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.FUNNEL_INTAKE);
        if (endEffector.isFunnelIntakeFinished()) {
            endEffector.setWantedState(EndEffectorSubsystem.WantedState.FUNNEL_TRANSFER);
        } else {
            endEffector.setWantedState(EndEffectorSubsystem.WantedState.FUNNEL_INTAKE);
        }
    }

    private void shootCoral() {
        endEffector.setWantedState(EndEffectorSubsystem.WantedState.SHOOT);
    }
    private void l1() {
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.L1);
    }
    private void l2() {
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.L2);
    }
    private void l3() {
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.L3);
    }
    private void l4() {
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.L4);
    }
    private void climb() {
        climberSubsystem.setWantedState(ClimberSubsystem.WantedState.CLIMB);
    }
    private void holdPosition() {
        climberSubsystem.setWantedState(ClimberSubsystem.WantedState.HOLD);
    }
    private void goDown() {
        climberSubsystem.setWantedState(ClimberSubsystem.WantedState.GO_DOWN);
    }

    private void outtake(){
        // intakerSubsystem.setWantedState(IntakerSubsystem.WantedState.OUTTAKE);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.GROUND_INTAKE);
    }

    private void groundIntake() {
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.GROUND_INTAKE);
        // intakerSubsystem.setWantedState(IntakerSubsystem.WantedState.INTAKE);
        if (endEffector.isFunnelIntakeFinished()) {
            endEffector.setWantedState(EndEffectorSubsystem.WantedState.FUNNEL_TRANSFER);
        } else {
            endEffector.setWantedState(EndEffectorSubsystem.WantedState.GROUND_INTAKE);
        }
    }

    private void handleStopped() {
        // intakerSubsystem.setWantedState(IntakerSubsystem.WantedState.IDLE);
        endEffector.setWantedState(EndEffectorSubsystem.WantedState.IDLE);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.BOTTOM);
        climberSubsystem.setWantedState(ClimberSubsystem.WantedState.IDLE);
    }

}