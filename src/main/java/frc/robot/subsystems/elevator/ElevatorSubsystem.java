package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

@Getter
public class ElevatorSubsystem extends SubsystemBase {

    public enum WantedState {
        BOTTOM,
        INTAKER_INTAKE,
        INTAKER_AVOID,
        FUNNEL_INTAKE,
        L1,
        L2,
        L3,
        L4,
        ZEROING
    }

    public enum SystemState {
        BOTTOM_STAYING,
        INTAKER_INTAKING,
        INTAKER_AVOIDING,
        FUNNEL_INTAKING,
        L1_STAYING,
        L2_STAYING,
        L3_STAYING,
        L4_STAYING,
        ZEROING
    }

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private WantedState wantedState = WantedState.BOTTOM;
    private SystemState systemState = SystemState.BOTTOM_STAYING;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // process inputs
        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            Logger.recordOutput("Elevator/SystemState", newState.toString());
            systemState = newState;
        }

        // set movements based on state
        switch (systemState) {
            case BOTTOM_STAYING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.BOTTOM_HEIGHT);
                break;
            case INTAKER_INTAKING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.INTAKE_INTAKING_HEIGHT);
                break;
            case INTAKER_AVOIDING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.INTAKE_AVOIDING_HEIGHT);
                break;
            case FUNNEL_INTAKING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.FUNNEL_INTAKING_HEIGHT);
                break;
            case L1_STAYING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.L1_HEIGHT);
                break;
            case L2_STAYING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.L2_HEIGHT);
                break;
            case L3_STAYING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.L3_HEIGHT);
                break;
            case L4_STAYING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.L4_HEIGHT);
                break;
            case ZEROING:
                io.zeroingElevator();
                break;
        }
    }


    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case BOTTOM -> {
                if (systemState == SystemState.INTAKER_INTAKING) {
                    if (true/*is intaker motor off*/){
                        yield SystemState.BOTTOM_STAYING;
                    }
                    yield SystemState.INTAKER_INTAKING;
                }
                if (systemState == SystemState.FUNNEL_INTAKING){
                    if (true/*is funnel motor off*/){
                        yield SystemState.BOTTOM_STAYING;
                    }
                    yield SystemState.FUNNEL_INTAKING;
                }
                if (systemState == SystemState.L1_STAYING || systemState == SystemState.L2_STAYING ||
                        systemState == SystemState.L3_STAYING || systemState == SystemState.L4_STAYING) {

                }
                if (systemState == SystemState.ZEROING) {
                    yield SystemState.ZEROING;
                }
            }
            case TRIGGER -> {
                if(!inputs.higherbeamBreakState) {
                    yield  SystemState.IDLING;
                }
                yield SystemState.TRIGGERING;
            }
            case OUTTAKE -> SystemState.OUTTAKING;
            case COLLECT -> {
                if (inputs.higherbeamBreakState) {
                    yield SystemState.IDLING;
                }
                if (inputs.lowerBeamBreakState) {
                    //Decide if note has entered intaker
                    yield SystemState.FEEDING;
                }
                yield SystemState.COLLECTING;
            }
            case FEED -> {
                if (inputs.higherbeamBreakState) {
                    yield SystemState.IDLING;
                }
                yield SystemState.FEEDING;
            }
            default -> SystemState.IDLING;
        };
    }


}
