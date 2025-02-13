package frc.robot.subsystems.elevator;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

@Getter
public class ElevatorSubsystem extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
    public double currentFilterValue = 0.0;
    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;
    private double wantedPosition = 0.0;
    private boolean hasReachedNearZero = false;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("ElevatorPosition", io.getElevatorPosition());
        Logger.recordOutput("Elevator/isNear", io.isNearExtension(wantedPosition));
        Logger.recordOutput("Elevator/isNearZero", io.isNearZeroExtension());
        Logger.recordOutput("Elevator/setPoint", wantedPosition);
        Logger.recordOutput("Elevator/WantedState", wantedState.toString());

        currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps[0]);
        Logger.recordOutput("Elevator/CurrentFilter", currentFilterValue);
        // process inputs
        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            Logger.recordOutput("Elevator/SystemState", newState.toString());
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            wantedState = WantedState.IDLE;
        }

        // set movements based on state
        switch (systemState) {
            case POSITION_GOING:
                io.setElevatorTarget(wantedPosition);
                break;
            case ZEROING:
                zeroElevator();
                break;
            case IDLING:
                io.setElevatorDirectVoltage(0);
                break;
            default:
                throw new IllegalArgumentException("Unknown systemState: " + systemState);
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case ZERO -> SystemState.ZEROING;
            case POSITION -> {
                if (systemState == SystemState.ZEROING) {
                    wantedState = WantedState.ZERO;
                    yield SystemState.ZEROING;
                }
                yield SystemState.POSITION_GOING;
            }
            case IDLE -> {
                if (systemState == SystemState.POSITION_GOING) {
                    wantedState = WantedState.ZERO;
                    yield SystemState.POSITION_GOING;
                }
                yield SystemState.IDLING;
            }
        };
    }

    public void setElevatorState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setElevatorPosition(double position) {
        wantedPosition = position;
        setElevatorState(WantedState.POSITION);
    }

    public void zeroElevator() {
        if (!io.isNearZeroExtension() && !hasReachedNearZero) {
            io.setElevatorTarget(0.05);
            return;
        }
        hasReachedNearZero = true;
        if (currentFilterValue <= RobotConstants.ElevatorConstants.ELEVATOR_ZEROING_CURRENT.get()) {
            io.setElevatorDirectVoltage(-1);
            wantedState = WantedState.ZERO;
        }
        if (currentFilterValue > RobotConstants.ElevatorConstants.ELEVATOR_ZEROING_CURRENT.get()) {
            io.setElevatorDirectVoltage(0);
            io.resetElevatorPosition();
            wantedState = WantedState.IDLE;
            hasReachedNearZero = false;
        }
    }

    public enum WantedState {
        POSITION,
        ZERO,
        IDLE
    }


    public enum SystemState {
        POSITION_GOING,
        ZEROING,
        IDLING
    }
}