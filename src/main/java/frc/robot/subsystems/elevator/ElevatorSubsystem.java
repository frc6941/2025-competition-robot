package frc.robot.subsystems.elevator;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.display.SuperstructureVisualizer;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.ElevatorConstants.ELEVATOR_MIN_SAFE_HEIGHT;
import static frc.robot.RobotConstants.ElevatorConstants.IDLE_EXTENSION_METERS;
import static frc.robot.RobotConstants.TUNING;
import static frc.robot.RobotContainer.elevatorIsDanger;

public class ElevatorSubsystem extends SubsystemBase {
    @Getter
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
    public double currentFilterValue = 0.0;
    private WantedState wantedState = WantedState.POSITION;
    private SystemState systemState = SystemState.POSITION_GOING;
    private double wantedPosition = IDLE_EXTENSION_METERS.get();
    private boolean hasReachedNearZero = false;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("ElevatorPosition", io.getElevatorHeight());
        Logger.recordOutput("Elevator/isNear", io.isNearExtension(wantedPosition));
        Logger.recordOutput("Elevator/isNearZero", io.isNearZeroExtension());
        Logger.recordOutput("Elevator/setPoint", wantedPosition);
        Logger.recordOutput("Elevator/WantedState", wantedState.toString());

        currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps);
        Logger.recordOutput("Elevator/CurrentFilter", currentFilterValue);
        // process inputs
        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            // Logger.recordOutput("Elevator/SystemState", newState.toString());
            systemState = newState;
        }
        Logger.recordOutput("Elevator/SystemState", systemState.toString());

        elevatorIsDanger = elevatorIsDanger();

        Logger.recordOutput("Flags/elevatorIsDanger", elevatorIsDanger());

        SuperstructureVisualizer.getInstance().updateElevator(io.getElevatorHeight());




        // set movements based on state
        switch (systemState) {
            case POSITION_GOING:
                //worked, but need clean up
                if (wantedPosition < ELEVATOR_MIN_SAFE_HEIGHT.get() && RobotContainer.intakeIsAvoiding && RobotContainer.intakeIsDanger) {
                    io.setElevatorTarget(Math.max(wantedPosition,0.4));
                } else if (wantedPosition < RobotConstants.ElevatorConstants.ELEVATOR_MIN_SAFE_HEIGHT.get() && RobotContainer.intakeIsDanger){
                    io.setElevatorTarget(ELEVATOR_MIN_SAFE_HEIGHT.get());
                } else {
                    io.setElevatorTarget(wantedPosition);
                }
                break;
            case ZEROING:
                zeroElevator();
                break;
            //TODO verify utility, may delete
            case IDLING:
                io.setElevatorVoltage(0);
                io.setElevatorTarget(IDLE_EXTENSION_METERS.get());
                break;
            default:
                throw new IllegalArgumentException("Unknown systemState: " + systemState);
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case ZERO -> SystemState.ZEROING;
            case POSITION -> SystemState.POSITION_GOING;
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
            if (RobotContainer.intakeIsDanger) {
                io.setElevatorTarget(ELEVATOR_MIN_SAFE_HEIGHT.get());
            } else {
                io.setElevatorTarget(0.05);
            }
            return;
        }
        hasReachedNearZero = true;
        if (RobotBase.isReal()) {
            if (currentFilterValue <= RobotConstants.ElevatorConstants.ELEVATOR_ZEROING_CURRENT.get()) {
                io.setElevatorVoltage(-1);
                wantedState = WantedState.ZERO;
            }
            if (currentFilterValue > RobotConstants.ElevatorConstants.ELEVATOR_ZEROING_CURRENT.get()) {
                io.setElevatorVoltage(0);
                io.resetElevatorPosition();
                wantedState = WantedState.IDLE;
                hasReachedNearZero = false;
            }
        }else{
            io.setElevatorTarget(0);
            wantedState = WantedState.IDLE;
            hasReachedNearZero = false;
        }
    }

    public boolean elevatorIsDanger() {
        return (inputs.positionMeters < ELEVATOR_MIN_SAFE_HEIGHT.get() - 0.01);
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
