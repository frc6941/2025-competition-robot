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
import static frc.robot.RobotConstants.ElevatorConstants.HOLD_EXTENSION_METERS;
import static frc.robot.RobotContainer.elevatorIsDanger;

public class ElevatorSubsystem extends SubsystemBase {
    @Getter
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
    public double currentFilterValue = 0.0;
    public boolean hasReachedNearZero = false;
    private WantedState wantedState = WantedState.POSITION;
    @Getter
    private SystemState systemState = SystemState.POSITION_GOING;
    @Getter
    private double wantedPosition = HOLD_EXTENSION_METERS.get();

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("ElevatorPosition", inputs.positionMeters);
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

        SuperstructureVisualizer.getInstance().updateElevator(inputs.positionMeters);


        // set movements based on state
        switch (systemState) {
            case POSITION_GOING:
                if (wantedPosition < (ELEVATOR_MIN_SAFE_HEIGHT.get()) && RobotContainer.endeffectorIsDanger && !RobotContainer.overrideEndEffectorDanger) {
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
                io.setElevatorTarget(HOLD_EXTENSION_METERS.get());
                break;
            default:
                throw new IllegalArgumentException("Unknown systemState: " + systemState);
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case ZERO -> SystemState.ZEROING;
            case POSITION -> SystemState.POSITION_GOING;
            case IDLE -> SystemState.IDLING;
        };
    }

    public void setElevatorState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public double getElevatorPosition() {
        return inputs.positionMeters;
    }

    public void setElevatorPosition(double position) {
        wantedPosition = position;
        setElevatorState(WantedState.POSITION);
    }

    public boolean elevatorReady(double offset) {
        boolean elevatorReady = Math.abs(inputs.positionMeters - wantedPosition) < offset;
        Logger.recordOutput("Elevator/elevatorReady", elevatorReady);
        return elevatorReady;
    }

    public void zeroElevator() {
        Logger.recordOutput("Elevator/hasReachedNearZero", hasReachedNearZero);
        if (!io.isNearZeroExtension() && !hasReachedNearZero) {
            if (RobotContainer.endeffectorIsDanger) {
                // safer
                io.setElevatorTarget(ELEVATOR_MIN_SAFE_HEIGHT.get() + 0.13);
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
        } else {
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