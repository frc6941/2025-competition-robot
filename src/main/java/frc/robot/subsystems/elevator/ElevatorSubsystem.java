package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private WantedState wantedState = WantedState.BOTTOM;
    private SystemState systemState = SystemState.AT_BOTTOM;


    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        
        SystemState newState = handleStateTransition();

        Logger.recordOutput("Elevator/SystemState", newState.toString());

        if (newState != systemState) {
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.AT_BOTTOM;
        }

        switch (systemState) {
            case AT_BOTTOM:
                io.setPosition(0.0);
                break;
            case GROUND_INTAKING:
                io.setPosition(GROUND_INTAKE_EXTENSION_METERS.get());
                break;
            case FUNNEL_INTAKING:
                io.setPosition(FUNNEL_INTAKE_EXTENSION_METERS.get());
                break;
            case AT_L1:
                io.setPosition(L1_EXTENSION_METERS.get());
                break;
            case AT_L2:
                io.setPosition(L2_EXTENSION_METERS.get());
                break;
            case AT_L3:
                io.setPosition(L3_EXTENSION_METERS.get());
                break;
            case AT_L4:
                io.setPosition(L4_EXTENSION_METERS.get());
                break;
            case ZEROING:
                break;
            default:
                break;
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case BOTTOM -> SystemState.AT_BOTTOM;
            case GROUND_INTAKE -> SystemState.GROUND_INTAKING;
            case FUNNEL_INTAKE -> SystemState.FUNNEL_INTAKING;
            case L1 -> SystemState.AT_L1;
            case L2 -> SystemState.AT_L2;
            case L3 -> SystemState.AT_L3;
            case L4 -> SystemState.AT_L4;
            case ZERO -> SystemState.ZEROING;
            default -> SystemState.AT_BOTTOM;
        };
    }

    public void setPosition(double heightMeters) {
        io.setPosition(heightMeters);
        Logger.recordOutput("Elevator/Setpoint", heightMeters);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public double getPositionMeters() {
        return inputs.positionMeters;
    }

    public boolean isAtSetpoint(double setpoint) {
        return MathUtil.isNear(setpoint, inputs.positionMeters, DEADZONE_DISTANCE);
    }

    public double getLeaderCurrent(){
        return io.getLeaderCurrent();
    }

    public void resetPosition(){
        io.resetPosition();
    }

    public void setWantedState(WantedState wantedState) {this.wantedState = wantedState;}

    public enum WantedState{
        BOTTOM,
        GROUND_INTAKE,
        FUNNEL_INTAKE,
        L1,
        L2,
        L3,
        L4,
        ZERO
    }
    public enum SystemState{
        AT_BOTTOM,
        GROUND_INTAKING,
        FUNNEL_INTAKING,
        AT_L1,
        AT_L2,
        AT_L3,
        AT_L4,
        ZEROING
    }
}
