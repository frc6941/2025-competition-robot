package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.commands.ElevatorZeroingCommand;

import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private WantedState wantedState = WantedState.BOTTOM;
    private SystemState systemState = SystemState.AT_BOTTOM;

    private double l1ExtensionMeters = L1_EXTENSION_METERS.get();
    private double l2ExtensionMeters = L2_EXTENSION_METERS.get();
    private double l3ExtensionMeters = L3_EXTENSION_METERS.get();
    private double l4ExtensionMeters = L4_EXTENSION_METERS.get();
    private double groundIntakeExtensionMeters = GROUND_INTAKE_EXTENSION_METERS.get();
    private double funnelIntakeMeters = FUNNEL_INTAKE_EXTENSION_METERS.get();
    private double Setted_Voltage = ELEVATOR_VOLTAGE.get();
    

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
        if(RobotConstants.TUNING){
            l1ExtensionMeters = L1_EXTENSION_METERS.get();
            l2ExtensionMeters = L2_EXTENSION_METERS.get();
            l3ExtensionMeters = L3_EXTENSION_METERS.get();
            l4ExtensionMeters = L4_EXTENSION_METERS.get();
            groundIntakeExtensionMeters = GROUND_INTAKE_EXTENSION_METERS.get();
            funnelIntakeMeters = FUNNEL_INTAKE_EXTENSION_METERS.get();
            Setted_Voltage = ELEVATOR_VOLTAGE.get();
        }


        switch (systemState) {
            case AT_BOTTOM:
                io.setPosition(0.0);
                break;
            case GROUND_INTAKING:
                io.setPosition(groundIntakeExtensionMeters);
                break;
            case FUNNEL_INTAKING:
                io.setPosition(funnelIntakeMeters);
                break;
            case AT_L1:
                io.setPosition(l1ExtensionMeters);
                break;
            case AT_L2:
                io.setPosition(l2ExtensionMeters);
                break;
            case AT_L3:
                io.setPosition(l3ExtensionMeters);
                break;
            case AT_L4:
                io.setPosition(l4ExtensionMeters);
                break;
            case ZEROING:
                    io.setVoltage(-1);
                    break;
            case SETTING_VOLTAGE:
                io.setVoltage(Setted_Voltage);
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
            case ZERO -> {
                if (io.getLeaderCurrent() > RobotConstants.ElevatorConstants.ELEVATOR_ZEROING_CURRENT.get()){
                    wantedState = WantedState.BOTTOM;
                    yield SystemState.AT_BOTTOM;
                }
                yield SystemState.ZEROING;
            }
            case SET_VOLTAGE -> SystemState.SETTING_VOLTAGE;
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
        ZERO,
        SET_VOLTAGE
    }
    public enum SystemState{
        AT_BOTTOM,
        GROUND_INTAKING,
        FUNNEL_INTAKING,
        AT_L1,
        AT_L2,
        AT_L3,
        AT_L4,
        ZEROING,
        SETTING_VOLTAGE
    }
}
