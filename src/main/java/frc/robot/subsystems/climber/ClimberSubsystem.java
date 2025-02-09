package frc.robot.subsystems.climber;

import static frc.robot.RobotConstants.ClimberConstants.MAX_ANGLE;
import static frc.robot.RobotConstants.ClimberConstants.MIN_ANGLE;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class ClimberSubsystem extends SubsystemBase{
    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public ClimberSubsystem(ClimberIOReal io){
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber/inputs", inputs);

        SystemState newState = handleStateTransition();

        Logger.recordOutput("Climber/SystemState", newState.toString());

        if (newState != systemState) {
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.IDLING;
        }

        switch (systemState) {
            case IDLING:
                io.setMotorPosition(MIN_ANGLE);
                break;
            case HOLDING:
                io.setMotorPosition(MAX_ANGLE);
                break;
            case CLIMBING:
                io.setMotorPosition(MAX_ANGLE);
                break;
            case GOING_DOWN:
                io.setMotorPosition(MIN_ANGLE);
                break;
            default:
                io.setMotorPosition(MIN_ANGLE);
                break;
        }
        }

    public Command setTargetAngle(Rotation2d target) {
        return setTargetAngle(() -> target);
    }

    public Command setTargetAngle(Supplier<Rotation2d> target) {
        return this.runOnce(() -> Logger.recordOutput("Climber/ClimberSetPoint", target.get()))
                .andThen(this.run(() -> io.setMotorPosition(target.get())));
    }

    private SystemState handleStateTransition(){
        return switch(wantedState){
            case IDLE -> SystemState.IDLING;
            case HOLD -> SystemState.HOLDING;
            case CLIMB -> SystemState.CLIMBING;
            case GO_DOWN -> SystemState.GOING_DOWN;
            default -> SystemState.IDLING;
        };
    }

    public void setWantedState(WantedState wantedState) {this.wantedState = wantedState;}

    public enum WantedState{
        IDLE,
        HOLD,
        CLIMB,
        GO_DOWN
    }
    public enum SystemState{
        IDLING,
        HOLDING,CLIMBING,
        GOING_DOWN
    }

}
