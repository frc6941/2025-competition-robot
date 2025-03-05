package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TunableNumber;
import org.littletonrobotics.junction.Logger;

//TODO: change motion logic and reset command afterwards

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final TunableNumber deployAngle = new TunableNumber("CLIMBER/deployAngle", 900);
    private final TunableNumber idleAngle = new TunableNumber("CLIMBER/idleAngle", 600);
    private final TunableNumber climbAngle = new TunableNumber("CLIMBER/climbAngle", -370);
    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        SystemState newState = handleStateTransition();

        Logger.processInputs("Climber", inputs);
        Logger.recordOutput("Climber/SystemState", newState.toString());

        if (newState != systemState) {
            systemState = newState;
        }

        switch (systemState) {
            case IDLING:
                io.setTargetPosition(idleAngle.get());
                break;
            case DEPLOYING:
                io.setTargetPosition(deployAngle.get());
                break;
            case CLIMBING:
                io.setTargetPosition(climbAngle.get());
                break;
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case DEPLOY -> SystemState.DEPLOYING;
            case CLIMB -> SystemState.CLIMBING;
            case IDLE -> SystemState.IDLING;
        };
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void resetPosition() {
        io.resetPosition();
    }

    public void setCoast() {
        io.setCoast();
    }

    public void setBrake() {
        io.setBrake();
    }

    public enum WantedState {
        DEPLOY,
        CLIMB,
        IDLE
    }

    public enum SystemState {
        DEPLOYING,
        CLIMBING,
        IDLING
    }


}
