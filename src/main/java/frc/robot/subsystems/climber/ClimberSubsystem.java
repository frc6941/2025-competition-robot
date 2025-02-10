package frc.robot.subsystems.climber;

import frc.robot.utils.TunableNumber;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private WantedState wantedState = WantedState.DEPLOY;
    private SystemState systemState = SystemState.DEPLOYING;

    private TunableNumber deployAngle = new TunableNumber("CLIMBER/deployAngle", 0);
    private TunableNumber climbAngle = new TunableNumber("CLIMBER/climbAngle", -420);

    public ClimberSubsystem(ClimberIOReal io){
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        SystemState newState = handleStateTransition();

        Logger.processInputs("Climber/inputs", inputs);
        Logger.recordOutput("Climber/SystemState", newState.toString());

        if (newState != systemState) {
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.DEPLOYING;
            io.resetPosition();
        }

        switch (systemState) {
            case DEPLOYING:
                io.setTargetPosition(deployAngle.get());
                break;
            case CLIMBING:
                io.setTargetPosition(climbAngle.get());
                break;
        }
    }

    private SystemState handleStateTransition(){
        return switch(wantedState){
            case DEPLOY -> SystemState.DEPLOYING;
            case CLIMB -> SystemState.CLIMBING;
        };
    }

    public void setWantedState(WantedState wantedState) {this.wantedState = wantedState;}

    public enum WantedState{
        DEPLOY,
        CLIMB
    }
    public enum SystemState{
        DEPLOYING,
        CLIMBING
    }

    public void resetPosition() {
        io.resetPosition();
    }
}
