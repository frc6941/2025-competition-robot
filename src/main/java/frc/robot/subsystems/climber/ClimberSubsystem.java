package frc.robot.subsystems.climber;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.utils.TunableNumber;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private LinearFilter currentFilter = LinearFilter.movingAverage(5);
    public double currentFilterValue = 0.0;

    private WantedState wantedState = WantedState.DEPLOY;
    private SystemState systemState = SystemState.DEPLOYING;

    private TunableNumber deployAngle = new TunableNumber("CLIMBER/deployAngle", 0);
    private TunableNumber climbAngle = new TunableNumber("CLIMBER/climbAngle", -420);

    public ClimberSubsystem(ClimberIOReal io){
        this.io = io;
    }

    private static TunableNumber zeroingCurrent = new TunableNumber("Climber/zeroCurrent", 40);

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber/inputs", inputs);

        Logger.recordOutput("Climber/CurrentFilter", currentFilterValue);

        SystemState newState = handleStateTransition();

        Logger.recordOutput("Climber/SystemState", newState.toString());

        currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps);

        if (newState != systemState) {
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.DEPLOYING;
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
            default -> SystemState.DEPLOYING;
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

    public boolean isClimbFinished() {
        return (Math.abs(currentFilterValue) > zeroingCurrent.get());
    }

    public void resetPosition() {
        io.resetPosition();
    }
}
