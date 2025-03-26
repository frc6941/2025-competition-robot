package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConstants;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.EndEffectorConstants.EndEffectorGainsClass.*;
import static frc.robot.RobotConstants.EndEffectorConstants.*;

public class EndEffectorSubsystem extends RollerSubsystem {
    public static final String NAME = "EndEffector";
    private static final double idleRPS = IDLE_RPS.get();
    private static double shootCoralRPS = SHOOT_CORAL_RPS.get();
    private static double shootAlgaeRPS = SHOOT_ALGAE_RPS.get();
    private static double intakeCoralRPS = INTAKE_CORAL_RPS.get();
    private static double intakeCoralFinalRPS = INTAKE_CORAL_FINAL_RPS.get();
    private static double intakeAlgaeRPS = INTAKE_ALGAE_RPS.get();
    private static double intakeAlgaeFinalRPS = INTAKE_ALGAE_FINAL_RPS.get();
    private final EndEffectorIO endEffectorIO;
    private final BeambreakIO CoralBBIO, AlgaeBBIO;
    private final BeambreakIOInputsAutoLogged CoralBBInputs = new BeambreakIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged AlgaeBBInputs = new BeambreakIOInputsAutoLogged();
    public double kp = ENDEFFECTOR_KP.get();
    public double ki = ENDEFFECTOR_KI.get();
    public double kd = ENDEFFECTOR_KD.get();
    public double ka = ENDEFFECTOR_KA.get();
    public double kv = ENDEFFECTOR_KV.get();
    public double ks = ENDEFFECTOR_KS.get();
    private WantedState wantedState = WantedState.IDLE;

    @Getter
    private SystemState systemState = SystemState.IDLING;

    public EndEffectorSubsystem(EndEffectorIO endEffectorIO, BeambreakIO CoralBBIO, BeambreakIO AlgaeBBIO) {
        super(endEffectorIO, NAME);
        this.endEffectorIO = endEffectorIO;
        this.CoralBBIO = CoralBBIO;
        this.AlgaeBBIO = AlgaeBBIO;
    }

    @Override
    public void periodic() {
        super.periodic();

        CoralBBIO.updateInputs(CoralBBInputs);
        AlgaeBBIO.updateInputs(AlgaeBBInputs);
        Logger.processInputs(NAME + "/Coral Beambreak", CoralBBInputs);
        Logger.processInputs(NAME + "/Algae Beambreak", AlgaeBBInputs);

        SystemState newState = handleStateTransition();

        Logger.recordOutput("EndEffector/SystemState", newState.toString());
        Logger.recordOutput(NAME + "Velocity", inputs.velocityRotPerSec);

//        Logger.recordOutput(NAME + "/isShootReady", isShootReady());
//        Logger.recordOutput(NAME + "/ShootFinished", isShootFinished());
//        Logger.recordOutput(NAME + "/isIntakeFinished", isIntakeFinished());

        if (newState != systemState) {
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.IDLING;
        }

        switch (systemState) {
            case IDLING:
                io.setVelocity(idleRPS);
                break;
            case CORAL_INTAKING:
                io.setVelocity(intakeCoralRPS);
                break;
            case ALGAE_INTAKING:
                io.setVelocity(intakeAlgaeRPS);
                break;
            case CORAL_HOLD:
                io.setVelocity(intakeCoralFinalRPS);
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                io.setVelocity(0.0);
                break;
            case ALGAE_HOLD:
                io.setVelocity(intakeAlgaeFinalRPS);
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                io.setVelocity(0.0);
                break;
            case CORAL_SHOOTING:
                io.setVelocity(shootCoralRPS);
                break;
            case ALGAE_SHOOTING:
                io.setVelocity(shootAlgaeRPS);
                break;
            case OFF:
        }

        if (RobotConstants.TUNING) {
            shootCoralRPS = SHOOT_CORAL_RPS.get();
            shootAlgaeRPS = SHOOT_ALGAE_RPS.get();
            intakeCoralRPS = INTAKE_CORAL_RPS.get();
            intakeCoralFinalRPS = INTAKE_CORAL_FINAL_RPS.get();
            intakeAlgaeRPS = INTAKE_ALGAE_RPS.get();
            intakeAlgaeFinalRPS = INTAKE_ALGAE_FINAL_RPS.get();

            kp = ENDEFFECTOR_KP.get();
            ki = ENDEFFECTOR_KI.get();
            kd = ENDEFFECTOR_KD.get();
            ka = ENDEFFECTOR_KA.get();
            kv = ENDEFFECTOR_KV.get();
            ks = ENDEFFECTOR_KS.get();

            endEffectorIO.updateConfigs(kp, ki, kd, ka, kv, ks);
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLING;
            case CORAL_INTAKE -> {
                if(hasCoral()){
                    setWantedState(WantedState.CORAL_HOLDING);
                    yield SystemState.CORAL_HOLD;
                }else if(hasAlgae()){
                    setWantedState(WantedState.ALGAE_HOLDING);
                    yield SystemState.ALGAE_HOLD;
                }
                yield SystemState.CORAL_INTAKING;
            }
            case ALGAE_INTAKE -> {
                if(hasAlgae()){
                    setWantedState(WantedState.ALGAE_HOLDING);
                    yield SystemState.ALGAE_HOLD;
                }else if(hasCoral()){
                    setWantedState(WantedState.CORAL_HOLDING);
                    yield SystemState.CORAL_HOLD;
                }
                yield SystemState.CORAL_INTAKING;
            }
            case CORAL_HOLDING -> SystemState.CORAL_HOLD;
            case ALGAE_HOLDING -> SystemState.ALGAE_HOLD;
            case CORAL_SHOOT -> {
                if(isCoralShootFinished()){
                    setWantedState(WantedState.IDLE);
                    yield SystemState.IDLING;
                }
                yield SystemState.CORAL_SHOOTING;
            }
            case ALGAE_SHOOT -> {
                if(isAlgaeShootFinished()){
                    setWantedState(WantedState.IDLE);
                    yield SystemState.IDLING;
                }
                yield SystemState.ALGAE_SHOOTING;
            }
            case OFF -> SystemState.OFF;
        };
    }

    public boolean hasCoral() {
        return CoralBBInputs.isBeambreakOn;
    }

    public boolean hasAlgae() {
        return AlgaeBBInputs.isBeambreakOn;
    }

    public boolean isCoralShootFinished(){
        return !CoralBBInputs.isBeambreakOn;
    }

    public boolean isAlgaeShootFinished(){
        return !AlgaeBBInputs.isBeambreakOn;
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public enum WantedState {
        IDLE,
        CORAL_HOLDING,
        ALGAE_HOLDING,
        CORAL_INTAKE,
        ALGAE_INTAKE,
        CORAL_SHOOT,
        ALGAE_SHOOT,
        OFF
    }

    public enum SystemState {
        IDLING,
        CORAL_HOLD,
        ALGAE_HOLD,
        CORAL_INTAKING,
        ALGAE_INTAKING,
        CORAL_SHOOTING,
        ALGAE_SHOOTING,
        OFF
    }
}
