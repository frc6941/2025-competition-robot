package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;

import static frc.robot.RobotConstants.EndEffectorConstants.*;
import static frc.robot.RobotConstants.EndEffectorConstants.EndEffectorGainsClass.*;

public class EndEffectorSubsystem extends RollerSubsystem {

    public static final String NAME = "EndEffector";

    private final EndEffectorIO endEffectorIO;
    private final BeambreakIO middleBBIO, edgeBBIO;
    private BeambreakIOInputsAutoLogged middleBBInputs = new BeambreakIOInputsAutoLogged();
    private BeambreakIOInputsAutoLogged edgeBBInputs = new BeambreakIOInputsAutoLogged();

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    private boolean hasTransitionedToTransfer = false;

    public double kp = ENDEFFECTOR_KP.get();
    public double ki = ENDEFFECTOR_KI.get();
    public double kd = ENDEFFECTOR_KD.get();
    public double ka = ENDEFFECTOR_KA.get();
    public double kv = ENDEFFECTOR_KV.get();
    public double ks = ENDEFFECTOR_KS.get();

    private static double idleRPS = IDLE_RPS.get();
    private static double indexRPS = INDEX_RPS.get();
    private static double transferRPS = TRANSFER_RPS.get();
    private static double holdRPS = HOLD_RPS.get();
    private static double shootRPS = SHOOT_RPS.get();

    public enum WantedState {
        IDLE,
        FUNNEL_INDEX,
        FUNNEL_TRANSFER,
        HOLD,
        SHOOT,
        OFF
    }

    public enum SystemState {
        IDLING,
        FUNNEL_INDEXING,
        FUNNEL_TRANSFERRING,
        HOLDING,
        SHOOTING,
        OFF
    }

    public EndEffectorSubsystem(EndEffectorIO endEffectorIO , BeambreakIO middleBBIO, BeambreakIO edgeBBIO) {
        super(endEffectorIO, NAME);
        this.endEffectorIO = endEffectorIO;
        this.middleBBIO = middleBBIO;
        this.edgeBBIO = edgeBBIO;
    }

    @Override
    public void periodic() {
        super.periodic();

        endEffectorIO.updateConfigs(kp, ki, kd, ka, kv, ks);
        middleBBIO.updateInputs(middleBBInputs);
        edgeBBIO.updateInputs(edgeBBInputs);

        Logger.processInputs(NAME + "/Middle Beambreak", middleBBInputs);
        Logger.processInputs(NAME + "/Edge Beambreak", edgeBBInputs);

        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            Logger.recordOutput("Intaker/SystemState", newState.toString());
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.IDLING;
        }

        switch(systemState) {
            case IDLING:
                io.setVelocity(idleRPS);
                break;
            case FUNNEL_INDEXING:
                io.setVelocity(-indexRPS);
                break;
            case FUNNEL_TRANSFERRING:
                io.setVelocity(-transferRPS);
                break;
            case HOLDING:
                io.setVelocity(holdRPS);
                break;
            case SHOOTING:
                io.setVelocity(shootRPS);
                break;
            case OFF:
            default:
                io.setVelocity(0.0);
                break;
        }

        if (RobotConstants.TUNING) {
            indexRPS = INDEX_RPS.get();
            holdRPS = HOLD_RPS.get();
            transferRPS = TRANSFER_RPS.get();
            shootRPS = SHOOT_RPS.get();

            kp = ENDEFFECTOR_KP.get();
            ki = ENDEFFECTOR_KI.get();
            kd = ENDEFFECTOR_KD.get();
            ka = ENDEFFECTOR_KA.get();
            kv = ENDEFFECTOR_KV.get();
            ks = ENDEFFECTOR_KS.get();
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLING;
            case FUNNEL_INDEX -> {
                if (middleBBInputs.isBeambreakOn) {
                    hasTransitionedToTransfer = true;
                    yield SystemState.FUNNEL_TRANSFERRING;
                }
                yield SystemState.FUNNEL_INDEXING;
            }
            case FUNNEL_TRANSFER -> {
                if (edgeBBInputs.isBeambreakOn && !middleBBInputs.isBeambreakOn) {
                    yield SystemState.HOLDING;
                }
                yield SystemState.FUNNEL_TRANSFERRING;
            }
            case HOLD -> SystemState.HOLDING;
            case SHOOT -> {
                hasTransitionedToTransfer = false;
                yield SystemState.SHOOTING;
            }
            case OFF -> SystemState.OFF;
            default -> SystemState.IDLING;
        };
    }

    public boolean isFunnelIndexFinished () {
        return hasTransitionedToTransfer;
    }

    public void setWantedState(WantedState wantedState) {this.wantedState = wantedState;}
}
