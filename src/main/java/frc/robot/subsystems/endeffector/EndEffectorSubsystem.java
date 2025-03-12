package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.ElevatorConstants.L1_EXTENSION_METERS;
import static frc.robot.RobotConstants.EndEffectorConstants.EndEffectorGainsClass.*;
import static frc.robot.RobotConstants.EndEffectorConstants.*;

public class EndEffectorSubsystem extends RollerSubsystem {
    public static final String NAME = "EndEffector";
    private static final double idleRPS = IDLE_RPS.get();
    private static double intakeRPS = INTAKE_RPS.get();
    private static double preShootRPS = PRE_SHOOT_RPS.get();
    private static double shootRPS = SHOOT_RPS.get();
    private static double l1RPS = L1_RPS.get();
    private static double reverseRPS = REVERSE_RPS.get();
    private final EndEffectorIO endEffectorIO;
    private final BeambreakIO middleBBIO, edgeBBIO;
    private final BeambreakIOInputsAutoLogged middleBBInputs = new BeambreakIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged edgeBBInputs = new BeambreakIOInputsAutoLogged();
    public double kp = ENDEFFECTOR_KP.get();
    public double ki = ENDEFFECTOR_KI.get();
    public double kd = ENDEFFECTOR_KD.get();
    public double ka = ENDEFFECTOR_KA.get();
    public double kv = ENDEFFECTOR_KV.get();
    public double ks = ENDEFFECTOR_KS.get();
    private WantedState wantedState = WantedState.IDLE;

    @Getter
    private SystemState systemState = SystemState.IDLING;
    private boolean hasTransitionedToPreShoot = false;

    public EndEffectorSubsystem(EndEffectorIO endEffectorIO, BeambreakIO middleBBIO, BeambreakIO edgeBBIO) {
        super(endEffectorIO, NAME);
        this.endEffectorIO = endEffectorIO;
        this.middleBBIO = middleBBIO;
        this.edgeBBIO = edgeBBIO;
    }

    @Override
    public void periodic() {
        super.periodic();

        middleBBIO.updateInputs(middleBBInputs);
        edgeBBIO.updateInputs(edgeBBInputs);
        Logger.processInputs(NAME + "/Middle Beambreak", middleBBInputs);
        Logger.processInputs(NAME + "/Edge Beambreak", edgeBBInputs);

        SystemState newState = handleStateTransition();

        Logger.recordOutput("EndEffector/SystemState", newState.toString());
        Logger.recordOutput(NAME + "Velocity", inputs.velocityRotPerSec);

        Logger.recordOutput(NAME + "/isShootReady", isShootReady());
        Logger.recordOutput(NAME + "/ShootFinished", isShootFinished());
        Logger.recordOutput(NAME + "/isIntakeFinished", isIntakeFinished());

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
            case FUNNEL_INTAKING:
                io.setVelocity(-intakeRPS);
                if (isIntakeFinished()) {
                    io.setVelocity(0.0);
                }
                break;
            case GROUND_INTAKING:
                io.setVelocity(intakeRPS);
                if (isIntakeFinished()) {
                    io.setVelocity(0.0);
                }
                break;
            case PRE_SHOOTING:
                io.setVelocity(-preShootRPS);
                if (isShootReady()) {
                    io.setVelocity(0.0);
                }
                break;
            case SHOOTING:
                io.setVelocity(DestinationSupplier.getInstance().getElevatorSetpoint(true) == L1_EXTENSION_METERS.get() ? -l1RPS : -shootRPS);
                break;
            case POKING:
                io.setVoltage(12);
                break;
            case REVERSING:
                io.setVelocity(-reverseRPS);
                break;
            case OFF:
        }

        if (RobotConstants.TUNING) {
            intakeRPS = INTAKE_RPS.get();
            preShootRPS = PRE_SHOOT_RPS.get();
            shootRPS = SHOOT_RPS.get();
            l1RPS = L1_RPS.get();
            reverseRPS = REVERSE_RPS.get();

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
            case FUNNEL_INTAKE -> {
                if (!RobotContainer.elevatorIsDanger && (isIntakeFinished() || hasTransitionedToPreShoot)) {
                    hasTransitionedToPreShoot = true;
                    setWantedState(WantedState.PRE_SHOOT);
                    yield SystemState.PRE_SHOOTING;
                }
                yield SystemState.FUNNEL_INTAKING;
            }
            case GROUND_INTAKE -> {
                if (!RobotContainer.elevatorIsDanger && (isIntakeFinished() || hasTransitionedToPreShoot)) {
                    hasTransitionedToPreShoot = true;
                    setWantedState(WantedState.PRE_SHOOT);
                    yield SystemState.PRE_SHOOTING;
                }
                yield SystemState.GROUND_INTAKING;
            }
            case PRE_SHOOT -> {
                hasTransitionedToPreShoot = true;
                yield SystemState.PRE_SHOOTING;
            }
            case SHOOT -> {
                if (isShootFinished()) {
                    hasTransitionedToPreShoot = false;
                    setWantedState(WantedState.IDLE);
                    yield SystemState.IDLING;
                }
                yield SystemState.SHOOTING;
            }
            case POKE -> SystemState.POKING;
            case OFF -> SystemState.OFF;
            case REVERSE -> SystemState.REVERSING;
        };
    }

    public boolean hasCoral() {
        return middleBBInputs.isBeambreakOn;
    }

    public boolean containsCoral() {
        return middleBBInputs.isBeambreakOn || edgeBBInputs.isBeambreakOn;
    }

    public boolean isShootFinished() {
        return (Swerve.getInstance().getState() == Swerve.State.PATH_FOLLOWING || hasTransitionedToPreShoot) && !edgeBBInputs.isBeambreakOn;
    }

    public boolean isIntakeFinished() {
        return middleBBInputs.isBeambreakOn && !edgeBBInputs.isBeambreakOn;
    }

    public boolean isShootReady() {
        return edgeBBInputs.isBeambreakOn && !middleBBInputs.isBeambreakOn;
    }

    public boolean isPreShootReady() {
        return hasTransitionedToPreShoot;
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public enum WantedState {
        IDLE,
        FUNNEL_INTAKE,
        GROUND_INTAKE,
        PRE_SHOOT,
        SHOOT,
        POKE,
        REVERSE,
        OFF
    }

    public enum SystemState {
        IDLING,
        FUNNEL_INTAKING,
        GROUND_INTAKING,
        PRE_SHOOTING,
        SHOOTING,
        POKING,
        REVERSING,
        OFF
    }
}
