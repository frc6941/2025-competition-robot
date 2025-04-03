package frc.robot.subsystems.endeffectorarm;

import edu.wpi.first.math.MathUtil;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.display.SuperstructureVisualizer;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.EndEffectorArmConstants.*;

public class EndEffectorArmSubsystem extends RollerSubsystem {
    public static final String NAME = "EndEffectorArm";

    // Static variables to hold the current values from TunableNumbers
    private static double homeAngle = HOME_ANGLE.get();
    private static double coralIntakeAngle = CORAL_INTAKE_ANGLE.get();
    private static double coralOuttakeAngle = CORAL_OUTTAKE_ANGLE.get();
    private static double coralPreShootAngle = CORAL_PRESHOOT_ANGLE.get();
    private static double algaeIntakeAngle = ALGAE_INTAKE_ANGLE.get();
    private static double algaePreShootAngle = ALGAE_PRESHOOT_ANGLE.get();

    private static double coralIntakeVoltage = CORAL_INTAKE_VOLTAGE.get();
    private static double coralOuttakeVoltage = CORAL_OUTTAKE_VOLTAGE.get();
    private static double coralPreShootVoltage = CORAL_PRESHOOT_VOLTAGE.get();
    private static double algaeIntakeVoltage = ALGAE_INTAKE_VOLTAGE.get();
    private static double algaePreShootVoltage = ALGAE_PRESHOOT_VOLTAGE.get();
    private static double coralHoldVoltage = CORAL_HOLD_VOLTAGE.get();
    private static double algaeHoldVoltage = ALGAE_HOLD_VOLTAGE.get();

    // Add these constants near the other static variables
    private static double coralShootVoltage = CORAL_SHOOT_VOLTAGE.get();
    private static double algaeShootVoltage = ALGAE_SHOOT_VOLTAGE.get();

    // IO devices and their inputs
    private final EndEffectorArmPivotIO armPivotIO;
    private final EndEffectorArmRollerIO armRollerIO;
    private final EndEffectorArmPivotIOInputsAutoLogged armPivotIOInputs = new EndEffectorArmPivotIOInputsAutoLogged();
    private final RollerIOInputsAutoLogged armRollerIOInputs = new RollerIOInputsAutoLogged();

    // Beambreak sensors for coral and algae detection
    private final BeambreakIO coralBeambreakIO;
    private final BeambreakIO algaeBeambreakIO;
    private final BeambreakIOInputsAutoLogged coralBeambreakInputs = new BeambreakIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged algaeBeambreakInputs = new BeambreakIOInputsAutoLogged();

    // State tracking
    @Setter
    private WantedState wantedState = WantedState.HOME;
    @Getter
    private SystemState systemState = SystemState.HOMING;

    /**
     * Creates a new EndEffectorArmSubsystem
     *
     * @param armPivotIO       The IO interface for the arm pivot
     * @param armRollerIO      The IO interface for the roller
     * @param coralBeambreakIO The beambreak sensor for coral detection
     * @param algaeBeambreakIO The beambreak sensor for algae detection
     */
    public EndEffectorArmSubsystem(
            EndEffectorArmPivotIO armPivotIO,
            EndEffectorArmRollerIO armRollerIO,
            BeambreakIO coralBeambreakIO,
            BeambreakIO algaeBeambreakIO) {
        super(armRollerIO, NAME);
        this.armPivotIO = armPivotIO;
        this.armRollerIO = armRollerIO;
        this.coralBeambreakIO = coralBeambreakIO;
        this.algaeBeambreakIO = algaeBeambreakIO;
    }

    @Override
    public void periodic() {
        super.periodic();

        // Update inputs from hardware
        armPivotIO.updateInputs(armPivotIOInputs);
        coralBeambreakIO.updateInputs(coralBeambreakInputs);
        algaeBeambreakIO.updateInputs(algaeBeambreakInputs);

        // Update danger flag based on arm position
        RobotContainer.endeffectorIsDanger = !isNearAngle(coralIntakeAngle);

        // Calculate current state transition
        SystemState newState;
        if (RobotContainer.elevatorIsDanger) {
            // Force CORAL_INTAKING or CORAL_OUTTAKING while elevator is in danger
            if (systemState == SystemState.CORAL_OUTTAKING) {
                newState = SystemState.CORAL_OUTTAKING;
            } else if (systemState == SystemState.CORAL_INTAKING) {
                newState = SystemState.CORAL_INTAKING;
            } else {
                newState = SystemState.NEUTRAL;
            }
        } else {
            // Normal state transitions when elevator is safe
            newState = handleStateTransition();
        }

        // Process and log inputs
        Logger.processInputs(NAME + "/Pivot", armPivotIOInputs);
        Logger.processInputs(NAME + "/Coral Beambreak", coralBeambreakInputs);
        Logger.processInputs(NAME + "/Algae Beambreak", algaeBeambreakInputs);

        // Log the system state
        Logger.recordOutput("EndEffectorArm/SystemState", systemState.toString());

        Logger.recordOutput("Flags/eeIsDanger", !isNearAngle(coralIntakeAngle));

        SuperstructureVisualizer.getInstance().updateEndEffector(armPivotIOInputs.currentAngleDeg);

        // Apply the new state if it has changed
        if (newState != systemState) {
            systemState = newState;
        }

        // Handle actions based on current system state
        switch (systemState) {
            case CORAL_INTAKING:
                armRollerIO.setVoltage(coralIntakeVoltage);
                armPivotIO.setPivotAngle(coralIntakeAngle);
                break;

            case CORAL_OUTTAKING:
                armRollerIO.setVoltage(coralOuttakeVoltage);
                armPivotIO.setPivotAngle(coralOuttakeAngle);
                break;

            case CORAL_PRESHOOTING:
                armPivotIO.setPivotAngle(coralPreShootAngle);
                break;

            case ALGAE_INTAKING:
                armRollerIO.setVoltage(algaeIntakeVoltage);
                armPivotIO.setPivotAngle(algaeIntakeAngle);
                break;

            case ALGAE_PRESHOOTING:
                armPivotIO.setPivotAngle(algaePreShootAngle);
                break;

            case HOMING:
                if (hasAlgae()) {
                    armRollerIO.setVoltage(algaeHoldVoltage);
                    armPivotIO.setPivotAngle(algaeIntakeAngle);
                } else if (hasCoral()) {
                    armPivotIO.setPivotAngle(homeAngle);
                    armRollerIO.setVoltage(coralHoldVoltage);
                } else {
                    setWantedState(WantedState.NEUTRAL);
                    systemState = SystemState.NEUTRAL;
                }
                break;

            case NEUTRAL:
                armRollerIO.stop();
                armPivotIO.setPivotAngle(coralIntakeAngle);
                break;

            case CORAL_SHOOTING:
                armRollerIO.setVoltage(coralShootVoltage);
                break;

            case ALGAE_SHOOTING:
                armRollerIO.setVoltage(algaeShootVoltage);
                break;
        }

        // Update tunable numbers if tuning is enabled
        if (RobotConstants.TUNING) {
            homeAngle = HOME_ANGLE.get();
            coralIntakeAngle = CORAL_INTAKE_ANGLE.get();
            coralOuttakeAngle = CORAL_OUTTAKE_ANGLE.get();
            coralPreShootAngle = CORAL_PRESHOOT_ANGLE.get();
            algaeIntakeAngle = ALGAE_INTAKE_ANGLE.get();
            algaePreShootAngle = ALGAE_PRESHOOT_ANGLE.get();

            coralIntakeVoltage = CORAL_INTAKE_VOLTAGE.get();
            coralOuttakeVoltage = CORAL_OUTTAKE_VOLTAGE.get();
            coralPreShootVoltage = CORAL_PRESHOOT_VOLTAGE.get();
            algaeIntakeVoltage = ALGAE_INTAKE_VOLTAGE.get();
            algaePreShootVoltage = ALGAE_PRESHOOT_VOLTAGE.get();
            coralHoldVoltage = CORAL_HOLD_VOLTAGE.get();
            algaeHoldVoltage = ALGAE_HOLD_VOLTAGE.get();

            coralShootVoltage = CORAL_SHOOT_VOLTAGE.get();
            algaeShootVoltage = ALGAE_SHOOT_VOLTAGE.get();
        }
    }

    /**
     * Handles the transition between states based on the current wanted state
     *
     * @return The new system state
     */
    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case CORAL_INTAKE -> {
                if (hasCoral()) {
                    setWantedState(WantedState.HOME);
                    yield SystemState.HOMING;
                }
                yield SystemState.CORAL_INTAKING;
            }
            case CORAL_OUTTAKE -> SystemState.CORAL_OUTTAKING;
            case CORAL_PRESHOOT -> SystemState.CORAL_PRESHOOTING;
            case ALGAE_INTAKE -> {
                if (hasAlgae()) {
                    setWantedState(WantedState.HOME);
                    yield SystemState.HOMING;
                }
                yield SystemState.ALGAE_INTAKING;
            }
            case ALGAE_PRESHOOT -> SystemState.ALGAE_PRESHOOTING;
            case HOME -> {
                if (!hasAlgae() && !hasCoral()) {
                    yield SystemState.NEUTRAL;
                }
                yield SystemState.HOMING;
            }
            case NEUTRAL -> SystemState.NEUTRAL;
            case CORAL_SHOOT -> {
                if (isShootFinished()) {
                    setWantedState(WantedState.HOME);
                    yield SystemState.HOMING;
                }
                yield SystemState.CORAL_SHOOTING;
            }
            case ALGAE_SHOOT -> {
                if (isShootFinished()) {
                    setWantedState(WantedState.HOME);
                    yield SystemState.HOMING;
                }
                yield SystemState.ALGAE_SHOOTING;
            }
        };
    }

    /**
     * Checks if the arm is near a target angle
     *
     * @param targetAngleDeg The target angle in degrees
     * @return True if the arm is within 1 degree of the target
     */
    public boolean isNearAngle(double targetAngleDeg) {
        return MathUtil.isNear(targetAngleDeg, armPivotIOInputs.currentAngleDeg, 5);
        //TODO：isNear angle value
    }

    /**
     * Checks if the mechanism has coral
     *
     * @return True if coral is detected by the beambreak
     */
    public boolean hasCoral() {
        return coralBeambreakInputs.isBeambreakOn;
    }

    /**
     * Checks if the mechanism has algae
     *
     * @return True if algae is detected by the beambreak
     */
    public boolean hasAlgae() {
        return algaeBeambreakInputs.isBeambreakOn;
    }

    /**
     * Checks if the shoot is finished
     *
     * @return True if neither coral nor algae is detected
     */
    public boolean isShootFinished() {
        return !hasCoral() && !hasAlgae();
    }

    /**
     * Checks if is ready to shoot, i.e. angle is ready and has a coral
     *
     * @return True if both angle is near the preshoot angle and contains a coral
     */
    public boolean isShootReady(){
        return isNearAngle(coralPreShootAngle);
    }

    /**
     * The wanted state for the EndEffectorArm subsystem
     */
    public enum WantedState {
        CORAL_INTAKE,
        CORAL_OUTTAKE,
        CORAL_PRESHOOT,
        CORAL_SHOOT,
        ALGAE_INTAKE,
        ALGAE_PRESHOOT,
        ALGAE_SHOOT,
        HOME,
        NEUTRAL
    }

    /**
     * The current system state of the EndEffectorArm subsystem
     */
    public enum SystemState {
        CORAL_INTAKING,
        CORAL_OUTTAKING,
        CORAL_PRESHOOTING,
        CORAL_SHOOTING,
        ALGAE_INTAKING,
        ALGAE_PRESHOOTING,
        ALGAE_SHOOTING,
        HOMING,
        NEUTRAL
    }
} 