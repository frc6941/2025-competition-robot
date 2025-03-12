package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.display.SuperstructureVisualizer;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.IntakeConstants.*;

public class IntakeSubsystem extends RollerSubsystem {
    public static final String NAME = "Intake/Roller";
    private static double shootAngle = SHOOT_ANGLE.get();
    private static double deployAngle = DEPLOY_ANGLE.get();
    private static double outtakeAngle = OUTTAKE_ANGLE.get();
    private static double shootVoltage = SHOOT_VOLTAGE.get();
    private static double outtakeHoldVoltage = OUT_TAKE_HOLD.get();
    private static double avoidAngle = AVOID_ANGLE.get();
    private static double homeAngle = HOME_ANGLE.get();
    private static double funnelAvoidAngle = FUNNEL_AVOID_ANGLE.get();
    private static double intakeVoltage = INTAKE_VOLTAGE.get();
    private static double intakeHoldVoltage = INTAKE_HOLD_VOLTAGE.get();
    private static double outtakeVoltage = OUTTAKE_VOLTAGE.get();
    private static double rollerAmpsHasCoral = ROLLER_AMPS_HAS_CORAL.get();
    private static double intakeTime = INTAKE_TIME.get();
    private static double outtakeTime = OUTTAKE_TIME.get();
    private final IntakePivotIO intakePivotIO;
    private final IntakeRollerIO intakeRollerIO;
    private final IntakePivotIOInputsAutoLogged intakePivotIOInputs = new IntakePivotIOInputsAutoLogged();
    private final RollerIOInputsAutoLogged intakeRollerIOInputs = new RollerIOInputsAutoLogged();
    private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
    private final BeambreakIO BBIO;
    private final BeambreakIOInputsAutoLogged BBInputs = new BeambreakIOInputsAutoLogged();
    public boolean hasHomed = false;
    Timer timer = new Timer();
    private boolean shouldOuttake = false;
    private WantedState wantedState = WantedState.HOME;
    @Getter
    private SystemState systemState = SystemState.HOMING;
    private double currentFilterValue = 0.0;
    private boolean timerStarted = false;
    @Setter
    private boolean lowerAngle = false;

    public IntakeSubsystem(
            IntakePivotIO intakePivotIO,
            IntakeRollerIO intakeRollerIO,
            BeambreakIO BBIO
    ) {
        super(intakeRollerIO, NAME);
        this.intakePivotIO = intakePivotIO;
        this.intakeRollerIO = intakeRollerIO;
        this.BBIO = BBIO;
    }

    @Override
    public void periodic() {
        super.periodic();

        BBIO.updateInputs(BBInputs);
        intakePivotIO.updateInputs(intakePivotIOInputs);

        SystemState newState = handleStateTransition();

        Logger.processInputs("Intake/Pivot", intakePivotIOInputs);
        Logger.processInputs(NAME + "/Beambreak", BBInputs);
        Logger.recordOutput("Intake/SystemState", systemState.toString());

        RobotContainer.intakeIsDanger = intakeIsDanger();
        RobotContainer.intakeIsAvoiding = intakeIsAvoiding();
        Logger.recordOutput("Flags/intakeIsDanger", intakeIsDanger());

        SuperstructureVisualizer.getInstance().updateIntake(intakePivotIOInputs.currentAngleDeg);

        currentFilterValue = currentFilter.calculate(intakePivotIOInputs.statorCurrentAmps);
        Logger.recordOutput("Intake/StatorCurrent", currentFilterValue);

        if (newState != systemState) {
            systemState = newState;
        }

        switch (systemState) {
            case DEPLOY_WITHOUT_ROLLING:
                intakeRollerIO.stop();
                intakePivotIO.setPivotAngle(deployAngle);
                break;
            case DEPLOY_INTAKING:
                rollerIntake();
                intakePivotIO.setPivotAngle(deployAngle);
                break;
            case TREMBLE_INTAKING:
                trembleIntake();
                break;
            case OUTTAKING:
                intakeRollerIO.setVoltage(outtakeVoltage);
                intakePivotIO.setPivotAngle(outtakeAngle);
                break;
            case HOLD_OUTTAKING:
                intakeRollerIO.setVoltage(outtakeHoldVoltage);
                intakePivotIO.setPivotAngle(deployAngle);
                break;
            case SHOOTING:
                intakeRollerIO.setVoltage(shootVoltage);
                intakePivotIO.setPivotAngle(shootAngle);
                break;
            case DEPLOY_SHOOTING:
                intakeRollerIO.setVoltage(0);
                intakePivotIO.setPivotAngle(shootAngle);
                break;
            case AVOIDING:
                intakeRollerIO.stop();
                intakePivotIO.setPivotAngle(avoidAngle);
                break;
            case HOMING:
                intakeRollerIO.stop();
                intakePivotIO.setPivotAngle(homeAngle);
                break;
            case GROUNDZEROING:
                zeroIntakeGround();
                break;
            case FUNNEL_AVOIDING:
                intakeRollerIO.stop();
                intakePivotIO.setPivotAngle(funnelAvoidAngle);
                break;
            case DEPLOY_INTAKE_HOLDING:
                rollerHoldIntake();
                intakePivotIO.setPivotAngle(deployAngle);
                break;
            case OFF:
        }

        if (RobotConstants.TUNING) {
            deployAngle = DEPLOY_ANGLE.get();
            avoidAngle = AVOID_ANGLE.get();
            funnelAvoidAngle = FUNNEL_AVOID_ANGLE.get();
            outtakeVoltage = OUTTAKE_VOLTAGE.get();
            shootVoltage = SHOOT_VOLTAGE.get();
            outtakeAngle = OUTTAKE_ANGLE.get();
            homeAngle = HOME_ANGLE.get();
            intakeVoltage = INTAKE_VOLTAGE.get();
            rollerAmpsHasCoral = ROLLER_AMPS_HAS_CORAL.get();
            intakeTime = INTAKE_TIME.get();
            outtakeTime = OUTTAKE_TIME.get();
            shootAngle = SHOOT_ANGLE.get();
            intakeHoldVoltage = INTAKE_HOLD_VOLTAGE.get();
            outtakeHoldVoltage = OUT_TAKE_HOLD.get();
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case DEPLOY_WITHOUT_ROLL -> SystemState.DEPLOY_WITHOUT_ROLLING;
            case DEPLOY_INTAKE -> {
                if (lowerAngle) {
                    yield SystemState.TREMBLE_INTAKING;
                } else {
                    yield SystemState.DEPLOY_INTAKING;
                }
            }
            case DEPLOY_INTAKE_HOLD -> SystemState.DEPLOY_INTAKE_HOLDING;
            case TREMBLE_INTAKE -> SystemState.TREMBLE_INTAKING;
            case OUTTAKE -> SystemState.OUTTAKING;
            case HOLD_OUTTAKE -> SystemState.HOLD_OUTTAKING;
            case AVOID -> SystemState.AVOIDING;
            case FUNNEL_AVOID -> SystemState.FUNNEL_AVOIDING;
            case HOME -> {
                if (RobotContainer.elevatorIsDanger) {
                    yield SystemState.AVOIDING;
                } else {
                    yield SystemState.HOMING;
                }
            }
            case SHOOT -> SystemState.SHOOTING;
            case DEPLOY_SHOOT -> SystemState.DEPLOY_SHOOTING;
            case GROUNDZERO -> SystemState.GROUNDZEROING;
            case OFF -> SystemState.OFF;
        };
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void trembleIntake() {
        intakeRollerIO.setVoltage(intakeVoltage);
        intakePivotIO.setPivotAngle(deployAngle + 2);
    }

    public void zeroIntakeGround() {
        intakeRollerIO.stop();
        if (!isNearAngle(101) && !hasHomed) {
            intakePivotIO.setPivotAngle(100);
            return;
        }
        hasHomed = true;
        if (RobotBase.isReal()) {
            if (currentFilterValue <= 18) {
                intakePivotIO.setMotorVoltage(0.5);
                setWantedState(WantedState.GROUNDZERO);
            }
            if (currentFilterValue > 18) {
                intakePivotIO.setMotorVoltage(0);
                intakePivotIO.resetAngle(120);
                setWantedState(WantedState.HOME);
                hasHomed = false;
            }
        } else {
            intakePivotIO.setPivotAngle(0);
            setWantedState(WantedState.HOME);
            hasHomed = false;
        }
    }

    private void rollerIntake() {
        if (inputs.statorCurrentAmps > rollerAmpsHasCoral && !timerStarted) {
            timer.start();
            timerStarted = true;
        }
        if (inputs.statorCurrentAmps < rollerAmpsHasCoral && timerStarted && !shouldOuttake) {
            timer.stop();
            timer.reset();
            timerStarted = false;
        }
        if (timerStarted && timer.hasElapsed(outtakeTime)) {
            intakeRollerIO.setVoltage(outtakeVoltage);
            shouldOuttake = true;
            if (timer.hasElapsed(intakeTime)) {
                intakeRollerIO.setVoltage(intakeVoltage);
                timer.stop();
                timer.reset();
                shouldOuttake = false;
                timerStarted = false;
            }
        } else {
            intakeRollerIO.setVoltage(intakeVoltage);
        }

    }

    private void rollerHoldIntake() {
        if (inputs.statorCurrentAmps > rollerAmpsHasCoral && !timerStarted) {
            timer.start();
            timerStarted = true;
        }
        if (inputs.statorCurrentAmps < rollerAmpsHasCoral && timerStarted && !shouldOuttake) {
            timer.stop();
            timer.reset();
            timerStarted = false;
        }
        if (timerStarted && timer.hasElapsed(outtakeTime)) {
            intakeRollerIO.setVoltage(outtakeVoltage);
            shouldOuttake = true;
            if (timer.hasElapsed(intakeTime)) {
                intakeRollerIO.setVoltage(intakeHoldVoltage);
                timer.stop();
                timer.reset();
                shouldOuttake = false;
                timerStarted = false;
            }
        } else {
            intakeRollerIO.setVoltage(intakeHoldVoltage);
        }

    }

    public boolean hasCoral() {
        return timerStarted && timer.hasElapsed(0.1);
    }

    public boolean hasCoralBB() {
        return BBInputs.isBeambreakOn;
    }


    public boolean isNearAngle(double targetAngleDeg) {
        return MathUtil.isNear(targetAngleDeg, intakePivotIOInputs.currentAngleDeg, 1);
        //TODO tolerance ++
    }

    public boolean intakeIsDanger() {
        return intakePivotIOInputs.currentAngleDeg < INTAKE_DANGER_ZONE - 2;
    }

    private boolean intakeIsAvoiding() {
        return intakePivotIOInputs.currentAngleDeg > 50;
    }

    public enum WantedState {
        DEPLOY_WITHOUT_ROLL,
        DEPLOY_INTAKE,
        TREMBLE_INTAKE,
        OUTTAKE,
        HOLD_OUTTAKE,
        AVOID,
        FUNNEL_AVOID,
        HOME,
        GROUNDZERO,
        DEPLOY_SHOOT,
        SHOOT,
        DEPLOY_INTAKE_HOLD,
        OFF,
    }

    public enum SystemState {
        DEPLOY_WITHOUT_ROLLING,
        DEPLOY_INTAKING,
        TREMBLE_INTAKING,
        OUTTAKING,
        HOLD_OUTTAKING,
        AVOIDING,
        FUNNEL_AVOIDING,
        HOMING,
        GROUNDZEROING,
        DEPLOY_SHOOTING,
        SHOOTING,
        DEPLOY_INTAKE_HOLDING,
        OFF,
    }
}