package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.display.SuperstructureVisualizer;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Timer;


import static frc.robot.RobotConstants.IntakeConstants.*;

public class IntakeSubsystem extends RollerSubsystem {
    public static final String NAME = "Intake/Roller";
    private static double deployAngle = DEPLOY_ANGLE.get();
    private static double outtakeAngle = OUTTAKE_ANGLE.get();
    private static double funnelAvoidAngle = FUNNEL_AVOID_ANGLE.get();
    private static double homeAngle = HOME_ANGLE.get();
    private static double intakeVoltage = INTAKE_VOLTAGE.get();
    private static double outtakeVoltage = OUTTAKE_VOLTAGE.get();
    private static double rollerAmpsHasCoral = ROLLER_AMPS_HAS_CORAL.get();
    private final IntakePivotIO intakePivotIO;
    private final IntakeRollerIO intakeRollerIO;
    private final IntakePivotIOInputsAutoLogged intakePivotIOInputs = new IntakePivotIOInputsAutoLogged();
    private final RollerIOInputsAutoLogged intakeRollerIOInputs = new RollerIOInputsAutoLogged();
    private WantedState wantedState = WantedState.HOME;
    private SystemState systemState = SystemState.HOMING;
    private boolean hasHomed = false;
    private double currentFilterValue = 0.0;
    private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
    Timer timer = new Timer();
    private boolean timerStarted = false;

    public IntakeSubsystem(
            IntakePivotIO intakePivotIO,
            IntakeRollerIO intakeRollerIO
    ) {
        super(intakeRollerIO, NAME);
        this.intakePivotIO = intakePivotIO;
        this.intakeRollerIO = intakeRollerIO;
    }

    @Override
    public void periodic() {
        super.periodic();

        intakePivotIO.updateInputs(intakePivotIOInputs);

        SystemState newState = handleStateTransition();

        Logger.processInputs("Intake/Pivot", intakePivotIOInputs);

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
                intakeRollerIO.setVoltage(intakeVoltage);
                intakePivotIO.setPivotAngle(deployAngle);
                break;
            case TREMBLE_INTAKING:
                trembleIntake();
                break;
            case OUTTAKING:
                intakeRollerIO.setVoltage(outtakeVoltage);
                intakePivotIO.setPivotAngle(outtakeAngle);
                break;
            case FUNNEL_AVOIDING:
                intakeRollerIO.stop();
                intakePivotIO.setPivotAngle(funnelAvoidAngle);
                break;
            case HOMING:
                intakeRollerIO.stop();
                intakePivotIO.setPivotAngle(homeAngle);
                break;
            case GROUNDZEROING:
                zeroIntakeGround();
                break;
            case OFF:
        }

        if (RobotConstants.TUNING) {
            deployAngle = DEPLOY_ANGLE.get();
            funnelAvoidAngle = FUNNEL_AVOID_ANGLE.get();
            outtakeVoltage = OUTTAKE_VOLTAGE.get();
            outtakeAngle = OUTTAKE_ANGLE.get();
            homeAngle = HOME_ANGLE.get();
            intakeVoltage = INTAKE_VOLTAGE.get();
            rollerAmpsHasCoral = ROLLER_AMPS_HAS_CORAL.get();
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case DEPLOY_WITHOUT_ROLL -> SystemState.DEPLOY_WITHOUT_ROLLING;
            case DEPLOY_INTAKE -> SystemState.DEPLOY_INTAKING;
            case TREMBLE_INTAKE -> SystemState.TREMBLE_INTAKING;
            case OUTTAKE -> SystemState.OUTTAKING;
            case FUNNEL_AVOID -> SystemState.FUNNEL_AVOIDING;
            case HOME -> {
                if (RobotContainer.elevatorIsDanger) {
                    yield SystemState.FUNNEL_AVOIDING;
                } else {
                    yield SystemState.HOMING;
                }
            }
            case GROUNDZERO -> SystemState.GROUNDZEROING;
            case OFF -> SystemState.OFF;
        };
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void trembleIntake() {
        intakeRollerIO.setVoltage(intakeVoltage);
        intakePivotIO.setPivotAngle(deployAngle - 3);
        if (intakePivotIOInputs.currentAngleDeg > deployAngle + 2) {
            intakePivotIO.setPivotAngle(deployAngle - 3);
        } else if (intakePivotIOInputs.currentAngleDeg < deployAngle - 2) {
            intakePivotIO.setPivotAngle(deployAngle + 3);
        }

    }

    public void zeroIntakeGround() {
        intakeRollerIO.stop();
        if (!isNearAngle(100) && !hasHomed) {
                intakePivotIO.setPivotAngle(100);
                return;
        }
        hasHomed = true;
        if(RobotBase.isReal()) {
            if (currentFilterValue <= 15) {
                intakePivotIO.setMotorVoltage(1);
                setWantedState(WantedState.GROUNDZERO);
            }
            if (currentFilterValue > 15) {
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

    private void rollerIntake(){
        intakeRollerIO.setVoltage(intakeVoltage);
        if(intakeRollerIOInputs.supplyCurrentAmps > rollerAmpsHasCoral && !timerStarted){
            timer.start();
            return;
        }
        if(intakeRollerIOInputs.supplyCurrentAmps < rollerAmpsHasCoral && timerStarted){
            timer.stop();
            timer.reset();
            timerStarted = false;
        }
        if(timerStarted && timer.hasElapsed(2)){
            intakeRollerIO.setVoltage(-6);
            if(timer.hasElapsed(2.5)){
                intakeRollerIO.setVoltage(intakeVoltage);
                timer.stop();
                timer.reset();
                timerStarted = false;
            }
        }

    }

    public boolean hasCoral(){
        return timerStarted && timer.hasElapsed(0.1);
    }


    public boolean isNearAngle(double targetAngleDeg) {
        return MathUtil.isNear(targetAngleDeg, intakePivotIOInputs.currentAngleDeg, 1);
    }

    public boolean intakeIsDanger() {
        return intakePivotIOInputs.currentAngleDeg < INTAKE_DANGER_ZONE + 2;
    }

    private boolean intakeIsAvoiding() {
        return intakePivotIOInputs.currentAngleDeg > 50;
    }

    public enum WantedState {
        DEPLOY_WITHOUT_ROLL,
        DEPLOY_INTAKE,
        TREMBLE_INTAKE,
        OUTTAKE,
        FUNNEL_AVOID,
        HOME,
        GROUNDZERO,
        OFF
    }

    public enum SystemState {
        DEPLOY_WITHOUT_ROLLING,
        DEPLOY_INTAKING,
        TREMBLE_INTAKING,
        OUTTAKING,
        FUNNEL_AVOIDING,
        HOMING,
        GROUNDZEROING,
        OFF
    }
}