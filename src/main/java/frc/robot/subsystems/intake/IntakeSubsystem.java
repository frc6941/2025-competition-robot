package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.roller.RollerSubsystem;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.IntakeConstants.*;

public class IntakeSubsystem extends RollerSubsystem {
    public static final String NAME = "Intake/Roller";
    private static double deployAngle = DEPLOY_ANGLE.get();
    private static double funnelAvoidAngle = FUNNEL_AVOID_ANGLE.get();
    private static double homeAngle = HOME_ANGLE.get();
    private static double intakeVoltage = INTAKE_VOLTAGE.get();
    private final IntakePivotIO intakePivotIO;
    private final IntakeRollerIO intakeRollerIO;
    private final IntakePivotIOInputsAutoLogged intakePivotIOInputs = new IntakePivotIOInputsAutoLogged();
    private WantedState wantedState = WantedState.HOME;
    private SystemState systemState = SystemState.HOMED;

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
        Logger.recordOutput(NAME + "/isnear", isNearAngle(FUNNEL_AVOID_ANGLE.get()));
        Logger.recordOutput("Flags/intakeIsDanger", intakeIsDanger());

        if (newState != systemState) {
            systemState = newState;
        }

        switch (systemState) {
            case DEPLOY_WITHOUT_ROLLING:
                intakePivotIO.setMotorPosition(deployAngle);
                break;
            case DEPLOY_INTAKING:
                intakeRollerIO.setVoltage(intakeVoltage);
                intakePivotIO.setMotorPosition(deployAngle);
                break;
            case TREMBLE_INTAKING:
                trembleIntake();
                break;
            case OUTTAKING:
                intakeRollerIO.setVoltage(-3);
                intakePivotIO.setMotorPosition(deployAngle);
                break;
            case FUNNEL_AVOIDING:
                intakeRollerIO.stop();
                intakePivotIO.setMotorPosition(funnelAvoidAngle);
                break;
            case HOMED:
                intakeRollerIO.stop();
                intakePivotIO.setMotorPosition(homeAngle);
                break;
            case GROUNDZEROING:
                intakeRollerIO.stop();
                zeroIntakeGround();
                break;
            case COLLISION_AVOIDING:
                intakePivotIO.setMotorPosition(COLLISION_AVOID_ANGLE);
                break;
            case OFF:
        }

        if (RobotConstants.TUNING) {
            deployAngle = DEPLOY_ANGLE.get();
            funnelAvoidAngle = FUNNEL_AVOID_ANGLE.get();
            homeAngle = HOME_ANGLE.get();
            intakeVoltage = INTAKE_VOLTAGE.get();
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
                    yield SystemState.COLLISION_AVOIDING;
                } else {
                    yield SystemState.HOMED;
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
        intakePivotIO.setMotorPosition(deployAngle - 3);
        if (intakePivotIOInputs.currentPositionDeg > deployAngle + 2) {
            intakePivotIO.setMotorPosition(deployAngle - 3);
        } else if (intakePivotIOInputs.currentPositionDeg < deployAngle - 2) {
            intakePivotIO.setMotorPosition(deployAngle + 3);
        }

    }

    public void zeroIntakeGround() {
        intakePivotIO.setMotorVoltage(3);
        if (intakePivotIOInputs.statorCurrentAmps > 10) {
            intakePivotIO.resetPosition(120);
            setWantedState(WantedState.HOME);
        }
    }

    public boolean isNearAngle(double targetAngle) {
        return MathUtil.isNear(targetAngle, intakePivotIOInputs.currentPositionDeg, 1);
    }

    public boolean intakeIsDanger() {
        return intakePivotIOInputs.currentPositionDeg < INTAKE_DANGER_ZONE;
    }

    private boolean intakeIsAvoiding() {
        return intakePivotIOInputs.currentPositionDeg > 50;
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
        COLLISION_AVOIDING,
        HOMED,
        GROUNDZEROING,
        OFF
    }
}