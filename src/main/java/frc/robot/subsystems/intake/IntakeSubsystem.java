package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;
import frc.robot.RobotConstants;
import frc.robot.subsystems.roller.RollerSubsystem;

import static frc.robot.RobotConstants.intakeConstants.*;

public class IntakeSubsystem extends RollerSubsystem{
    public static final String NAME = "Intake/Roller";
    private final IntakePivotIO intakePivotIO;
    private final IntakeRollerIO intakeRollerIO;
    private IntakePivotIOInputsAutoLogged intakePivotIOInputs = new IntakePivotIOInputsAutoLogged();

    private WantedState wantedState = WantedState.HOME;
    private SystemState systemState = SystemState.HOMED;

    private static double deployAngle = DEPLOY_ANGLE.get();
    private static double funnelAvoidAngle = FUNNEL_AVOID_ANGLE.get();
    private static double homeAngle = HOME_ANGLE.get();
    private static double intakeVoltage = INTAKE_VOLTAGE.get();

    public IntakeSubsystem(
        IntakePivotIO intakePivotIO,
        IntakeRollerIO intakeRollerIO
    ){
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

        Logger.recordOutput("Intake/SystemState",systemState.toString());

        if(newState!= systemState) {
            systemState = newState;
        }

        switch (systemState) {
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
            case OFF:
        }

        if (RobotConstants.TUNING) {
            deployAngle = DEPLOY_ANGLE.get();
            funnelAvoidAngle = FUNNEL_AVOID_ANGLE.get();
            homeAngle = HOME_ANGLE.get();
            intakeVoltage = INTAKE_VOLTAGE.get();
        }
    };

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case DEPLOY_INTAKE -> SystemState.DEPLOY_INTAKING;
            case TREMBLE_INTAKE -> SystemState.TREMBLE_INTAKING;
            case OUTTAKE -> SystemState.OUTTAKING;
            case FUNNEL_AVOID -> SystemState.FUNNEL_AVOIDING;
            case HOME -> SystemState.HOMED;
            case GROUNDZERO -> SystemState.GROUNDZEROING;
            case OFF -> SystemState.OFF;
        };
    }

    public void setWantedState(WantedState wantedState) {this.wantedState = wantedState;}

    public enum WantedState{
        DEPLOY_INTAKE,
        TREMBLE_INTAKE,
        OUTTAKE,
        FUNNEL_AVOID,
        HOME,
        GROUNDZERO,
        OFF
    }
    public enum SystemState{
        DEPLOY_INTAKING,
        TREMBLE_INTAKING,
        OUTTAKING,
        FUNNEL_AVOIDING,
        HOMED,
        GROUNDZEROING,
        OFF
    }

    public void trembleIntake() {
        intakeRollerIO.setVoltage(intakeVoltage);
        intakePivotIO.setMotorPosition(deployAngle - 3);
        if (intakePivotIOInputs.currentPositionDeg > deployAngle + 2) {
            intakePivotIO.setMotorPosition(deployAngle - 3);
        } else if (intakePivotIOInputs.currentPositionDeg < deployAngle - 2){
            intakePivotIO.setMotorPosition(deployAngle + 3);
        }
    }

    public void zeroIntakeGround() {
        intakePivotIO.setMotorVoltage(0.5);
        if (intakePivotIOInputs.statorCurrentAmps > 10) {
            intakePivotIO.resetPosition(120);
            setWantedState(WantedState.FUNNEL_AVOID);
        }
    }

    public boolean isNearAngle(double targetAngle) {
        return MathUtil.isNear(targetAngle, intakePivotIOInputs.currentPositionDeg, 0.5);
    }
}