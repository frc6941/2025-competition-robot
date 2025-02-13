package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotConstants;

public class IntakePivotIOSim implements IntakePivotIO {
    private final SingleJointedArmSim intakePivotSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            RobotConstants.intakeConstants.PIVOT_RATIO,
            0.07,
            0.4,
            Math.toRadians(0),
            Math.toRadians(1),
            true,
            0.0);

    private final ProfiledPIDController pivotPid = new ProfiledPIDController(5.0, 0.0, 2.6,
            new TrapezoidProfile.Constraints(10.0, 10.0));
    private final ArmFeedforward pivotFf = new ArmFeedforward(0.0, 0.3856, 0.543); // 0.543

    private double targetPosition = 0.0;

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        intakePivotSim.update(0.02);

        inputs.currentPositionDeg = Math.toDegrees(intakePivotSim.getAngleRads());
        inputs.velocityRotPerSec = RadiansPerSecond.of(intakePivotSim.getVelocityRadPerSec())
                .in(RotationsPerSecond);
        inputs.statorCurrentAmps = intakePivotSim.getCurrentDrawAmps();
        inputs.supplyCurrentAmps = 0.0;
        inputs.tempCelsius = 0.0;
        inputs.targetPositionDeg = targetPosition;
    }

    @Override
    public void setMotorVoltage(double voltage) {
        intakePivotSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    @Override
    public void setMotorPosition(double targetPosition) {
        setMotorVoltage(
                pivotPid.calculate(intakePivotSim.getAngleRads(), targetPosition)
                        + pivotFf.calculate(pivotPid.getSetpoint().position, pivotPid.getSetpoint().velocity));
        this.targetPosition = targetPosition;
    }

    public void setResetSimState() {
        intakePivotSim.setState(0.0, 0.0);
    }
}