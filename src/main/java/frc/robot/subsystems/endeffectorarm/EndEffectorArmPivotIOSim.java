package frc.robot.subsystems.endeffectorarm;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.RobotConstants;

import static edu.wpi.first.units.Units.Volts;

public class EndEffectorArmPivotIOSim implements EndEffectorArmPivotIO {
    // Default values - adjust according to your specific mechanism
    private static final double moi = 1.0; // Moment of inertia
    private static final double cgRadius = 0.2; // Center of gravity radius
    private static final double PIVOT_RATIO = 50.0; // Default gear ratio

    private static final DCMotor gearbox =
            DCMotor.getKrakenX60Foc(1).withReduction(PIVOT_RATIO);

    private static final Matrix<N2, N2> A =
            MatBuilder.fill(
                    Nat.N2(),
                    Nat.N2(),
                    0,
                    1,
                    0,
                    -gearbox.KtNMPerAmp
                            / (gearbox.KvRadPerSecPerVolt
                            * gearbox.rOhms * moi));
    private static final Vector<N2> B = VecBuilder.fill(0, gearbox.KtNMPerAmp / moi);

    private final PIDController controller = new PIDController(100, 0.0, 0.0);
    // Simulation state
    private Vector<N2> simState;
    private Measure<VoltageUnit> appliedVolts = Volts.zero();
    private double inputTorqueCurrent = 0.0;
    private double setpointDegrees = 0.0;

    public EndEffectorArmPivotIOSim() {
        // Initial position (90 degrees)
        simState = VecBuilder.fill(Math.PI / 2.0, 0.0);
    }

    @Override
    public void updateInputs(EndEffectorArmPivotIOInputs inputs) {
        for (int i = 0; i < RobotConstants.LOOPER_DT / (1.0 / 1000.0); i++) {
            setInputTorqueCurrent(
                    controller.calculate(simState.get(0)));
            update(1.0 / 1000.0);
        }
        inputs.targetAngleDeg = setpointDegrees;
        inputs.currentAngleDeg = Math.toDegrees(simState.get(0));
        inputs.velocityRotPerSec = Math.toDegrees(simState.get(1));
        inputs.appliedVolts = appliedVolts.magnitude();
        inputs.statorCurrentAmps = Math.copySign(inputTorqueCurrent, appliedVolts.magnitude());
        inputs.supplyCurrentAmps = Math.copySign(inputTorqueCurrent, appliedVolts.magnitude());
        inputs.tempCelsius = 20.0; // Default room temperature
    }

    @Override
    public void setPivotAngle(double targetAngleDeg) {
        this.setpointDegrees = targetAngleDeg;
        controller.setSetpoint(Math.toRadians(targetAngleDeg));
    }

    @Override
    public void resetAngle(double resetAngleDeg) {
        // Reset simulation angle
        simState = VecBuilder.fill(Math.toRadians(resetAngleDeg), 0.0);
    }

    private void setInputTorqueCurrent(double torqueCurrent) {
        inputTorqueCurrent = torqueCurrent;
        appliedVolts =
                Volts.of(gearbox.getVoltage(
                        gearbox.getTorque(inputTorqueCurrent),
                        simState.get(1, 0)));
        appliedVolts = Volts.of(MathUtil.clamp(appliedVolts.magnitude(), -12.0, 12.0));
    }

    private void update(double dt) {
        inputTorqueCurrent =
                MathUtil.clamp(inputTorqueCurrent, -gearbox.stallCurrentAmps, gearbox.stallCurrentAmps);

        Matrix<N2, N1> updatedState =
                NumericalIntegration.rkdp(
                        (Matrix<N2, N1> x, Matrix<N1, N1> u) ->
                                A.times(x)
                                        .plus(B.times(u)),
                        simState,
                        VecBuilder.fill(inputTorqueCurrent * 5),
                        dt);
        simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    }
} 