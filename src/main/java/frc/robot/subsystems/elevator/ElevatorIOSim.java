package frc.robot.subsystems.elevator;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.ElevatorGainsClass;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.RobotConstants.ElevatorConstants.*;

public class ElevatorIOSim implements ElevatorIO {
    public static final double carriaeMass = Units.lbsToKilograms(7.0 + (3.25 / 2));
    private static final DCMotor elevatorTalonsim = DCMotor.getKrakenX60Foc(2).withReduction(ELEVATOR_GEAR_RATIO);
    public static final Matrix<N2, N2> A =
            MatBuilder.fill(
                    Nat.N2(),
                    Nat.N2(),
                    0,
                    1,
                    0,
                    -elevatorTalonsim.KtNMPerAmp
                            / (elevatorTalonsim.rOhms
                            * Math.pow(ELEVATOR_SPOOL_DIAMETER / 2, 2)
                            * (carriaeMass)
                            * elevatorTalonsim.KvRadPerSecPerVolt));

    public static final Vector<N2> B =
            VecBuilder.fill(
                    0.0, elevatorTalonsim.KtNMPerAmp / ((ELEVATOR_SPOOL_DIAMETER / 2) * carriaeMass));
    private final ProfiledPIDController controller =
            new ProfiledPIDController(
                    ElevatorGainsClass.ELEVATOR_KP.get(),
                    ElevatorGainsClass.ELEVATOR_KI.get(),
                    ElevatorGainsClass.ELEVATOR_KD.get(),
                    new Constraints(100, 300));
    private final double feedforward = 5;
    private Measure<VoltageUnit> appliedVolts = Volts.zero();
    private double targetPositionMeters = 0.0;
    private Vector<N2> simState;
    private double inputTorqueCurrent = 0.0;

    public ElevatorIOSim() {
        //initial position
        simState = VecBuilder.fill(0.0, 0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        for (int i = 0; i < RobotConstants.LOOPER_DT / (1.0 / 1000.0); i++) {
            setInputTorqueCurrent(
                    controller.calculate(simState.get(0)) * 15 + feedforward);
            update(1.0 / 1000.0);
        }

        inputs.positionMeters = radToHeight(simState.get(0));
        inputs.velocityMetersPerSec = radToHeight(simState.get(1));
        inputs.setpointMeters = targetPositionMeters;
        inputs.statorCurrentAmps = Math.copySign(inputTorqueCurrent, appliedVolts.magnitude());
        inputs.supplyCurrentAmps = Math.copySign(inputTorqueCurrent, appliedVolts.magnitude());
    }

    private void update(double dt) {
        inputTorqueCurrent =
                MathUtil.clamp(inputTorqueCurrent, -elevatorTalonsim.stallCurrentAmps, elevatorTalonsim.stallCurrentAmps);
        Matrix<N2, N1> updatedState =
                NumericalIntegration.rkdp(
                        (Matrix<N2, N1> x, Matrix<N1, N1> u) ->
                                A.times(x)
                                        .plus(B.times(u))
                                        .plus(
                                                VecBuilder.fill(
                                                        0.0,
                                                        -9.8)),
                        simState,
                        MatBuilder.fill(Nat.N1(), Nat.N1(), inputTorqueCurrent * 30),
                        dt);
        // Apply limits
        simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
        if (simState.get(0) <= 0.0) {
            simState.set(1, 0, 0.0);
            simState.set(0, 0, 0.0);
        }
        if (simState.get(0) >= heightToRad(MAX_EXTENSION_METERS.get())) {
            simState.set(1, 0, 0.0);
            simState.set(0, 0, heightToRad(MAX_EXTENSION_METERS.get()));
        }
    }

    private void setInputTorqueCurrent(double torqueCurrent) {
        inputTorqueCurrent = torqueCurrent;
        appliedVolts =
                Volts.of(elevatorTalonsim.getVoltage(
                        elevatorTalonsim.getTorque(inputTorqueCurrent),
                        simState.get(1, 0)));
        appliedVolts = Volts.of(MathUtil.clamp(appliedVolts.magnitude(), -12.0, 12.0));
    }

    @Override
    public void setElevatorVoltage(double volts) {

    }

    @Override
    public void resetElevatorPosition() {
        simState.set(0, 0, 0.0);
    }

    @Override
    public boolean isNearExtension(double expected) {
        return MathUtil.isNear(expected, radToHeight(simState.get(0)), 0.02);
    }

    @Override
    public void setElevatorTarget(double meters) {
        targetPositionMeters = meters;
        controller.setGoal(heightToRad(meters));
    }


    @Override
    public boolean isNearZeroExtension() {
        return MathUtil.isNear(0.05, radToHeight(simState.get(0)), 0.3);
    }

    @Override
    public double getElevatorVelocity() {
        return radToHeight(simState.get(1));
    }

    private double heightToRad(double heightMeters) {
        return heightMeters / (ELEVATOR_SPOOL_DIAMETER / 2);
    }

    private double radToHeight(double rad) {
        return rad * (ELEVATOR_SPOOL_DIAMETER / 2);
    }

}