package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO;
import static frc.robot.RobotConstants.ElevatorConstants.ELEVATOR_SPOOL_DIAMETER;

public class ElevatorIOSim implements ElevatorIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    private final DCMotorSim leftElevatorTalonSim = new DCMotorSim(edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1),
            0.025, 6.75), DCMotor.getFalcon500(1), null);

    private Measure<VoltageUnit> leftElevatorAppliedVoltage = Volts.zero();

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        leftElevatorTalonSim.update(LOOP_PERIOD_SECS);

        inputs.leftElevatorVelocity =
                RadiansPerSecond.of(leftElevatorTalonSim.getAngularVelocityRadPerSec());
        inputs.leftElevatorPosition =
                Radians.of(leftElevatorTalonSim.getAngularPositionRad());
        inputs.leftElevatorAppliedVoltage =
                leftElevatorAppliedVoltage;
        inputs.leftElevatorSupplyCurrent =
                Amps.of(leftElevatorTalonSim.getCurrentDrawAmps());

        inputs.rightElevatorVelocity =
                RadiansPerSecond.of(leftElevatorTalonSim.getAngularVelocityRadPerSec());
        inputs.rightElevatorPosition =
                Radians.of(leftElevatorTalonSim.getAngularPositionRad());
        inputs.rightElevatorAppliedVoltage =
                leftElevatorAppliedVoltage;
        inputs.rightElevatorSupplyCurrent =
                Amps.of(leftElevatorTalonSim.getCurrentDrawAmps());
    }

    @Override
    public void setElevatorDirectVoltage(double volts) {
        leftElevatorAppliedVoltage = Volts.of(volts);
        leftElevatorTalonSim.setInputVoltage(volts);
    }

    @Override
    public void resetElevatorPosition() {
        leftElevatorTalonSim.setAngle(0);
    }

    @Override
    public boolean isNearExtension(double expected) {
        return MathUtil.isNear(metersToRotations(expected), leftElevatorTalonSim.getAngularPositionRad(), 0);
    }

    @Override
    public void setElevatorTarget(double radians) {
        // FIXME: Adding delays?
        leftElevatorTalonSim.setAngle(radians);
    }

    @Override
    public double getElevatorPosition() {
        return leftElevatorTalonSim.getAngularPositionRad();
    }

    @Override
    public boolean isNearZeroExtension() {
        return MathUtil.isNear(metersToRotations(0.05), leftElevatorTalonSim.getAngularPositionRad(), 0.3);
    }

    @Override
    public double getElevatorVelocity() {
        return leftElevatorTalonSim.getAngularVelocityRadPerSec() / 6.28 * 60;
    }

    private double metersToRotations(double heightMeters) {
        return (heightMeters / (Math.PI * ELEVATOR_SPOOL_DIAMETER)) * ELEVATOR_GEAR_RATIO;
    }

    private double rotationsToMeters(double rotations) {
        return rotations * (Math.PI * ELEVATOR_SPOOL_DIAMETER) / ELEVATOR_GEAR_RATIO;
    }
}