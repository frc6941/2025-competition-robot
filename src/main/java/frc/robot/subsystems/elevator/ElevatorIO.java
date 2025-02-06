package frc.robot.subsystems.elevator;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ElevatorIO {
    void updateInputs(ElevatorIOInputs inputs);

    void setElevatorDirectVoltage(double volts);
    void setElevatorTarget(double meters);
    void resetElevatorPosition();
    void zeroingElevator();
    double getElevatorVelocity();
    double getElevatorPosition();
    boolean isNearExtension(double expected);
    boolean isCurrentMax(double expected);

    @AutoLog
    class ElevatorIOInputs {
        public Measure<AngularVelocityUnit> leftElevatorVelocity = RadiansPerSecond.zero();
        public Measure<AngleUnit> leftElevatorPosition = Radians.zero();
        public Measure<VoltageUnit> leftElevatorAppliedVoltage = Volts.zero();
        public Measure<CurrentUnit> leftElevatorSupplyCurrent = Amps.zero();

        public Measure<AngularVelocityUnit> rightElevatorVelocity = RadiansPerSecond.zero();
        public Measure<AngleUnit> rightElevatorPosition = Radians.zero();
        public Measure<VoltageUnit> rightElevatorAppliedVoltage = Volts.zero();
        public Measure<CurrentUnit> rightElevatorSupplyCurrent = Amps.zero();

        public Measure<AngularVelocityUnit> targetElevatorVelocity = RadiansPerSecond.zero();

        public double ElevatorKP = frc.robot.RobotConstants.ElevatorConstants.ElevatorGainsClass.ELEVATOR_KP.get();
        public double ElevatorKI = frc.robot.RobotConstants.ElevatorConstants.ElevatorGainsClass.ELEVATOR_KI.get();
        public double ElevatorKD = frc.robot.RobotConstants.ElevatorConstants.ElevatorGainsClass.ELEVATOR_KD.get();
        public double ElevatorKA = frc.robot.RobotConstants.ElevatorConstants.ElevatorGainsClass.ELEVATOR_KA.get();
        public double ElevatorKV = frc.robot.RobotConstants.ElevatorConstants.ElevatorGainsClass.ELEVATOR_KV.get();
        public double ElevatorKS = frc.robot.RobotConstants.ElevatorConstants.ElevatorGainsClass.ELEVATOR_KS.get();
    }
}
