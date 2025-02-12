package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import static frc.robot.RobotConstants.intakeConstants.intakePivotGainsClass.*;

public interface IntakePivotIO {
    @AutoLog
    class IntakePivotIOInputs {
        public double targetPositionDeg = 0.0;
        public double currentPositionDeg = 0.0;
        public double velocityRotPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;

        public double intakePivotKP = INTAKE_PIVOT_KP.get();
        public double intakePivotKI = INTAKE_PIVOT_KI.get();
        public double intakePivotKD = INTAKE_PIVOT_KD.get();
        public double intakePivotKA = INTAKE_PIVOT_KA.get();
        public double intakePivotKV = INTAKE_PIVOT_KV.get();
        public double intakePivotKS = INTAKE_PIVOT_KS.get();
    }

    default void updateInputs(IntakePivotIOInputs inputs) {}

    default void setMotorVoltage(double voltage) {}

    default void setMotorPosition(double targetPositionDeg) {}

    default void resetPosition() {}
}