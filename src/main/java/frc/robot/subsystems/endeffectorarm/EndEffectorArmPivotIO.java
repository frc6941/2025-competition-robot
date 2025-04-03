package frc.robot.subsystems.endeffectorarm;

import org.littletonrobotics.junction.AutoLog;

import static frc.robot.RobotConstants.EndEffectorArmConstants.EndEffectorArmPivotGainsClass.*;

public interface EndEffectorArmPivotIO {
    default void updateInputs(EndEffectorArmPivotIOInputs inputs) {
    }

    default void setPivotAngle(double targetAngleDeg) {
    }

    default void resetAngle(double resetAngleDeg) {
    }

    default void updateGains(double kP, double kI, double kD, double kA, double kV, double kS, double kG) {
    }

    @AutoLog
    class EndEffectorArmPivotIOInputs {
        public double targetAngleDeg = 0.0;
        public double currentAngleDeg = 0.0;
        public double velocityRotPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double motorVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;

        public double endEffectorArmPivotKP = END_EFFECTOR_ARM_PIVOT_KP.get();
        public double endEffectorArmPivotKI = END_EFFECTOR_ARM_PIVOT_KI.get();
        public double endEffectorArmPivotKD = END_EFFECTOR_ARM_PIVOT_KD.get();
        public double endEffectorArmPivotKA = END_EFFECTOR_ARM_PIVOT_KA.get();
        public double endEffectorArmPivotKV = END_EFFECTOR_ARM_PIVOT_KV.get();
        public double endEffectorArmPivotKS = END_EFFECTOR_ARM_PIVOT_KS.get();
        public double endEffectorArmPivotKG = END_EFFECTOR_ARM_PIVOT_KG.get();
    }
} 