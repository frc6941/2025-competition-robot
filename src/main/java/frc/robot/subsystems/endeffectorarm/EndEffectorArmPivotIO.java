package frc.robot.subsystems.endeffectorarm;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorArmPivotIO {
    default void updateInputs(EndEffectorArmPivotIOInputs inputs) {
    }

    default void setMotorVoltage(double voltage) {
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

            // PID constants will be added later when we set up RobotConstants
            public double endEffectorArmPivotKP = 2.5;
            public double endEffectorArmPivotKI = 0.0;
            public double endEffectorArmPivotKD = 0.0;
            public double endEffectorArmPivotKA = 0.0;
            public double endEffectorArmPivotKV = 0.0;
            public double endEffectorArmPivotKS = 0.0;
            public double endEffectorArmPivotKG = 0.0;
        }
} 