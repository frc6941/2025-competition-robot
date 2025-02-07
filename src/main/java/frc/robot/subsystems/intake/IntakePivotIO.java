package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.RobotConstants.intakeConstants.intakePivotGainsClass.*;

public interface IntakePivotIO {
    @AutoLog
    public static class IntakePivotIOInputs {
        public Rotation2d position = new Rotation2d();
        public double velocityRotationsPerSec = 0.0;
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

    public default void updateInputs(IntakePivotIOInputs inputs) {
    }

    public default void setMotorVoltage(double voltage) {
    }

    public default void setMotorPosition(Rotation2d targetPosition) {
    }

    public default void updateConfigs(double kp, double ki, double kd, double ka, double kv, double ks) {
    }

}