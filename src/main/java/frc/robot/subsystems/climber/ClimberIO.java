package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.RobotConstants.ClimberConstants.ClimberGainsClass;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public double currentPositionDeg = 0.0;
        public double targetPositionDeg = 0.0;
        public double velocityRotationsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;

        public double ClimberKP = ClimberGainsClass.CLIMBER_KP.get();
        public double ClimberKI = ClimberGainsClass.CLIMBER_KI.get();
        public double ClimberKD = ClimberGainsClass.CLIMBER_KD.get();
        public double ClimberKA = ClimberGainsClass.CLIMBER_KA.get();
        public double ClimberKV = ClimberGainsClass.CLIMBER_KV.get();
        public double ClimberKS = ClimberGainsClass.CLIMBER_KS.get();
    }

    default void updateInputs(ClimberIOInputs inputs) {}

    default void setTargetPosition(double targetPositionDeg) {}

    default void resetPosition() {}

    default void setCoast() {}

    default void setBrake() {}
}
