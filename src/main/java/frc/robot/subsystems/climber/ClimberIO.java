package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants.ClimberConstants;
import frc.robot.RobotConstants.ClimberConstants.ClimberGainsClass;
import frc.robot.subsystems.intake.IntakePivotIO.IntakePivotIOInputs;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public Rotation2d position = new Rotation2d();
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

    public default void updateInputs(ClimberIOInputs inputs) {
    }

    public default void setMotorVoltage(double voltage) {
    }

    public default void setMotorPosition(Rotation2d targetPosition) {
    }

    public default void updateConfigs(double kp, double ki, double kd, double ka, double kv, double ks) {
    }
}
