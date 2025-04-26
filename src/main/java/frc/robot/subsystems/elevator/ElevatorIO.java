package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    default void updateInputs(ElevatorIOInputs inputs) {
    }

    default void setElevatorVoltage(double volts) {
    }

    default void setElevatorTarget(double meters) {
    }

    default void resetElevatorPosition() {
    }

    default double getElevatorVelocity() {
        return 0.0;
    }

    default boolean isNearExtension(double expected) {
        return false;
    }

    default boolean isNearZeroExtension() {
        return false;
    }

    @AutoLog
    class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double setpointMeters = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

}