package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double motionMagicVelocityTarget = 0.0;
        public double motionMagicPositionTarget = 0.0;
        public double setpointMeters = 0.0;
        public double[] appliedVolts = new double[] {}; // {leader, follower}
        public double[] statorCurrentAmps = new double[] {}; // {leader, follower}
        public double[] supplyCurrentAmps = new double[] {}; // {leader, follower}
        public double[] tempCelsius = new double[] {}; // {leader, follower}
        // public double acceleration = 0.0; may add later
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setVoltage(double voltage) {}

    public default void setPosition(double meters) {}

    public default void stop() {
        setVoltage(0.0);
    }

    public default double getLeaderCurrent(){
        return getLeaderCurrent();
    }

    public default void resetEncoder(double position) {}

    public default void resetEncoder() {
        resetEncoder(0.0);
    }

    public default void resetPosition() {}
}
