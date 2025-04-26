package frc.robot.subsystems.beambreak;

import org.littletonrobotics.junction.AutoLog;

public interface BeambreakIO {
    default void updateInputs(BeambreakIOInputs inputs) {
    }

    @AutoLog
    class BeambreakIOInputs {
        public boolean isBeambreakOn = false;
        public double voltage = 0.0;
    }
}
