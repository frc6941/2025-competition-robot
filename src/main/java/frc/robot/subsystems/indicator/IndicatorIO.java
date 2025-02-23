package frc.robot.subsystems.indicator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.drivers.led.AddressableLEDPattern;
import frc.robot.drivers.led.patterns.BlinkingPattern;
import frc.robot.drivers.led.patterns.RainbowPattern;
import frc.robot.drivers.led.patterns.SolidColorPattern;
import org.littletonrobotics.junction.AutoLog;

;

public interface IndicatorIO {
    /**
     * All available patterns.
     */
    enum Patterns {
        NORMAL(new SolidColorPattern(Color.kBlue)),
        INTAKING(new BlinkingPattern(Color.kGreen,0.2)),
        INTAKED(new BlinkingPattern(Color.kAqua,0.1)),
        HOLDING_CORAL(new BlinkingPattern(Color.kCyan, 0.1)),
        POKING(new SolidColorPattern(Color.kCyan)),
        SHOOTING(new RainbowPattern()),
        SHOOT_FINISH(new BlinkingPattern(Color.kBlue, 0.1)),
        CAN_CLIMB(new BlinkingPattern(Color.kPurple, 0.2)),
        CLIMBING(new BlinkingPattern(Color.kBeige, 0.7)),
        AIMING(new BlinkingPattern(Color.kBlanchedAlmond,0.2)),
        RESET_ODOM(new BlinkingPattern(Color.kWhite, 0.25));

        public final AddressableLEDPattern pattern;

        Patterns(AddressableLEDPattern color) {
            this.pattern = color;
        }
    }

    /**
     * Returns alliance color.
     *
     * @return Current alliance color
     */
    default Color allianceColor() {
    //    return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
    //        case Blue -> Color.kBlue;
    //        case Red -> Color.kRed;
    //    };
        return Color.kBlue;
    }

    @AutoLog
    class IndicatorIOInputs {
        public Patterns currentPattern;
    }

    /**
     * Updates the set of loggable inputs.
     */
    void updateInputs(IndicatorIOInputs inputs);

    /**
     * Set current pattern.
     */
    void setPattern(Patterns pattern);

    /**
     * Stops and starts indicator.
     */
    void reset();
}
