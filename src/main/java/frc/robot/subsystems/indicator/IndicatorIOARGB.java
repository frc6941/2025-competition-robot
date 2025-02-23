package frc.robot.subsystems.indicator;

import frc.robot.drivers.led.AddressableLEDWrapper;
import frc.robot.drivers.led.patterns.SolidColorPattern;

import static frc.robot.RobotConstants.IndicatorConstants.LED_BUFFER_LENGTH;
import static frc.robot.RobotConstants.IndicatorConstants.LED_PORT;

public class IndicatorIOARGB implements IndicatorIO {
    private final AddressableLEDWrapper led;
    private Patterns currentPattern = Patterns.NORMAL;

    public IndicatorIOARGB() {
        led = new AddressableLEDWrapper(
                LED_PORT,
                LED_BUFFER_LENGTH
        );
        led.setIntensity(1.0);
        led.start(0.02);
    }

    @Override
    public void updateInputs(IndicatorIOInputs inputs) {
        inputs.currentPattern = currentPattern;
    }

    @Override
    public void setPattern(Patterns pattern) {
        currentPattern = pattern;
        if (pattern == Patterns.NORMAL) {
            led.setPattern(new SolidColorPattern(allianceColor()));
            return;
        }
        led.setPattern(pattern.pattern);
    }

    @Override
    public void reset() {
        led.stop();
        led.start(0.02);
    }
}
