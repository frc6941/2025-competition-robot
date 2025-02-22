package frc.robot.subsystems.indicator;

public class IndicatorIOSim implements IndicatorIO {
    private Patterns currentPattern = Patterns.NORMAL;

    @Override
    public void updateInputs(IndicatorIOInputs inputs) {
        inputs.currentPattern = currentPattern;
    }

    @Override
    public void setPattern(Patterns pattern) {
        currentPattern = pattern;
    }

    @Override
    public void reset() {
    }
}
