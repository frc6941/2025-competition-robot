package frc.robot.subsystems.indicator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class IndicatorSubsystem extends SubsystemBase {
    private final IndicatorIO io;
    private final IndicatorIOInputsAutoLogged inputs = new IndicatorIOInputsAutoLogged();
    private final Timer timer = new Timer();
    private IndicatorIO.Patterns currentPattern = IndicatorIO.Patterns.NORMAL;
    @Getter
    private IndicatorIO.Patterns lastPattern = IndicatorIO.Patterns.NORMAL;

    public IndicatorSubsystem(IndicatorIO io) {
        this.io = io;
    }

    public void setPattern(IndicatorIO.Patterns pattern) {
        if (pattern == currentPattern) {
            io.setPattern(currentPattern);
            return;
        }
        lastPattern = currentPattern;
        currentPattern = pattern;
        io.setPattern(pattern);
        switch (pattern) {
            case AFTER_INTAKE, RESET_ODOM, AIMED -> timer.restart();
            default -> {
            }
        }
    }

    @Override
    public void periodic() {
        switch (currentPattern) {
            case AFTER_INTAKE, RESET_ODOM, AIMED -> resetLed();
            default -> {
            }
        }
        io.updateInputs(inputs);
        Logger.processInputs("Indicator", inputs);
    }

    private void resetLed() {
        if (!timer.hasElapsed(2)) return;
        setPattern(IndicatorIO.Patterns.NORMAL);
    }

    public void reset() {
        this.io.reset();
    }

    public void resetToLastPattern() {
        setPattern(lastPattern);
    }
}
