package frc.robot.subsystems.roller;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class RollerIOReal implements RollerIO {
    private final TalonFX motor;
    private final Slot0Configs slot0Configs = new Slot0Configs();

    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

    private final StatusSignal<AngularVelocity> velocityRotPerSec;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> statorCurrentAmps;
    private final StatusSignal<Current> supplyCurrentAmps;
    private final StatusSignal<Temperature> tempCelsius;

    public RollerIOReal(int id, String canbus, int statorCurrentLimitAmps, int supplyCurrentLimitAmps, boolean invert, boolean brake) {
        this.motor = new TalonFX(id, canbus);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimitAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = statorCurrentLimitAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);

        motor.clearStickyFaults();

        velocityRotPerSec = motor.getVelocity();
        appliedVolts = motor.getMotorVoltage();
        statorCurrentAmps = motor.getStatorCurrent();
        supplyCurrentAmps = motor.getSupplyCurrent();
        tempCelsius = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocityRotPerSec, appliedVolts, statorCurrentAmps,
                supplyCurrentAmps, tempCelsius);

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        BaseStatusSignal.refreshAll(velocityRotPerSec, appliedVolts, statorCurrentAmps, supplyCurrentAmps, tempCelsius);
        inputs.velocityRotPerSec = velocityRotPerSec.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
    }

    @Override
    public void updateConfigs(double kp, double ki, double kd, double ka, double kv, double ks) {
        slot0Configs.withKP(kp);
        slot0Configs.withKI(ki);
        slot0Configs.withKD(kd);
        slot0Configs.withKA(ka);
        slot0Configs.withKV(kv);
        slot0Configs.withKS(ks);
        motor.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void setVelocity(double velocityRPS) {
        motor.setControl(velocityVoltage.withVelocity(velocityRPS));
    }
}
