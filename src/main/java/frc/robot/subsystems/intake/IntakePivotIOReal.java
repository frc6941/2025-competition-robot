package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class IntakePivotIOReal implements IntakePivotIO {
    private final TalonFX motor = new TalonFX(RobotConstants.intakeConstants.INTAKE_PIVOT_MOTOR_ID,
            RobotConstants.CAN_BUS_NAME);

    private final StatusSignal<AngularVelocity> velocityRotPerSec = motor.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = motor.getSupplyVoltage();
    private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
    private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
    private final StatusSignal<Temperature> tempCelsius = motor.getDeviceTemp();
    private final StatusSignal<Angle> currentPositionRot = motor.getPosition();

    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);

    double targetPositionDeg = 0.0;
    double PIVOT_RATIO = RobotConstants.intakeConstants.PIVOT_RATIO;

    public IntakePivotIOReal() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = PIVOT_RATIO;

        motor.getConfigurator().apply(config);
        motor.optimizeBusUtilization();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                velocityRotPerSec,
                tempCelsius,
                appliedVolts,
                supplyCurrentAmps,
                statorCurrentAmps,
                currentPositionRot);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                velocityRotPerSec,
                tempCelsius,
                appliedVolts,
                supplyCurrentAmps,
                statorCurrentAmps,
                currentPositionRot);

        inputs.velocityRotPerSec = velocityRotPerSec.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValue().in(Units.Celsius);
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.currentPositionDeg = currentPositionRot.getValueAsDouble()* 360 / PIVOT_RATIO;
        inputs.targetPositionDeg = targetPositionDeg;

        motor.getConfigurator().apply(new Slot0Configs()
                .withKP(inputs.intakePivotKP)
                .withKI(inputs.intakePivotKI)
                .withKD(inputs.intakePivotKD)
                .withKA(inputs.intakePivotKA)
                .withKV(inputs.intakePivotKV)
                .withKS(inputs.intakePivotKS));
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setMotorPosition(double targetPositionDeg) {
        this.targetPositionDeg = targetPositionDeg;
        motor.setControl(motionMagic.withPosition(targetPositionDeg * PIVOT_RATIO / 360));
    }

    @Override
    public void resetPosition() {
        motor.setPosition(0.0);
    }
}