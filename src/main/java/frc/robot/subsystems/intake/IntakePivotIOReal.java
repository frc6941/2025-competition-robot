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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class IntakePivotIOReal implements IntakePivotIO {
    private final TalonFX motor = new TalonFX(RobotConstants.intakeConstants.INTAKER_PIVOT_MOTOR_ID,
            RobotConstants.CAN_BUS_NAME);

    private final StatusSignal<AngularVelocity> velocityRotationsPerSec = motor.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = motor.getSupplyVoltage();
    private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
    private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
    private final StatusSignal<Temperature> tempCelsius = motor.getDeviceTemp();
    private final StatusSignal<Angle> motorPositionRotations = motor.getPosition();

    private final Slot0Configs slot0Configs = new Slot0Configs();

    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);

    public IntakePivotIOReal() {
        var config = new TalonFXConfiguration();
        // TODO check configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = RobotConstants.intakeConstants.PIVOT_RATIO;

        motor.getConfigurator().apply(config);
        motor.optimizeBusUtilization();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                velocityRotationsPerSec,
                tempCelsius,
                appliedVolts,
                supplyCurrentAmps,
                statorCurrentAmps,
                motorPositionRotations);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                velocityRotationsPerSec,
                tempCelsius,
                appliedVolts,
                supplyCurrentAmps,
                statorCurrentAmps,
                motorPositionRotations);

        inputs.velocityRotationsPerSec = velocityRotationsPerSec.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValue().in(Units.Celsius);
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.position = Rotation2d.fromRotations(motorPositionRotations.getValueAsDouble());

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
    public void setMotorPosition(Rotation2d targetPosition) {
        motor.setControl(motionMagic.withPosition(targetPosition.getRotations()));
    }
}