package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.ClimberConstants;
import frc.robot.RobotConstants.ClimberConstants.ClimberGainsClass;


public class ClimberIOReal implements ClimberIO {
    private final TalonFX motor = new TalonFX(RobotConstants.ClimberConstants.CLIMBER_MOTOR_ID,
            RobotConstants.CANIVORE_CAN_BUS_NAME);

    private final StatusSignal<Angle> currentPositionRot = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityRotPerSec = motor.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = motor.getSupplyVoltage();
    private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
    private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
    private final StatusSignal<Temperature> tempCelsius = motor.getDeviceTemp();
    private final MotionMagicConfigs motionMagicConfigs;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);
    private final double CLIMBER_RATIO = RobotConstants.ClimberConstants.CLIMBER_RATIO;
    private double targetPositionDeg = 0.0;

    public ClimberIOReal() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.withSlot0(new Slot0Configs()
                .withKP(ClimberGainsClass.CLIMBER_KP.get())
                .withKI(ClimberGainsClass.CLIMBER_KI.get())
                .withKD(ClimberGainsClass.CLIMBER_KD.get())
                .withKA(ClimberGainsClass.CLIMBER_KA.get())
                .withKV(ClimberGainsClass.CLIMBER_KV.get())
                .withKS(ClimberGainsClass.CLIMBER_KS.get()));

        motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = ClimberConstants.CLIMBER_CRUISE_VELOCITY.get();
        motionMagicConfigs.MotionMagicAcceleration = ClimberConstants.CLIMBER_ACCELERATION.get();
        motionMagicConfigs.MotionMagicJerk = ClimberConstants.CLIMBER_JERK.get();
        config.withMotionMagic(motionMagicConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                velocityRotPerSec,
                tempCelsius,
                appliedVolts,
                supplyCurrentAmps,
                statorCurrentAmps,
                currentPositionRot);

        motor.getConfigurator().apply(config);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                currentPositionRot,
                velocityRotPerSec,
                tempCelsius,
                appliedVolts,
                supplyCurrentAmps,
                statorCurrentAmps);

        inputs.currentPositionDeg = currentPositionRot.getValueAsDouble() * 360 / CLIMBER_RATIO;
        inputs.velocityRotationsPerSec = velocityRotPerSec.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValue().in(Units.Celsius);
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.targetPositionDeg = targetPositionDeg;
        if (RobotConstants.TUNING) {
            inputs.ClimberKP = ClimberGainsClass.CLIMBER_KP.get();
            inputs.ClimberKI = ClimberGainsClass.CLIMBER_KI.get();
            inputs.ClimberKD = ClimberGainsClass.CLIMBER_KD.get();
            inputs.ClimberKA = ClimberGainsClass.CLIMBER_KA.get();
            inputs.ClimberKV = ClimberGainsClass.CLIMBER_KV.get();
            inputs.ClimberKS = ClimberGainsClass.CLIMBER_KS.get();
            motor.getConfigurator().apply(new Slot0Configs()
                    .withKP(inputs.ClimberKP)
                    .withKI(inputs.ClimberKI)
                    .withKD(inputs.ClimberKD)
                    .withKA(inputs.ClimberKA)
                    .withKV(inputs.ClimberKV)
                    .withKS(inputs.ClimberKS));
            motionMagicConfigs.MotionMagicCruiseVelocity = ClimberConstants.CLIMBER_CRUISE_VELOCITY.get();
            motionMagicConfigs.MotionMagicAcceleration = ClimberConstants.CLIMBER_ACCELERATION.get();
            motionMagicConfigs.MotionMagicJerk = ClimberConstants.CLIMBER_JERK.get();
            motor.getConfigurator().apply(motionMagicConfigs);
        }
    }

    @Override
    public void setTargetPosition(double targetPositionDeg) {
        motor.setControl(positionRequest.withPosition(targetPositionDeg * CLIMBER_RATIO / 360));
        this.targetPositionDeg = targetPositionDeg;
    }

    @Override
    public void resetPosition() {
        motor.setPosition(0.0);
    }

    @Override
    public void setCoast() {
        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void setBrake() {
        System.out.println(motor.setNeutralMode(NeutralModeValue.Brake));
    }
}
