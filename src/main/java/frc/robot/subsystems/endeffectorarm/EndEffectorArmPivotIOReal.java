package frc.robot.subsystems.endeffectorarm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.RobotConstants;

import static frc.robot.RobotConstants.EndEffectorArmConstants.*;

public class EndEffectorArmPivotIOReal implements EndEffectorArmPivotIO {
    private final TalonFX motor = new TalonFX(RobotConstants.EndEffectorArmConstants.END_EFFECTOR_ARM_PIVOT_MOTOR_ID,
            RobotConstants.CANIVORE_CAN_BUS_NAME);
    private final CANcoder CANcoder = new CANcoder(END_EFFECTOR_ARM_ENCODER_ID,RobotConstants.CANIVORE_CAN_BUS_NAME);
    private final StatusSignal<AngularVelocity> velocityRotPerSec = motor.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = motor.getSupplyVoltage();
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
    private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
    private final StatusSignal<Temperature> tempCelsius = motor.getDeviceTemp();
    private final StatusSignal<Angle> currentPositionRot = motor.getPosition();

    double targetAngleDeg = 0.0;

    public EndEffectorArmPivotIOReal() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.CurrentLimits.SupplyCurrentLimit = 100.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 100.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        //initialize CANcoder
        CANcoderConfiguration CANconfig = new CANcoderConfiguration();
        CANconfig.MagnetSensor.MagnetOffset = END_EFFECTOR_ARM_ENCODER_OFFSET;
        CANconfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CANcoder.getConfigurator().apply(CANconfig);
        //integrate with fused encoder
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = END_EFFECTOR_ARM_ENCODER_ID;
        config.Feedback.RotorToSensorRatio = ROTOR_SENSOR_RATIO;

        config.withSlot0(new Slot0Configs()
                .withKP(RobotConstants.EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KP.get())
                .withKI(RobotConstants.EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KI.get())
                .withKD(RobotConstants.EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KD.get())
                .withKA(RobotConstants.EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KA.get())
                .withKV(RobotConstants.EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KV.get())
                .withKS(RobotConstants.EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KS.get())
                .withKG(RobotConstants.EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KG.get())
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
        );

        motor.getConfigurator().apply(config);

        motor.clearStickyFaults();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                velocityRotPerSec,
                tempCelsius,
                appliedVolts,
                motorVolts,
                supplyCurrentAmps,
                statorCurrentAmps,
                currentPositionRot);

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(EndEffectorArmPivotIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                velocityRotPerSec,
                tempCelsius,
                appliedVolts,
                motorVolts,
                supplyCurrentAmps,
                statorCurrentAmps,
                currentPositionRot);

        inputs.velocityRotPerSec = velocityRotPerSec.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValue().in(Units.Celsius);
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.motorVolts = motorVolts.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.currentAngleDeg = talonPosToAngle(currentPositionRot.getValueAsDouble()) - 63;
        inputs.targetAngleDeg = targetAngleDeg;

        if (RobotConstants.TUNING) {
            inputs.endEffectorArmPivotKP = EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KP.get();
            inputs.endEffectorArmPivotKI = EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KI.get();
            inputs.endEffectorArmPivotKD = EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KD.get();
            inputs.endEffectorArmPivotKA = EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KA.get();
            inputs.endEffectorArmPivotKV = EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KV.get();
            inputs.endEffectorArmPivotKS = EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KS.get();
            inputs.endEffectorArmPivotKG = EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KG.get();

            updateGains(inputs.endEffectorArmPivotKP, inputs.endEffectorArmPivotKI, inputs.endEffectorArmPivotKD, inputs.endEffectorArmPivotKA, inputs.endEffectorArmPivotKV, inputs.endEffectorArmPivotKS, inputs.endEffectorArmPivotKG);
        }
    }

    @Override
    public void updateGains(double kP, double kI, double kD, double kA, double kV, double kS, double kG) {
        motor.getConfigurator().apply(new Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKA(kA)
                .withKV(kV)
                .withKS(kS)
                .withKG(kG)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withGravityType(GravityTypeValue.Arm_Cosine));
    }

    @Override
    public void setPivotAngle(double targetAngleDeg) {
        this.targetAngleDeg = targetAngleDeg;
        motor.setControl(new PositionDutyCycle(angleToTalonPos(targetAngleDeg + 63)).withEnableFOC(true));
    }

    private double angleToTalonPos(double angleDeg) {
        return (angleDeg / 360) * 1;
    }

    private double talonPosToAngle(double rotations) {
        return rotations * 360 / 1;
    }
}
