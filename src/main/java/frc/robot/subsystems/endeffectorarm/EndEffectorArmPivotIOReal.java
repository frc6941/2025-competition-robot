package frc.robot.subsystems.endeffectorarm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.RobotConstants;

import static frc.robot.RobotConstants.EndEffectorArmConstants.*;

public class EndEffectorArmPivotIOReal implements EndEffectorArmPivotIO {
    private final TalonFX motor = new TalonFX(RobotConstants.EndEffectorArmConstants.END_EFFECTOR_ARM_PIVOT_MOTOR_ID,
            RobotConstants.CANIVORE_CAN_BUS_NAME);
    private final CANcoder caNcoder = new CANcoder(END_EFFECTOR_ARM_ENCODER_ID,RobotConstants.CANIVORE_CAN_BUS_NAME);
    private final StatusSignal<AngularVelocity> velocityRotPerSec = motor.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = motor.getSupplyVoltage();
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
    private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
    private final StatusSignal<Temperature> tempCelsius = motor.getDeviceTemp();
    private final StatusSignal<Angle> currentPositionRot = motor.getPosition();


    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(false);
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);

    double targetAngleDeg = 0.0;

    public EndEffectorArmPivotIOReal() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;//TODO: set to the right direction
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.CurrentLimits.SupplyCurrentLimit = 100.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 100.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        //initialize CANcoder
        CANcoderConfiguration CANconfig = new CANcoderConfiguration();
        CANconfig.MagnetSensor.MagnetOffset = END_EFFECTOR_ARM_ENCODER_OFFSET;
        caNcoder.getConfigurator().apply(CANconfig);
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.FeedbackRemoteSensorID = END_EFFECTOR_ARM_ENCODER_ID;
        config.Feedback.RotorToSensorRatio = ROTOR_SENSOR_RATO;



        config.withSlot0(new Slot0Configs()
                .withKP(RobotConstants.EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KP.get())
                .withKI(RobotConstants.EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KI.get())
                .withKD(RobotConstants.EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KD.get())
                .withKA(RobotConstants.EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KA.get())
                .withKV(RobotConstants.EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KV.get())
                .withKS(RobotConstants.EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KS.get())
                .withKG(RobotConstants.EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KG.get())
        );


        motor.getConfigurator().apply(config);

        motor.clearStickyFaults();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
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
        inputs.currentAngleDeg = talonPosToAngle(currentPositionRot.getValueAsDouble());
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
                .withKG(kG));
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setPivotAngle(double targetAngleDeg) {
        this.targetAngleDeg = targetAngleDeg;
        motor.setControl(new PositionDutyCycle(angleToTalonPos(targetAngleDeg)));
    }

    private double angleToTalonPos(double angleDeg) {
        return (angleDeg / 360) * END_EFFECTOR_ARM_PIVOT_RATIO;
    }

    private double talonPosToAngle(double rotations) {
        return rotations * 360 / END_EFFECTOR_ARM_PIVOT_RATIO;
    }
}
