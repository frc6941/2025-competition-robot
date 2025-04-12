package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.RobotConstants;

import static frc.robot.RobotConstants.IntakeConstants.*;

public class IntakePivotIOReal implements IntakePivotIO {
    private final TalonFX motor = new TalonFX(RobotConstants.IntakeConstants.INTAKE_PIVOT_MOTOR_ID,
            RobotConstants.CANIVORE_CAN_BUS_NAME);

    private final StatusSignal<AngularVelocity> velocityRotPerSec = motor.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = motor.getSupplyVoltage();
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
    private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
    private final StatusSignal<Temperature> tempCelsius = motor.getDeviceTemp();
    private final StatusSignal<Angle> currentPositionRot = motor.getPosition();

    private final CANcoder canCoder = new CANcoder(INTAKE_PIVOT_ENCODER_ID, RobotConstants.CANIVORE_CAN_BUS_NAME);


    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(false);
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);
    private final MotionMagicConfigs motionMagicConfigs;

    double targetAngleDeg = 0.0;

    public IntakePivotIOReal() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Initialize CANcoder
        CANcoderConfiguration CANconfig = new CANcoderConfiguration();
        CANconfig.MagnetSensor.MagnetOffset = INTAKE_PIVOT_ENCODER_OFFSET;
        CANconfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoder.getConfigurator().apply(CANconfig);
        // Try the fused CANcoder option, if it doesn't work, use the remote
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = INTAKE_PIVOT_ENCODER_ID;
        config.Feedback.RotorToSensorRatio = INTAKE_PIVOT_ROTOR_ENCODER_RATIO;

        config.withSlot0(new Slot0Configs()
                .withKP(IntakePivotGainsClass.INTAKE_PIVOT_KP.get())
                .withKI(IntakePivotGainsClass.INTAKE_PIVOT_KI.get())
                .withKD(IntakePivotGainsClass.INTAKE_PIVOT_KD.get()));

        motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = INTAKE_PIVOT_CRUISE_VELOCITY.get();
        motionMagicConfigs.MotionMagicAcceleration = INTAKE_PIVOT_ACCELERATION.get();
        motionMagicConfigs.MotionMagicJerk = INTAKE_PIVOT_JERK.get();
        config.withMotionMagic(motionMagicConfigs);

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
    public void updateInputs(IntakePivotIOInputs inputs) {
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
        inputs.currentAngleDeg = encoderPosToAngle(currentPositionRot.getValueAsDouble());
        inputs.targetAngleDeg = targetAngleDeg;
        inputs.motorVolts = motorVolts.getValueAsDouble();

        if (RobotConstants.TUNING) {
            inputs.intakePivotKP = IntakePivotGainsClass.INTAKE_PIVOT_KP.get();
            inputs.intakePivotKI = IntakePivotGainsClass.INTAKE_PIVOT_KI.get();
            inputs.intakePivotKD = IntakePivotGainsClass.INTAKE_PIVOT_KD.get();

            motor.getConfigurator().apply(new Slot0Configs()
                    .withKP(inputs.intakePivotKP)
                    .withKI(inputs.intakePivotKI)
                    .withKD(inputs.intakePivotKD));
            motionMagicConfigs.MotionMagicCruiseVelocity = INTAKE_PIVOT_CRUISE_VELOCITY.get();
            motionMagicConfigs.MotionMagicAcceleration = INTAKE_PIVOT_ACCELERATION.get();
            motionMagicConfigs.MotionMagicJerk = INTAKE_PIVOT_JERK.get();
            motor.getConfigurator().apply(motionMagicConfigs);
        }
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setPivotAngle(double targetAngleDeg) {
        this.targetAngleDeg = targetAngleDeg;
        motor.setControl(motionMagic.withPosition(angleToEncoderPos(targetAngleDeg)).withEnableFOC(true));
    }

    @Override
    public void resetAngle(double resetAngleDeg) {
        motor.setPosition(resetAngleDeg / 360);
    }

    private double angleToEncoderPos(double angleDeg) {
        return angleDeg / 360;
    }

    private double encoderPosToAngle(double rotations) {
        return rotations * 360;
    }

    @Override
    public boolean isNearAngle(double targetAngleDeg, double toleranceDeg) {
        return Math.abs(currentPositionRot.getValueAsDouble() - angleToEncoderPos(targetAngleDeg)) <= angleToEncoderPos(toleranceDeg);
    }
}
