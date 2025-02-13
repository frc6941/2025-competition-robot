package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.ElevatorConstants.*;


public class ElevatorIOReal implements ElevatorIO {
    private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0.0).withEnableFOC(true);

    private final TalonFX leftElevatorTalon = new TalonFX(LEFT_ELEVATOR_MOTOR_ID, RobotConstants.CANIVORE_CAN_BUS_NAME);
    private final TalonFX rightElevatorTalon = new TalonFX(RIGHT_ELEVATOR_MOTOR_ID, RobotConstants.CANIVORE_CAN_BUS_NAME);

    private final StatusSignal<AngularVelocity> leftElevatorVelocity = leftElevatorTalon.getVelocity();
    private final StatusSignal<Angle> leftElevatorPosition = leftElevatorTalon.getPosition();
    private final StatusSignal<Voltage> leftElevatorAppliedVoltage = leftElevatorTalon.getMotorVoltage();
    private final StatusSignal<Current> leftElevatorSupplyCurrent = leftElevatorTalon.getSupplyCurrent();
    private final StatusSignal<Current> leftElevatorCurrent = leftElevatorTalon.getStatorCurrent();
    private final StatusSignal<AngularVelocity> rightElevatorVelocity = rightElevatorTalon.getVelocity();
    private final StatusSignal<Angle> rightElevatorPosition = rightElevatorTalon.getPosition();
    private final StatusSignal<Voltage> rightElevatorAppliedVoltage = rightElevatorTalon.getMotorVoltage();
    private final StatusSignal<Current> rightElevatorSupplyCurrent = rightElevatorTalon.getSupplyCurrent();

    private final StatusSignal<Current> statorLeft = leftElevatorTalon.getStatorCurrent();
    private final StatusSignal<Current> statorRight = rightElevatorTalon.getStatorCurrent();

    public ElevatorIOReal() {
        TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        elevatorMotorConfig.Feedback.SensorToMechanismRatio = 1;
        elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        MotionMagicConfigs mmConfigs = elevatorMotorConfig.MotionMagic;
        mmConfigs.MotionMagicCruiseVelocity = RobotConstants.ElevatorConstants.motionCruiseVelocity.get();
        mmConfigs.MotionMagicAcceleration = RobotConstants.ElevatorConstants.motionAcceleration.get();
        leftElevatorTalon.setPosition(0.0);
        rightElevatorTalon.setPosition(0.0);
        leftElevatorTalon.getConfigurator().apply(elevatorMotorConfig);
        rightElevatorTalon.getConfigurator().apply(elevatorMotorConfig);
        StatusCode response = leftElevatorTalon.getConfigurator().apply(elevatorMotorConfig);
        if (response.isError())
            System.out.println("Left Elevator TalonFX failed config with error" + response);
        response = leftElevatorTalon.clearStickyFaults();
        if (response.isError())
            System.out.println("Left Elevator TalonFX failed sticky fault clearing with error" + response);

        rightElevatorTalon.setControl(new Follower(leftElevatorTalon.getDeviceID(), true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leftElevatorVelocity,
                leftElevatorPosition,
                leftElevatorAppliedVoltage,
                leftElevatorSupplyCurrent,
                rightElevatorVelocity,
                rightElevatorPosition,
                rightElevatorAppliedVoltage,
                rightElevatorSupplyCurrent,
                statorLeft, statorRight);

        inputs.leftElevatorVelocity = RadiansPerSecond.of(Units.rotationsToRadians(leftElevatorVelocity.getValueAsDouble()));
        inputs.leftElevatorPosition = Radians.of(Units.rotationsToRadians(leftElevatorPosition.getValueAsDouble()));
        inputs.leftElevatorAppliedVoltage = Volts.of(leftElevatorAppliedVoltage.getValueAsDouble());
        inputs.leftElevatorSupplyCurrent = Amps.of(leftElevatorSupplyCurrent.getValueAsDouble());

        inputs.rightElevatorVelocity = RadiansPerSecond.of(Units.rotationsToRadians(rightElevatorVelocity.getValueAsDouble()));
        inputs.rightElevatorPosition = Radians.of(Units.rotationsToRadians(rightElevatorPosition.getValueAsDouble()));
        inputs.rightElevatorAppliedVoltage = Volts.of(rightElevatorAppliedVoltage.getValueAsDouble());
        inputs.rightElevatorSupplyCurrent = Amps.of(rightElevatorSupplyCurrent.getValueAsDouble());

        inputs.statorCurrentAmps = new double[] { statorLeft.getValueAsDouble(), statorRight.getValueAsDouble() };

        inputs.ElevatorKP = ElevatorGainsClass.ELEVATOR_KP.get();
        inputs.ElevatorKI = ElevatorGainsClass.ELEVATOR_KI.get();
        inputs.ElevatorKD = ElevatorGainsClass.ELEVATOR_KD.get();
        inputs.ElevatorKA = ElevatorGainsClass.ELEVATOR_KA.get();
        inputs.ElevatorKV = ElevatorGainsClass.ELEVATOR_KV.get();
        inputs.ElevatorKS = ElevatorGainsClass.ELEVATOR_KS.get();
        inputs.ElevatorKG = ElevatorGainsClass.ELEVATOR_KG.get();

        leftElevatorTalon.getConfigurator().apply(new Slot0Configs()
                .withKP(inputs.ElevatorKP)
                .withKI(inputs.ElevatorKI)
                .withKD(inputs.ElevatorKD)
                .withKA(inputs.ElevatorKA)
                .withKV(inputs.ElevatorKV)
                .withKS(inputs.ElevatorKS)
                .withKG(inputs.ElevatorKG)
        );
    }

    @Override
    public void setElevatorDirectVoltage(double volts) {
        leftElevatorTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public void setElevatorTarget(double meters) {
        leftElevatorTalon.setControl(positionVoltage.withPosition(metersToRotations(meters)));
    }

    @Override
    public void resetElevatorPosition(){
        leftElevatorTalon.setPosition(0.0);
        rightElevatorTalon.setPosition(0.0);
    }

    @Override
    public boolean isNearExtension(double expected) {
        return MathUtil.isNear(metersToRotations(expected), leftElevatorTalon.getPosition().getValueAsDouble(), 0.02);
    }

    @Override
    public double getElevatorPosition() {
        return rotationsToMeters(leftElevatorTalon.getPosition().getValueAsDouble());
    }

    @Override
    public double getElevatorVelocity() {
        return leftElevatorVelocity.getValueAsDouble() * 60;
    }

    @Override
    public boolean isNearZeroExtension(){
        return MathUtil.isNear(metersToRotations(0.05), leftElevatorTalon.getPosition().getValueAsDouble(), 0.3);
    }

    private double metersToRotations(double heightMeters) {
        return (heightMeters / (Math.PI * ELEVATOR_SPOOL_DIAMETER)) * ELEVATOR_GEAR_RATIO;
    }

    private double rotationsToMeters(double rotations) {
        return rotations * (Math.PI * ELEVATOR_SPOOL_DIAMETER) / ELEVATOR_GEAR_RATIO;
    }
}