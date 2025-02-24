package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.ElevatorGainsClass;

import static frc.robot.RobotConstants.CANIVORE_CAN_BUS_NAME;
import static frc.robot.RobotConstants.ElevatorConstants.*;


public class ElevatorIOReal implements ElevatorIO {
    // Hardware
    private final TalonFX leader;
    private final TalonFX follower;

    // Configurators
    private final TalonFXConfigurator leaderConfigurator;
    private final TalonFXConfigurator followerConfigurator;

    private final Slot0Configs slot0Configs;
    private final MotionMagicConfigs motionMagicConfigs;

    private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);

    private final StatusSignal<AngularVelocity> velocityLeft;
    private final StatusSignal<Angle> positionLeft;
    private final StatusSignal<Voltage> voltageLeft;
    private final StatusSignal<Current> statorLeft;
    private final StatusSignal<Current> supplyLeft;
    private final StatusSignal<Temperature> tempLeft;
    private double setpointMeters = 0;

    public ElevatorIOReal() {
        this.leader = new TalonFX(LEFT_ELEVATOR_MOTOR_ID, CANIVORE_CAN_BUS_NAME);
        this.follower = new TalonFX(RIGHT_ELEVATOR_MOTOR_ID, CANIVORE_CAN_BUS_NAME);

        this.leaderConfigurator = leader.getConfigurator();
        this.followerConfigurator = follower.getConfigurator();

        // Configs
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = 80.0;
        currentLimitsConfigs.SupplyCurrentLimit = 30.0;

        leader.setPosition(heightToTalonPos(ELEVATOR_DEFAULT_POSITION_WHEN_DISABLED));
        follower.setPosition(heightToTalonPos(ELEVATOR_DEFAULT_POSITION_WHEN_DISABLED));

        MotorOutputConfigs leaderMotorConfigs = new MotorOutputConfigs();
        leaderMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        leaderMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        MotorOutputConfigs followerMotorConfigs = new MotorOutputConfigs();
        followerMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
        motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
        motionMagicConfigs.MotionMagicJerk = motionJerk.get();

        slot0Configs = new Slot0Configs();
        slot0Configs.kA = ElevatorGainsClass.ELEVATOR_KA.get();
        slot0Configs.kS = ElevatorGainsClass.ELEVATOR_KS.get();
        slot0Configs.kV = ElevatorGainsClass.ELEVATOR_KV.get();
        slot0Configs.kP = ElevatorGainsClass.ELEVATOR_KP.get();
        slot0Configs.kI = ElevatorGainsClass.ELEVATOR_KI.get();
        slot0Configs.kD = ElevatorGainsClass.ELEVATOR_KD.get();

        //Since elevator don't start at zero, not needed
        //resetElevatorPosition();

        leaderConfigurator.apply(currentLimitsConfigs);
        leaderConfigurator.apply(leaderMotorConfigs);
        leaderConfigurator.apply(slot0Configs);
        leaderConfigurator.apply(motionMagicConfigs);
        followerConfigurator.apply(currentLimitsConfigs);
        followerConfigurator.apply(followerMotorConfigs);
        followerConfigurator.apply(slot0Configs);
        followerConfigurator.apply(motionMagicConfigs);

        leader.clearStickyFaults();
        follower.clearStickyFaults();

        velocityLeft = leader.getVelocity();
        positionLeft = leader.getPosition();
        voltageLeft = leader.getSupplyVoltage();
        statorLeft = leader.getStatorCurrent();
        supplyLeft = leader.getSupplyCurrent();
        tempLeft = leader.getDeviceTemp();

        follower.setControl(new Follower(leader.getDeviceID(), true));

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                velocityLeft,
                positionLeft,
                voltageLeft,
                statorLeft,
                supplyLeft,
                tempLeft
        );

        inputs.positionMeters = getElevatorHeight();
        inputs.setpointMeters = setpointMeters;
        inputs.velocityMetersPerSec = getElevatorVelocity();
        inputs.appliedVolts = voltageLeft.getValueAsDouble();
        inputs.statorCurrentAmps = statorLeft.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyLeft.getValueAsDouble();
        inputs.tempCelsius = tempLeft.getValueAsDouble();

        if (RobotConstants.TUNING) {
            slot0Configs.kA = ElevatorGainsClass.ELEVATOR_KA.get();
            slot0Configs.kS = ElevatorGainsClass.ELEVATOR_KS.get();
            slot0Configs.kV = ElevatorGainsClass.ELEVATOR_KV.get();
            slot0Configs.kP = ElevatorGainsClass.ELEVATOR_KP.get();
            slot0Configs.kI = ElevatorGainsClass.ELEVATOR_KI.get();
            slot0Configs.kD = ElevatorGainsClass.ELEVATOR_KD.get();
            slot0Configs.kG = ElevatorGainsClass.ELEVATOR_KG.get();

            motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
            motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
            motionMagicConfigs.MotionMagicJerk = motionJerk.get();

            leaderConfigurator.apply(slot0Configs);
            followerConfigurator.apply(slot0Configs);
            leaderConfigurator.apply(motionMagicConfigs);
            followerConfigurator.apply(motionMagicConfigs);
        }
    }

    @Override
    public void setElevatorVoltage(double volts) {
        leader.setControl(new VoltageOut(volts));
    }

    @Override
    public void setElevatorTarget(double meters) {
        setpointMeters = meters;
        leader.setControl(motionRequest.withPosition(heightToTalonPos(meters)));
    }

    @Override
    public void resetElevatorPosition() {
        leader.setPosition(0.0);
        follower.setPosition(0.0);
    }

    @Override
    public double getElevatorVelocity() {
        return talonPosToHeight(leader.getVelocity().getValueAsDouble());
    }

    @Override
    public double getElevatorHeight() {
        return talonPosToHeight(leader.getPosition().getValueAsDouble());
    }

    @Override
    public boolean isNearZeroExtension() {
        return MathUtil.isNear(heightToTalonPos(0.05), leader.getPosition().getValueAsDouble(), 0.3);
    }

    @Override
    public boolean isNearExtension(double expected) {
        return MathUtil.isNear(expected, talonPosToHeight(leader.getPosition().getValueAsDouble()), 0.02);
    }

    private double heightToTalonPos(double heightMeters) {
        return (heightMeters / (Math.PI * ELEVATOR_SPOOL_DIAMETER)) * ELEVATOR_GEAR_RATIO;
    }

    private double talonPosToHeight(double rotations) {
        return rotations * (Math.PI * ELEVATOR_SPOOL_DIAMETER) / ELEVATOR_GEAR_RATIO;
    }
}