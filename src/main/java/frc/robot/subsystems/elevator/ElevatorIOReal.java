package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.RobotConstants;

import static frc.robot.RobotConstants.*;
import static frc.robot.RobotConstants.ElevatorConstants.*;
import static frc.robot.RobotConstants.ElevatorGainsClass.*;

public class ElevatorIOReal implements ElevatorIO {
    /* Hardware */
    private final TalonFX leader;
    private final TalonFX follower;

    /* Configurators */
    private TalonFXConfigurator leaderConfigurator;
    private TalonFXConfigurator followerConfigurator;

    /* Configs */
    private final CurrentLimitsConfigs currentLimitsConfigs;
    private final MotorOutputConfigs leaderMotorConfigs;
    private final MotorOutputConfigs followerMotorConfigs;
    private final Slot0Configs slot0Configs;
    private final MotionMagicConfigs motionMagicConfigs;

    private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);
    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

    private final StatusSignal<Voltage> voltageLeft;
    private final StatusSignal<Voltage> voltageRight;
    private final StatusSignal<Current> statorLeft;
    private final StatusSignal<Current> statorRight;
    private final StatusSignal<Current> supplyLeft;
    private final StatusSignal<Current> supplyRight;
    private final StatusSignal<Temperature> tempLeft;
    private final StatusSignal<Temperature> tempRight;
    private final StatusSignal<Double> closedLoopReferenceSlope;

    public ElevatorIOReal() {
        this.leader = new TalonFX(LEFT_ELEVATOR_MOTOR_ID, CANIVORE_CAN_BUS_NAME);
        this.follower = new TalonFX(RIGHT_ELEVATOR_MOTOR_ID, CANIVORE_CAN_BUS_NAME);

        this.leaderConfigurator = leader.getConfigurator();
        this.followerConfigurator = follower.getConfigurator();

        currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = 60.0;
        currentLimitsConfigs.SupplyCurrentLimit = 20.0;

        leaderMotorConfigs = new MotorOutputConfigs();
        leaderMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        leaderMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        followerMotorConfigs = new MotorOutputConfigs();
        followerMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
        motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
        motionMagicConfigs.MotionMagicJerk = motionJerk.get();

        slot0Configs = new Slot0Configs();
        slot0Configs.kA = ELEVATOR_KA.get();
        slot0Configs.kS = ELEVATOR_KS.get();
        slot0Configs.kV = ELEVATOR_KV.get();
        slot0Configs.kP = ELEVATOR_KP.get();
        slot0Configs.kI = ELEVATOR_KI.get();
        slot0Configs.kD = ELEVATOR_KD.get();

        leader.setPosition(0.0);
        follower.setPosition(0.0);

        leaderConfigurator.apply(currentLimitsConfigs);
        leaderConfigurator.apply(leaderMotorConfigs);
        leaderConfigurator.apply(slot0Configs);
        leaderConfigurator.apply(motionMagicConfigs);
        followerConfigurator.apply(currentLimitsConfigs);
        followerConfigurator.apply(followerMotorConfigs);
        followerConfigurator.apply(slot0Configs);
        followerConfigurator.apply(motionMagicConfigs);

        voltageLeft = leader.getSupplyVoltage();
        voltageRight = follower.getSupplyVoltage();
        statorLeft = leader.getStatorCurrent();
        statorRight = follower.getStatorCurrent();
        supplyLeft = leader.getSupplyCurrent();
        supplyRight = leader.getSupplyCurrent();
        tempLeft = leader.getDeviceTemp();
        tempRight = follower.getDeviceTemp();
        closedLoopReferenceSlope = leader.getClosedLoopReferenceSlope();

        follower.setControl(new Follower(leader.getDeviceID(), true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll();

        inputs.positionMeters = getHeight();
        inputs.velocityMetersPerSec = getVelocity();
        inputs.motionMagicVelocityTarget = rotationsToMeters(leader.getClosedLoopReferenceSlope().getValue());
        inputs.motionMagicPositionTarget = rotationsToMeters(leader.getClosedLoopReference().getValue());
        inputs.appliedVolts = new double[] { voltageLeft.getValueAsDouble(), voltageRight.getValueAsDouble() };
        inputs.statorCurrentAmps = new double[] { statorLeft.getValueAsDouble(), statorRight.getValueAsDouble() };
        inputs.supplyCurrentAmps = new double[] { supplyLeft.getValueAsDouble(), supplyRight.getValueAsDouble() };
        inputs.tempCelcius = new double[] { tempLeft.getValueAsDouble(), tempRight.getValueAsDouble() };

        if (RobotConstants.TUNING) {
            slot0Configs.kA = ELEVATOR_KA.get();
            slot0Configs.kS = ELEVATOR_KS.get();
            slot0Configs.kV = ELEVATOR_KV.get();
            slot0Configs.kP = ELEVATOR_KP.get();
            slot0Configs.kI = ELEVATOR_KI.get();
            slot0Configs.kD = ELEVATOR_KD.get();

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
    public void setPosition(double heightMeters) {
        leader.setControl(motionRequest.withPosition(metersToRotations(heightMeters)));
    }

    @Override
    public void setVoltage(double voltage) {
        leader.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void resetEncoder(final double position) {
        leader.setPosition(position);
        follower.setPosition(position);
    }

    private double getHeight() {
        return rotationsToMeters(leader.getPosition().getValueAsDouble());
    }

    private double getVelocity() {
        return rotationsToMeters(leader.getVelocity().getValueAsDouble());
    }

    private double metersToRotations(double heightMeters) {
        return (heightMeters / (Math.PI * ELEVATOR_SPOOL_DIAMETER)) * ELEVATOR_GEAR_RATIO;
    }

    private double rotationsToMeters(double rotations) {
        return rotations * (Math.PI * ELEVATOR_SPOOL_DIAMETER) / ELEVATOR_GEAR_RATIO;
    }

}
