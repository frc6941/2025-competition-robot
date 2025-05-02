// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstantsFactory;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.utils.TunableNumber;
import org.frcteam6941.swerve.SwerveSetpointGenerator.KinematicLimits;

import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class RobotConstants {

    // basic constants
    public static final boolean disableHAL = false;
    public static final double LOOPER_DT = 1 / 50.0;
    public static final boolean TUNING = true;
    public static final boolean DriverCamera = true;
    public static final boolean useReplay = true;
    // Judge whether the team is 10541
    // During Huston, is10541 should always be TRUE
    public static final String Serial10541 = "0327BA65";
    public static final boolean is10541 = RobotController.getSerialNumber().matches(Serial10541) && Robot.isReal();
    public static String CANIVORE_CAN_BUS_NAME = is10541 ? "10541Canivore0" : "6941Canivore0";
    public static String CLIMBER_CAN_BUS = is10541 ? "rio" : "6941Canivore0";

    /**
     * Constants related to the robot's indicators, such as LEDs.
     */
    public static class IndicatorConstants {
        public static final int LED_PORT = is10541 ? 0 : 9;
        public static final int LED_BUFFER_LENGTH = is10541 ? 30 : 92;
    }

    /**
     * Constants specific to the swerve drivetrain configuration.
     */
    public static class SwerveConstants {
        // tolerance seconds, for swerve to reset in auto.
        // current set to 0.01s (10ms) since no path would consume <10ms.
        public static final double AUTO_SWERVE_TOLERANCE_SECS = 0.01;

        // pigeon id
        public static final int PIGEON_ID = 14;

        // swerve driving
        /**
         * Gearing between the drive motor output shaft and the wheel.
         */
        public static final double DRIVE_GEAR_RATIO = 6.7460317460317460317460317460317;
        /**
         * Gearing between the steer motor output shaft and the azimuth gear.
         */
        public static final double STEER_GEAR_RATIO = 21.428571428571428571428571428571;

        /**
         * Max Voltage Output in voltage.
         */
        public static final Measure<VoltageUnit> MAX_VOLTAGE = Volts.of(12.0);
        /**
         * Radius of the wheel in meters.
         */
        public static final Measure<DistanceUnit> wheelRadius = Meters.of(0.0479);
        /**
         * The max speed of the swerve (should not larger than speedAt12Volts)
         */
        public static final Measure<LinearVelocityUnit> maxSpeed = MetersPerSecond.of(3.5);//4.5
        /**
         * The max angular speed of the swerve.
         */
        public static final Measure<AngularVelocityUnit> maxAngularRate = RotationsPerSecond.of(1.5 * Math.PI);

        // Kinematic limits for different driving modes
        public static final KinematicLimits DRIVETRAIN_UNCAPPED = new KinematicLimits(maxSpeed.magnitude(),
                11.0, 1000.0);
        public static final KinematicLimits DRIVETRAIN_SMOOTHED = new KinematicLimits(4.5, 30.0, 200.0);
        public static final KinematicLimits DRIVETRAIN_LIMITED = new KinematicLimits(1.5, 10.0, 1200.0);
        public static final KinematicLimits DRIVETRAIN_ROBOT_ORIENTED = new KinematicLimits(2.0, 5.0, 1500.0);

        public static final Measure<LinearVelocityUnit> speedAt12Volts = maxSpeed;
        /**
         * The stator current at which the wheels start to slip
         */
        public static final Measure<CurrentUnit> slipCurrent = Amps.of(150.0);
        /**
         * Theoretical free speed (m/s) at 12v applied output;
         */

        // ffw & wheel c
        public static final Measure<DistanceUnit> wheelCircumferenceMeters = Meters
                .of(wheelRadius.magnitude() * 2 * Math.PI);
        public static final SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(0.69522,
                2.3623, 0.19367);

        public static final double statorCurrent = 110;
        public static final double supplyCurrent = 50;
        public static final double VOLTAGE_CLOSED_LOOP_RAMP_PERIOD = 0;//0.5
        public static final double deadband = 0.05;
        public static final double rotationalDeadband = 0.05;

        /**
         * Swerve steering gains
         */
        private static final Slot0Configs steerGains = new Slot0Configs().withKP(120)// 120
                .withKI(0.2)// 0.2
                .withKD(0.005)// 0.005
                .withKS(0).withKV(0).withKA(0);

        /**
         * Swerve driving gains
         */
        private static final Slot0Configs driveGains = new Slot0Configs().withKP(1).withKI(0).withKD(0)
                .withKS(0).withKV(0.12).withKA(0);
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        /**
         * The closed-loop output type to use for the drive motors; This affects the
         * PID/FF gains for the drive motors
         */
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;
        /**
         * The closed-loop output type to use for the steer motors; This affects the
         * PID/FF gains for the steer motors
         */

        private static final double STEER_INERTIA = 0.00001;
        /**
         * Simulation only
         */
        private static final double DRIVE_INERTIA = 0.001;
        /**
         * Simulation only
         */
        private static final Measure<VoltageUnit> steerFrictionVoltage = Volts.of(0.25);
        /**
         * Simulation only
         */
        private static final Measure<VoltageUnit> driveFrictionVoltage = Volts.of(0.25);
        /**
         * Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
         */
        private static final double COUPLE_RATIO = 3.5;
        private static final boolean STEER_MOTOR_REVERSED = true;
        public static final LegacySwerveModuleConstantsFactory ConstantCreator = new LegacySwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO).withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withWheelRadius(wheelRadius.in(Inches)).withSlipCurrent(slipCurrent.magnitude())
                .withSteerMotorGains(steerGains).withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                .withSpeedAt12VoltsMps(speedAt12Volts.magnitude()).withSteerInertia(STEER_INERTIA)
                .withDriveInertia(DRIVE_INERTIA)
                .withSteerFrictionVoltage(steerFrictionVoltage.magnitude())
                .withDriveFrictionVoltage(driveFrictionVoltage.magnitude())
                .withFeedbackSource(LegacySwerveModuleConstants.SteerFeedbackType.SyncCANcoder)
                .withCouplingGearRatio(COUPLE_RATIO).withSteerMotorInverted(STEER_MOTOR_REVERSED);
        private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 4;
        private static final int FRONT_LEFT_STEER_MOTOR_ID = 3;
        private static final int FRONT_LEFT_ENCODER_ID = 10;
        private static final double FRONT_LEFT_ENCODER_OFFSET = is10541 ? -0.1533204844 : -0.4532636;
        private static final Measure<DistanceUnit> frontLeftXPos = Meters.of(0.29);
        private static final Measure<DistanceUnit> frontLeftYPos = Meters.of(0.29);
        public static final LegacySwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                FRONT_LEFT_STEER_MOTOR_ID, FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_ENCODER_ID,
                FRONT_LEFT_ENCODER_OFFSET, frontLeftXPos.magnitude(), frontLeftYPos.magnitude(), false);
        // Front Right
        private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 6;
        private static final int FRONT_RIGHT_STEER_MOTOR_ID = 5;
        private static final int FRONT_RIGHT_ENCODER_ID = 11;
        private static final double FRONT_RIGHT_ENCODER_OFFSET = is10541 ? -0.3049317344 : -0.420898875;
        private static final Measure<DistanceUnit> frontRightXPos = Meters.of(0.29);
        private static final Measure<DistanceUnit> frontRightYPos = Meters.of(-0.29);
        public static final LegacySwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                FRONT_RIGHT_STEER_MOTOR_ID, FRONT_RIGHT_DRIVE_MOTOR_ID, FRONT_RIGHT_ENCODER_ID,
                FRONT_RIGHT_ENCODER_OFFSET, frontRightXPos.magnitude(), frontRightYPos.magnitude(),
                true);
        // Back Left
        private static final int BACK_LEFT_DRIVE_MOTOR_ID = 2;
        private static final int BACK_LEFT_STEER_MOTOR_ID = 1;
        private static final int BACK_LEFT_ENCODER_ID = 0;
        private static final double BACK_LEFT_ENCODER_OFFSET = is10541 ? 0.0383297031 : -0.354248046875;
        private static final Measure<DistanceUnit> backLeftXPos = Meters.of(-0.29);
        private static final Measure<DistanceUnit> backLeftYPos = Meters.of(0.29);
        public static final LegacySwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                BACK_LEFT_STEER_MOTOR_ID, BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_ENCODER_ID,
                BACK_LEFT_ENCODER_OFFSET, backLeftXPos.magnitude(), backLeftYPos.magnitude(), false);
        // Back Right
        private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 8;
        private static final int BACK_RIGHT_STEER_MOTOR_ID = 7;
        private static final int BACK_RIGHT_ENCODER_ID = 20;
        private static final double BACK_RIGHT_ENCODER_OFFSET = is10541 ? 0.6206053438 : 0.4663082031;
        private static final Measure<DistanceUnit> backRightXPos = Meters.of(-0.29);
        private static final Measure<DistanceUnit> backRightYPos = Meters.of(-0.29);
        public static final LegacySwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                BACK_RIGHT_STEER_MOTOR_ID, BACK_RIGHT_DRIVE_MOTOR_ID, BACK_RIGHT_ENCODER_ID,
                BACK_RIGHT_ENCODER_OFFSET, backRightXPos.magnitude(), backRightYPos.magnitude(), true);
        // swervemodule
        public static LegacySwerveModuleConstants[] modules = {FrontLeft, FrontRight, BackLeft, BackRight};
        public static final Translation2d[] modulePlacements = new Translation2d[]{
                new Translation2d(SwerveConstants.FrontLeft.LocationX,
                        SwerveConstants.FrontLeft.LocationY),
                new Translation2d(SwerveConstants.FrontRight.LocationX,
                        SwerveConstants.FrontRight.LocationY),
                new Translation2d(SwerveConstants.BackLeft.LocationX,
                        SwerveConstants.BackLeft.LocationY),
                new Translation2d(SwerveConstants.BackRight.LocationX,
                        SwerveConstants.BackRight.LocationY)};

        /**
         * Constants for the heading controller used in the swerve drivetrain.
         */
        public static class headingController {
            public static final frc.robot.utils.TunableNumber HEADING_KP = new frc.robot.utils.TunableNumber(
                    "HEADING PID/kp", 0.08);
            public static final frc.robot.utils.TunableNumber HEADING_KI = new frc.robot.utils.TunableNumber(
                    "HEADING PID/ki", 0.000);
            public static final frc.robot.utils.TunableNumber HEADING_KD = new frc.robot.utils.TunableNumber(
                    "HEADING PID/kd", 0.005);
            public static final frc.robot.utils.TunableNumber MAX_ERROR_CORRECTION_ANGLE = new frc.robot.utils.TunableNumber(
                    "HEADING/Max Error Correction Angle", 50);
        }

        /**
         * Constants for the steer motor gains in the swerve drivetrain.
         * IF YOU WANT TO ADJUST SWERVE PID, GO TO src/main/java/org/frcteam6941/swerve/CTRESwerveModule.java AND FOLLOW THE INSTRUCTION
         */
        public static class steerGainsClass {
            public static final TunableNumber STEER_KP = new TunableNumber("STEER PID/kp", 120);
            public static final TunableNumber STEER_KI = new TunableNumber("STEER PID/ki", 0.2);
            public static final TunableNumber STEER_KD = new TunableNumber("STEER PID/kd", 0.005);
            public static final TunableNumber STEER_KA = new TunableNumber("STEER PID/ka", 0);
            public static final TunableNumber STEER_KV = new TunableNumber("STEER PID/kv", 0);
            public static final TunableNumber STEER_KS = new TunableNumber("STEER PID/ks", 0);
        }

        /**
         * Constants for the drive motor gains in the swerve drivetrain.
         * IF YOU WANT TO ADJUST SWERVE PID, GO TO src/main/java/org/frcteam6941/swerve/CTRESwerveModule.java AND FOLLOW THE INSTRUCTION
         */
        public static class driveGainsClass {
            public static final TunableNumber DRIVE_KP = new TunableNumber("DRIVE PID/kp", 0.025);
            public static final TunableNumber DRIVE_KI = new TunableNumber("DRIVE PID/ki", 0);
            public static final TunableNumber DRIVE_KD = new TunableNumber("DRIVE PID/kd", 0.0);
            public static final TunableNumber DRIVE_KA = new TunableNumber("DRIVE PID/ka", 0);
            public static final TunableNumber DRIVE_KV = new TunableNumber("DRIVE PID/kv", 0.125);
            public static final TunableNumber DRIVE_KS = new TunableNumber("DRIVE PID/ks", 0.28);
        }

        /**
         * Constants for the gains in the ReefAimCommand.
         */
        public static class AimGainsClass {
            public static final TunableNumber AIM_KP = new TunableNumber("AIM/kp", 4.0);
            public static final TunableNumber AIM_KI = new TunableNumber("AIM/ki", 0);
            public static final TunableNumber AIM_KD = new TunableNumber("AIM/kd", 0.1);
        }

    }

    /**
     * Constants specific to the reef aim mechanism.
     */
    public static final class ReefAimConstants {
        public static final TunableNumber MAX_DISTANCE_REEF_LINEUP = new TunableNumber("AIM/maxLineupDistance", 1.5);
        public static final Measure<DistanceUnit> PIPE_TO_TAG = Meters.of(0.164308503);
        public static final TunableNumber ROBOT_TO_PIPE_METERS = new TunableNumber("AIM/ROBOT_TO_PIPE_METERS", 0.59);
        public static final TunableNumber X_TOLERANCE_METERS = new TunableNumber("AIM/X_TOLERANCE_METERS", 0.01);
        public static final TunableNumber Y_TOLERANCE_METERS = new TunableNumber("AIM/Y_TOLERANCE_METERS", 0.01);
        public static final TunableNumber RAISE_LIMIT_METERS = new TunableNumber("AIM/RAISE_LIMIT_METERS", 1);
        public static final TunableNumber OMEGA_TOLERANCE_DEGREES = new TunableNumber("AIM/OMEGA_TOLERANCE_DEGREES", 1);
        public static final Measure<LinearVelocityUnit> MAX_AIMING_SPEED = MetersPerSecond.of(3.5);
        public static final Measure<LinearAccelerationUnit> MAX_AIMING_ACCELERATION = MetersPerSecondPerSecond.of(7);
        public static final TunableNumber Edge_Case_Max_Delta = new TunableNumber("AIM/MAX DELTA", 0.5);
        public static final TunableNumber ROBOT_TO_ALGAE_METERS = new TunableNumber("AIM/ROBOT_TO_ALGAE_METERS", 0.48);
        public static final TunableNumber ALGAE_TO_TAG_METERS = new TunableNumber("AIM/ALGAE_TO_TAG_METERS", 0);
    }

    /**
     * Constants related to the beambreak subsystem.
     */
    public static class BeamBreakConstants {
        public static final int ENDEFFECTORARM_CORAL_BEAMBREAK_ID = 0;
        public static final int ENDEFFECTORARM_ALGAE_BEAMBREAK_ID = 2;
        public static final int INTAKE_BEAMBREAK_ID = 3;
    }

    /**
     * Constants related to the robot's intake subsystem.
     */
    public static class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 15;
        public static final int INTAKE_PIVOT_MOTOR_ID = 16;
        public static final int INTAKE_PIVOT_ENCODER_ID = 17;
        public static final double INTAKE_PIVOT_ROTOR_ENCODER_RATIO = (12.0 * 50) / 11;

        //Constants for intake roller
        public static final int STATOR_CURRENT_LIMIT_AMPS = 80;
        public static final int SUPPLY_CURRENT_LIMIT_AMPS = 80;
        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERT = false;
        public static final double REDUCTION = 1;
        public static final double moi = 0;//inertia for simulation
        public static final double ROLLER_RATIO = 1;
        public static final double INTAKE_DANGER_ZONE = 90;
        public static final TunableNumber ROLLER_AMPS_HAS_CORAL = new TunableNumber("INTAKE_ROLLER/rollerAmpsHasCoral", 55);
        //Motion constants for intake pivot
        public static final TunableNumber INTAKE_PIVOT_CRUISE_VELOCITY = new TunableNumber("INTAKE_PIVOT/cruiseVelocity", 250);
        public static final TunableNumber INTAKE_PIVOT_ACCELERATION = new TunableNumber("INTAKE_PIVOT/acceleration", 250);
        public static final TunableNumber INTAKE_PIVOT_JERK = new TunableNumber("INTAKE_PIVOT/jerk", 0);
        public static final TunableNumber DEPLOY_ANGLE = new TunableNumber("INTAKE_PIVOT/deployAngle", 112.5);
        public static final TunableNumber OUTTAKE_ANGLE = new TunableNumber("INTAKE_PIVOT/outtakeAngle", 105);
        public static final TunableNumber HOME_ANGLE = new TunableNumber("INTAKE_PIVOT/homeAngle", 40);
        public static final TunableNumber AVOID_ANGLE = new TunableNumber("INTAKE_PIVOT/avoidAngle", 90);
        public static final TunableNumber FUNNEL_AVOID_ANGLE = new TunableNumber("INTAKE_PIVOT/funnelAvoidAngle", 51);
        public static final TunableNumber SHOOT_ANGLE = new TunableNumber("INTAKE_PIVOT/shootAngle", 45);
        public static final double INTAKE_PIVOT_ENCODER_OFFSET = -0.018473310625;
        //Motion constants for intake roller
        public static final TunableNumber INTAKE_VOLTAGE = new TunableNumber("INTAKE_ROLLER/intakeVoltage", 15.0);
        public static final TunableNumber OUTTAKE_VOLTAGE = new TunableNumber("INTAKE_ROLLER/outtakeVoltage", -6.0);
        public static final TunableNumber SHOOT_VOLTAGE = new TunableNumber("INTAKE_ROLLER/shootVoltage", -2.5);
        public static final TunableNumber INTAKE_HOLD_VOLTAGE = new TunableNumber("INTAKE_ROLLER/intakeHoldVoltage", 5.0);
        public static final TunableNumber OUT_TAKE_HOLD = new TunableNumber("INTAKE_ROLLER/outtakeHoldVoltage", -1.0);

        public static final TunableNumber OUTTAKE_TIME = new TunableNumber("INTAKE_ROLLER/outtake time", 0.3);
        public static final TunableNumber INTAKE_TIME = new TunableNumber("INTAKE_ROLLER/intake time", 0.4);
        // public static final TunableNumber INTAKE_VOLTAGE = new TunableNumber("INTAKE_ROLLER/intake time",10);

        //Constants for intake pivot
        public static double PIVOT_RATIO = (double) (12 * 50) / 11;

        /**
         * Constants for the intake pivot motor gains in the intake subsystem.
         */
        public static class IntakePivotGainsClass {
            public static final TunableNumber INTAKE_PIVOT_KP = new TunableNumber("INTAKE_PIVOT PID/kp", 60);
            public static final TunableNumber INTAKE_PIVOT_KI = new TunableNumber("INTAKE_PIVOT PID/ki", 0);
            public static final TunableNumber INTAKE_PIVOT_KD = new TunableNumber("INTAKE_PIVOT PID/kd", 1);
        }
    }

    public static class ClimberConstants {
        public static final int CLIMBER_MOTOR_ID = 52;
        public static final double CLIMBER_RATIO = 5 * 5 * 4;

        public static final TunableNumber CLIMBER_CRUISE_VELOCITY = new TunableNumber("CLIMBER/ClimberCruiseVelocity", 100);
        public static final TunableNumber CLIMBER_ACCELERATION = new TunableNumber("CLIMBER/ClimberAcceleration", 200);
        public static final TunableNumber CLIMBER_JERK = new TunableNumber("CLIMBER/ClimberJerk", 0);

        /**
         * Constants for the CLIMBER pivot motor gains in the CLIMBER subsystem.
         */
        public static class ClimberGainsClass {
            public static final TunableNumber CLIMBER_KP = new TunableNumber("CLIMBER PID/kp",
                    2.001);
            public static final TunableNumber CLIMBER_KI = new TunableNumber("CLIMBER PID/ki", 0);
            public static final TunableNumber CLIMBER_KD = new TunableNumber("CLIMBER PID/kd",
                    0.01);
            public static final TunableNumber CLIMBER_KA = new TunableNumber("CLIMBER PID/ka", 0);
            public static final TunableNumber CLIMBER_KV = new TunableNumber("CLIMBER PID/kv",
                    0);
            public static final TunableNumber CLIMBER_KS = new TunableNumber("CLIMBER PID/ks",
                    0);
        }
    }

    /**
     * Constants related to the elevator subsystem.
     */
    public static class ElevatorConstants {
        public static final int LEFT_ELEVATOR_MOTOR_ID = 50;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 51;

        public static final double ELEVATOR_SPOOL_DIAMETER = 0.04 + 0.003; //0.04m for spool diameter, 0.003 for rope diameter
        public static final double ELEVATOR_GEAR_RATIO = 3.0;
        public static final double ELEVATOR_DANGER_ZONE = 0.4180619200456253;
        public static final double ELEVATOR_DEFAULT_POSITION_WHEN_DISABLED = 0.0;//TODO: fixme

        public static final TunableNumber motionAcceleration = new TunableNumber("Elevator/MotionAcceleration",
                300);
        public static final TunableNumber motionCruiseVelocity = new TunableNumber("Elevator/MotionCruiseVelocity",
                100);
        public static final TunableNumber motionJerk = new TunableNumber("Elevator/MotionJerk",
                0.0);
        public static final TunableNumber MAX_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/max",
                1.4);
        public static final TunableNumber HOLD_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/idle",
                0.6);
        public static final TunableNumber HOME_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/HOME",
                0.13);
        public static final TunableNumber HOLD_INTAKE_METERS = new TunableNumber("ELEVATOR SETPOINTS/HOLD INTAKE",
                0.16);
        public static final TunableNumber ALGAE_NET_EXTENSION_METER = new TunableNumber("ELEVATOR SETPOINTS/ALGAE_NET",
                1.4);
        public static final TunableNumber ALGAE_PROCESSOR_EXTENSION_METER = new TunableNumber("ELEVATOR SETPOINTS/ALGAE_PROCESSOR",
                0.12);
        public static final TunableNumber L1_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/L1",
                0.3);
        public static final TunableNumber L2_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/L2",
                0.14);
        public static final TunableNumber L3_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/L3",
                0.545);
        public static final TunableNumber L4_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/L4",
                1.4);
        public static final TunableNumber P1_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/P1",
                0.55);
        public static final TunableNumber P2_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/P2",
                0.88);
        public static final TunableNumber PRE_INTAKE_METERS = new TunableNumber("ELEVATOR SETPOINTS/PRE_INTAKE",
                0.65);

        public static final TunableNumber ELEVATOR_ZEROING_CURRENT = new TunableNumber("Elevator zeroing current",
                40);
        public static final TunableNumber ELEVATOR_MIN_SAFE_HEIGHT = new TunableNumber("Elevator min safe height", 0.59);

    }

    /**
     * Constants for the elevator motor gains.
     */
    public static class ElevatorGainsClass {
        public static final TunableNumber ELEVATOR_KP = new TunableNumber("ELEVATOR PID/kp", 2.5);
        public static final TunableNumber ELEVATOR_KI = new TunableNumber("ELEVATOR PID/ki", 0);
        public static final TunableNumber ELEVATOR_KD = new TunableNumber("ELEVATOR PID/kd", 0);
        public static final TunableNumber ELEVATOR_KA = new TunableNumber("ELEVATOR PID/ka", 0);
        public static final TunableNumber ELEVATOR_KV = new TunableNumber("ELEVATOR PID/kv", 0.08);// 0.107853495
        public static final TunableNumber ELEVATOR_KS = new TunableNumber("ELEVATOR PID/ks", 0.1);
        public static final TunableNumber ELEVATOR_KG = new TunableNumber("ELEVATOR PID/kg", 0.2);//0.3
    }

    public static class LimelightConstants {
        public static final String LIMELIGHT_LEFT = "limelight-leftf";
        public static final String LIMELIGHT_RIGHT = "limelight-rightf";
        public static final double AREA_THRESHOLD = 0.1;
    }

    /**
     * Constants related to the EndEffectorArm subsystem.
     */
    public static class EndEffectorArmConstants {
        // Motor IDs
        public static final int END_EFFECTOR_ARM_PIVOT_MOTOR_ID = 21;
        public static final int END_EFFECTOR_ARM_ROLLER_MOTOR_ID = 22;
        public static final int END_EFFECTOR_ARM_ENCODER_ID = 23;

        // Roller motor configuration
        public static final int STATOR_CURRENT_LIMIT_AMPS = 80;
        public static final int SUPPLY_CURRENT_LIMIT_AMPS = 40;
        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERT = false;

        // Pivot motor configuration
        public static final double ROTOR_SENSOR_RATIO = 1.0 / 8 * 64 / 18 * 60;
        public static final double END_EFFECTOR_ARM_ENCODER_OFFSET = 0.0661628126 + 0.008;

        // Pivot angles for different positions (in degrees)
        public static final TunableNumber HOME_ANGLE = new TunableNumber("END_EFFECTOR_ARM_PIVOT/homeAngle", 135);
        public static final TunableNumber CORAL_INTAKE_ANGLE = new TunableNumber("END_EFFECTOR_ARM_PIVOT/coralIntakeAngle", 0);
        public static final TunableNumber CORAL_OUTTAKE_ANGLE = new TunableNumber("END_EFFECTOR_ARM_PIVOT/coralOuttakeAngle", 0);
        public static final TunableNumber CORAL_PRESHOOT_ANGLE = new TunableNumber("END_EFFECTOR_ARM_PIVOT/coralPreShootAngle", 190);
        public static final TunableNumber CORAL_PRESHOOT_ANGLE_L4 = new TunableNumber("END_EFFECTOR_ARM_PIVOT/coralPreShootAngleL4", 220);
        public static final TunableNumber CORAL_PRESHOOT_ANGLE_L1 = new TunableNumber("END_EFFECTOR_ARM_PIVOT/coralPreShootAngleL1", 200);
        public static final TunableNumber ALGAE_INTAKE_ANGLE = new TunableNumber("END_EFFECTOR_ARM_PIVOT/algaeIntakeAngle", 0);
        public static final TunableNumber ALGAE_NET_PRESHOOT_ANGLE = new TunableNumber("END_EFFECTOR_ARM_PIVOT/algaeNetPreShootAngle", 230.0);
        public static final TunableNumber ALGAE_PROCESSOR_PRESHOOT_ANGLE = new TunableNumber("END_EFFECTOR_ARM_PIVOT/algaeProcessorPreShootAngle", 0.0);


        // Roller voltages for different operations
        public static final TunableNumber CORAL_INTAKE_VOLTAGE = new TunableNumber("END_EFFECTOR_ARM_ROLLER/coralIntakeVoltage", 12.0);
        public static final TunableNumber CORAL_OUTTAKE_VOLTAGE = new TunableNumber("END_EFFECTOR_ARM_ROLLER/coralOuttakeVoltage", -6.0);
        public static final TunableNumber CORAL_PRESHOOT_VOLTAGE = new TunableNumber("END_EFFECTOR_ARM_ROLLER/coralPreShootVoltage", -10.0);
        public static final TunableNumber ALGAE_INTAKE_VOLTAGE = new TunableNumber("END_EFFECTOR_ARM_ROLLER/algaeIntakeVoltage", 8.0);
        public static final TunableNumber ALGAE_PRESHOOT_VOLTAGE = new TunableNumber("END_EFFECTOR_ARM_ROLLER/algaePreShootVoltage", -12.0);
        public static final TunableNumber CORAL_HOLD_VOLTAGE = new TunableNumber("END_EFFECTOR_ARM_ROLLER/coralHoldVoltage", 0.5);
        public static final TunableNumber ALGAE_HOLD_VOLTAGE = new TunableNumber("END_EFFECTOR_ARM_ROLLER/algaeHoldVoltage", 1.5);
        public static final TunableNumber CORAL_SHOOT_VOLTAGE = new TunableNumber("END_EFFECTOR_ARM_ROLLER/coralShootVoltage", -12.0);
        public static final TunableNumber CORAL_SHOOT_VOLTAGE_L1 = new TunableNumber("END_EFFECTOR_ARM_ROLLER/coralShootVoltageL1", -2.0);
        public static final TunableNumber ALGAE_NET_SHOOT_VOLTAGE = new TunableNumber("END_EFFECTOR_ARM_ROLLER/algaeNetShootVoltage", -15.0);
        public static final TunableNumber ALGAE_PROCESSOR_SHOOT_VOLTAGE = new TunableNumber("END_EFFECTOR_ARM_ROLLER/algaeProcessorShootVoltage", -4.0);
        public static final TunableNumber CORAL_SHOOT_DELAY_TIME = new TunableNumber("END_EFFECTOR_ARM_ROLLER/coralShootDelayTime", 0.2);

        /**
         * Constants for the EndEffectorArm pivot motor gains.
         */
        public static class EndEffectorArmPivotGainsClass {
            public static final TunableNumber END_EFFECTOR_ARM_PIVOT_KP = new TunableNumber("END_EFFECTOR_ARM_PIVOT_PID/kp", 2.5);
            public static final TunableNumber END_EFFECTOR_ARM_PIVOT_KI = new TunableNumber("END_EFFECTOR_ARM_PIVOT_PID/ki", 0);
            public static final TunableNumber END_EFFECTOR_ARM_PIVOT_KD = new TunableNumber("END_EFFECTOR_ARM_PIVOT_PID/kd", 0.1);//0.3 or 0.1 or 0.01
            public static final TunableNumber END_EFFECTOR_ARM_PIVOT_KA = new TunableNumber("END_EFFECTOR_ARM_PIVOT_PID/ka", 0);
            public static final TunableNumber END_EFFECTOR_ARM_PIVOT_KV = new TunableNumber("END_EFFECTOR_ARM_PIVOT_PID/kv", 0);
            public static final TunableNumber END_EFFECTOR_ARM_PIVOT_KS = new TunableNumber("END_EFFECTOR_ARM_PIVOT_PID/ks", 0.015);
            public static final TunableNumber END_EFFECTOR_ARM_PIVOT_KG = new TunableNumber("END_EFFECTOR_ARM_PIVOT_PID/kg", -0.0513);
        }

        /**
         * Constants for the EndEffectorArm roller motor gains.
         */
        // EndEffector roller is currently open loop
        public static class EndEffectorArmRollerGainsClass {
            public static final TunableNumber END_EFFECTOR_ARM_ROLLER_KP = new TunableNumber("END_EFFECTOR_ARM_ROLLER_PID/kp", 0);
            public static final TunableNumber END_EFFECTOR_ARM_ROLLER_KI = new TunableNumber("END_EFFECTOR_ARM_ROLLER_PID/ki", 0);
            public static final TunableNumber END_EFFECTOR_ARM_ROLLER_KD = new TunableNumber("END_EFFECTOR_ARM_ROLLER_PID/kd", 0);
            public static final TunableNumber END_EFFECTOR_ARM_ROLLER_KA = new TunableNumber("END_EFFECTOR_ARM_ROLLER_PID/ka", 0);
            public static final TunableNumber END_EFFECTOR_ARM_ROLLER_KV = new TunableNumber("END_EFFECTOR_ARM_ROLLER_PID/kv", 0);
            public static final TunableNumber END_EFFECTOR_ARM_ROLLER_KS = new TunableNumber("END_EFFECTOR_ARM_ROLLER_PID/ks", 0);
        }
    }
}
       