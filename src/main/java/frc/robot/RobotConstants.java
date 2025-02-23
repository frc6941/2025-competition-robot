// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstantsFactory;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
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
    // Judge whether the team is 10541
    public static final String Serial10541 = "03415993";
    public static final boolean is10541 = RobotController.getSerialNumber().matches(Serial10541) || Robot.isReal();
    // canbus name
    public static String CAN_BUS_NAME = "rio";
    public static String CANIVORE_CAN_BUS_NAME = "10541Canivore0";
    // serial baud rate
    public static int baudRate = 115200;

    /**
     * Constants related to the robot's vision subsystem.
     */
    public static class VisionConstants {
        public static final SerialPort.Port serialPort = SerialPort.Port.kMXP;
    }

    /**
     * Constants related to the robot's indicators, such as LEDs.
     */
    public static class IndicatorConstants {
        // TODO:adapt when needed
        public static final int LED_PORT = 0;
        public static final int LED_BUFFER_LENGTH = 17;
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
        public static final Measure<LinearVelocityUnit> maxSpeed = MetersPerSecond.of(4.5);
        /**
         * The max angular speed of the swerve.
         */
        public static final Measure<AngularVelocityUnit> maxAngularRate = RotationsPerSecond.of(1.5 * Math.PI);

        // Kinematic limits for different driving modes
        public static final KinematicLimits DRIVETRAIN_UNCAPPED = new KinematicLimits(maxSpeed.magnitude(),
                11.0, 1000.0);
        public static final KinematicLimits DRIVETRAIN_SMOOTHED = new KinematicLimits(4.5, 30.0, 200.0);
        public static final KinematicLimits DRIVETRAIN_LIMITED = new KinematicLimits(2.0, 10.0, 1200.0);
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
        public static final double VOLTAGE_CLOSED_LOOP_RAMP_PERIOD = 0.5;
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
        private static final double FRONT_LEFT_ENCODER_OFFSET = is10541 ? -0.1481936719 : -0.458007;
        private static final Measure<DistanceUnit> frontLeftXPos = Meters.of(0.29);
        private static final Measure<DistanceUnit> frontLeftYPos = Meters.of(0.29);
        public static final LegacySwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                FRONT_LEFT_STEER_MOTOR_ID, FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_ENCODER_ID,
                FRONT_LEFT_ENCODER_OFFSET, frontLeftXPos.magnitude(), frontLeftYPos.magnitude(), false);
        // Front Right
        private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 6;
        private static final int FRONT_RIGHT_STEER_MOTOR_ID = 5;
        private static final int FRONT_RIGHT_ENCODER_ID = 11;
        private static final double FRONT_RIGHT_ENCODER_OFFSET = is10541 ? -0.310058875 : -0.420166;
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
        private static final double BACK_LEFT_ENCODER_OFFSET = is10541 ? -0.1186526563 : -0.3500976;
        private static final Measure<DistanceUnit> backLeftXPos = Meters.of(-0.29);
        private static final Measure<DistanceUnit> backLeftYPos = Meters.of(0.29);
        public static final LegacySwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                BACK_LEFT_STEER_MOTOR_ID, BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_ENCODER_ID,
                BACK_LEFT_ENCODER_OFFSET, backLeftXPos.magnitude(), backLeftYPos.magnitude(), false);
        // Back Right
        private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 8;
        private static final int BACK_RIGHT_STEER_MOTOR_ID = 7;
        private static final int BACK_RIGHT_ENCODER_ID = 20;
        private static final double BACK_RIGHT_ENCODER_OFFSET = is10541 ? 0.6188965 : 0.4609375;
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
            public static final TunableNumber DRIVE_KP = new TunableNumber("DRIVE PID/kp", 0.03);
            public static final TunableNumber DRIVE_KI = new TunableNumber("DRIVE PID/ki", 0);
            public static final TunableNumber DRIVE_KD = new TunableNumber("DRIVE PID/kd", 0.0001);
            public static final TunableNumber DRIVE_KA = new TunableNumber("DRIVE PID/ka", 0);
            public static final TunableNumber DRIVE_KV = new TunableNumber("DRIVE PID/kv", 0.12);
            public static final TunableNumber DRIVE_KS = new TunableNumber("DRIVE PID/ks", 0.045);
        }

        /**
         * Constants for the gains in the ReefAimCommand.
         */
        public static class AimGainsClass {
            public static final TunableNumber AIM_KP = new TunableNumber("AIM PID/kp", 5);
            public static final TunableNumber AIM_KI = new TunableNumber("AIM PID/ki", 0.5);
            public static final TunableNumber AIM_KD = new TunableNumber("AIM PID/kd", 0.5);
        }

    }

    /**
     * Constants specific to the reef aim mechanism.
     */
    public static final class ReefAimConstants {
        public static final Transform2d tagLeftToRobot = new Transform2d(); // vec(robot) - vec(tag) when
        // shooting left coral
        public static final Transform2d tagRightToRobot = new Transform2d();
    }

    /**
     * Constants related to the beambreak subsystem.
     */
    public static class BeamBreakConstants {
        public static final int ENDEFFECTOR_MIDDLE_BEAMBREAK_ID = 2;
        public static final int ENDEFFECTOR_EDGE_BEAMBREAK_ID = 0;
    }

    /**
     * Constants related to the endeffector subsystem.
     */
    public static class EndEffectorConstants {
        public static final int ENDEFFECTOR_MOTOR_ID = 31;

        public static final int STATOR_CURRENT_LIMIT_AMPS = 60;
        public static final int SUPPLY_CURRENT_LIMIT_AMPS = 20;
        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERT = false;
        public static final double REDUCTION = 1;

        public static final TunableNumber INTAKE_RPS = new TunableNumber("ENDEFFECTOR/indexRPS", -40);
        public static final TunableNumber HOLD_RPS = new TunableNumber("ENDEFFECTOR/holdRPS", 0.0);
        public static final TunableNumber PRE_SHOOT_RPS = new TunableNumber("ENDEFFECTOR/preShootRPS", -15);
        public static final TunableNumber SHOOT_RPS = new TunableNumber("ENDEFFECTOR/shootRPS", -80);
        public static final TunableNumber IDLE_RPS = new TunableNumber("ENDEFFECTOR/idleRPS", -0);

        /**
         * Constants for the endeffector motor gains.
         */
        public static class EndEffectorGainsClass {
            public static final TunableNumber ENDEFFECTOR_KP = new TunableNumber("ENDEFFECTOR PID/kp", 0.2);
            public static final TunableNumber ENDEFFECTOR_KI = new TunableNumber("ENDEFFECTOR PID/ki", 0);
            public static final TunableNumber ENDEFFECTOR_KD = new TunableNumber("ENDEFFECTOR PID/kd",
                    0.001);
            public static final TunableNumber ENDEFFECTOR_KA = new TunableNumber("ENDEFFECTOR PID/ka",
                    0.0037512677);
            public static final TunableNumber ENDEFFECTOR_KV = new TunableNumber("ENDEFFECTOR PID/kv",
                    0.113);// 0.107853495
            public static final TunableNumber ENDEFFECTOR_KS = new TunableNumber("ENDEFFECTOR PID/ks",
                    0.28475008);
        }
    }

    /**
     * Constants related to the robot's intake subsystem.
     */
    public static class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 15;
        public static final int INTAKE_PIVOT_MOTOR_ID = 16;
        //Constants for intake roller
        public static final int STATOR_CURRENT_LIMIT_AMPS = 60;
        public static final int SUPPLY_CURRENT_LIMIT_AMPS = 20;
        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERT = false;
        public static final double REDUCTION = 1;
        public static final double moi = 0;//inertia for simulation
        public static final double ROLLER_RATIO = 1;
        public static final double INTAKE_DANGER_ZONE = 90;
        public static final TunableNumber ROLLER_AMPS_HAS_CORAL = new TunableNumber("INTAKE_ROLLER/rollerAmpsHasCoral",10);
        //Motion constants for intake pivot
        public static final TunableNumber INTAKE_PIVOT_CRUISE_VELOCITY = new TunableNumber("INTAKE_PIVOT/cruiseVelocity", 100);
        public static final TunableNumber INTAKE_PIVOT_ACCELERATION = new TunableNumber("INTAKE_PIVOT/acceleration", 500);
        public static final TunableNumber INTAKE_PIVOT_JERK = new TunableNumber("INTAKE_PIVOT/jerk", 0);
        public static final TunableNumber DEPLOY_ANGLE = new TunableNumber("INTAKE_PIVOT/deployAngle", 113);
        public static final TunableNumber OUTTAKE_ANGLE = new TunableNumber("INTAKE_PIVOT/outtakeAngle", 105);
        public static final TunableNumber HOME_ANGLE = new TunableNumber("INTAKE_PIVOT/homeAngle", 5);
        public static final TunableNumber FUNNEL_AVOID_ANGLE = new TunableNumber("INTAKE_PIVOT/funnelAvoidAngle", 90);
        //Motion constants for intake roller
        public static final TunableNumber INTAKE_VOLTAGE = new TunableNumber("INTAKE_ROLLER/intakeVoltage", 12.0);
        public static final TunableNumber OUTTAKE_VOLTAGE = new TunableNumber("INTAKE_ROLLER/outtakeVoltage", -6.0);
        //Constants for intake pivot
        public static double PIVOT_RATIO = 36 * 50 / 11;

        /**
         * Constants for the intake pivot motor gains in the intake subsystem.
         */
        public static class IntakePivotGainsClass {
            public static final TunableNumber INTAKE_PIVOT_KP = new TunableNumber("INTAKE_PIVOT PID/kp", 50);
            public static final TunableNumber INTAKE_PIVOT_KI = new TunableNumber("INTAKE_PIVOT PID/ki", 0);
            public static final TunableNumber INTAKE_PIVOT_KD = new TunableNumber("INTAKE_PIVOT PID/kd", 0);
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
        public static final int ELEVATOR_MOTOR_ID = 50;

        public static final double ELEVATOR_SPOOL_DIAMETER = 0.04 + 0.003; //0.04m for spool diameter, 0.003 for rope diameter
        public static final double ELEVATOR_GEAR_RATIO = 3.0;
        public static final double ELEVATOR_DANGER_ZONE = 0.4180619200456253;
        public static final double ELEVATOR_DEFAULT_POSITION_WHEN_DISABLED = 0.46;

        public static final TunableNumber motionAcceleration = new TunableNumber("Elevator/MotionAcceleration",
                200);
        public static final TunableNumber motionCruiseVelocity = new TunableNumber("Elevator/MotionCruiseVelocity",
                40);
        public static final TunableNumber motionJerk = new TunableNumber("Elevator/MotionJerk",
                0.0);
        public static final TunableNumber MAX_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/max",
                1.57);
        public static final TunableNumber IDLE_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/idle",
                0.7);
        public static final TunableNumber HOME_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/HOME",
                0.01);
        public static final TunableNumber L1_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/L1",
                0.45);
        public static final TunableNumber L2_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/L2",
                0.58);
        public static final TunableNumber L3_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/L3",
                1.01);// 0.107853495
        public static final TunableNumber L4_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/L4",
                1.625);
        public static final TunableNumber P1_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/P1",
                0.50);
        public static final TunableNumber P2_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/P2",
                0.80);
        public static final TunableNumber FUNNEL_INTAKE_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/FunnnelIntake",
                0.35);
        public static final TunableNumber ELEVATOR_ZEROING_CURRENT = new TunableNumber("Elevator zeroing current",
                20);
        public static final double ELEVATOR_MIN_SAFE_HEIGHT = 0.5;

    }


    /**
     * Constants for the elevator motor gains.
     */
    public static class ElevatorGainsClass {
        public static final TunableNumber ELEVATOR_KP = new TunableNumber("ELEVATOR PID/kp", 15);
        public static final TunableNumber ELEVATOR_KI = new TunableNumber("ELEVATOR PID/ki", 0);
        public static final TunableNumber ELEVATOR_KD = new TunableNumber("ELEVATOR PID/kd", 0);
        public static final TunableNumber ELEVATOR_KA = new TunableNumber("ELEVATOR PID/ka",
                0);
        public static final TunableNumber ELEVATOR_KV = new TunableNumber("ELEVATOR PID/kv", 0);// 0.107853495
        public static final TunableNumber ELEVATOR_KS = new TunableNumber("ELEVATOR PID/ks",
                0);
        public static final TunableNumber ELEVATOR_KG = new TunableNumber("ELEVATOR PID/kg", 0.45);//0.3
    }
}
