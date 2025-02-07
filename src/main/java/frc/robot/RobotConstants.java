// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
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
                public static final double deadband = maxSpeed.magnitude() * 0.05;
                public static final double rotationalDeadband = maxAngularRate.magnitude() * 0.05;

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
                private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 8;
                private static final int FRONT_LEFT_STEER_MOTOR_ID = 7;
                private static final int FRONT_LEFT_ENCODER_ID = 20;
                private static final double FRONT_LEFT_ENCODER_OFFSET = 0.4609375; // -0.37451171875;//
                                                                                   // 0.052955;//0.127686//0.5329550781
                private static final Measure<DistanceUnit> frontLeftXPos = Meters.of(0.127);
                private static final Measure<DistanceUnit> frontLeftYPos = Meters.of(0.247);
                public static final LegacySwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                                FRONT_LEFT_STEER_MOTOR_ID, FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_ENCODER_ID,
                                FRONT_LEFT_ENCODER_OFFSET, frontLeftXPos.magnitude(), frontLeftYPos.magnitude(), false);
                // Front Right
                private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
                private static final int FRONT_RIGHT_STEER_MOTOR_ID = 1;
                private static final int FRONT_RIGHT_ENCODER_ID = 0;
                private static final double FRONT_RIGHT_ENCODER_OFFSET = -0.3500976; // -0.119385;//
                                                                                     // 0.125685;//0.13623046875//0.117686//0.046875
                private static final Measure<DistanceUnit> frontRightXPos = Meters.of(0.127);
                private static final Measure<DistanceUnit> frontRightYPos = Meters.of(-0.247);
                public static final LegacySwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                                FRONT_RIGHT_STEER_MOTOR_ID, FRONT_RIGHT_DRIVE_MOTOR_ID, FRONT_RIGHT_ENCODER_ID,
                                FRONT_RIGHT_ENCODER_OFFSET, frontRightXPos.magnitude(), frontRightYPos.magnitude(),
                                true);
                // Back Left
                private static final int BACK_LEFT_DRIVE_MOTOR_ID = 6;
                private static final int BACK_LEFT_STEER_MOTOR_ID = 5;
                private static final int BACK_LEFT_ENCODER_ID = 11;
                private static final double BACK_LEFT_ENCODER_OFFSET = -0.420166; // -0.310302734375;//
                                                                                  // 0.773925;//-0.223//0.401611//0.77392578125
                private static final Measure<DistanceUnit> backLeftXPos = Meters.of(-0.180);
                private static final Measure<DistanceUnit> backLeftYPos = Meters.of(0.247);
                public static final LegacySwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                                BACK_LEFT_STEER_MOTOR_ID, BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_ENCODER_ID,
                                BACK_LEFT_ENCODER_OFFSET, backLeftXPos.magnitude(), backLeftYPos.magnitude(), false);
                // Back Right
                private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 4;
                private static final int BACK_RIGHT_STEER_MOTOR_ID = 3;
                private static final int BACK_RIGHT_ENCODER_ID = 10;
                private static final double BACK_RIGHT_ENCODER_OFFSET = -0.458007;// -0.151611;//
                                                                                  // 0.422119;//-0.5684550781//-0.064453//0.432279296875
                private static final Measure<DistanceUnit> backRightXPos = Meters.of(-0.180);
                private static final Measure<DistanceUnit> backRightYPos = Meters.of(-0.247);
                public static final LegacySwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                                BACK_RIGHT_STEER_MOTOR_ID, BACK_RIGHT_DRIVE_MOTOR_ID, BACK_RIGHT_ENCODER_ID,
                                BACK_RIGHT_ENCODER_OFFSET, backRightXPos.magnitude(), backRightYPos.magnitude(), true);
                // swervemodule
                public static LegacySwerveModuleConstants[] modules = { FrontLeft, FrontRight, BackLeft, BackRight };
                public static final Translation2d[] modulePlacements = new Translation2d[] {
                                new Translation2d(SwerveConstants.FrontLeft.LocationX,
                                                SwerveConstants.FrontLeft.LocationY),
                                new Translation2d(SwerveConstants.FrontRight.LocationX,
                                                SwerveConstants.FrontRight.LocationY),
                                new Translation2d(SwerveConstants.BackLeft.LocationX,
                                                SwerveConstants.BackLeft.LocationY),
                                new Translation2d(SwerveConstants.BackRight.LocationX,
                                                SwerveConstants.BackRight.LocationY) };

                /**
                 * Constants for the heading controller used in the swerve drivetrain.
                 */
                public static class headingController {
                        public static final frc.robot.utils.TunableNumber HEADING_KP = new frc.robot.utils.TunableNumber(
                                        "HEADING PID/kp", 0.09);
                        public static final frc.robot.utils.TunableNumber HEADING_KI = new frc.robot.utils.TunableNumber(
                                        "HEADING PID/ki", 0.000);
                        public static final frc.robot.utils.TunableNumber HEADING_KD = new frc.robot.utils.TunableNumber(
                                        "HEADING PID/kd", 0.004);
                        public static final frc.robot.utils.TunableNumber MAX_ERROR_CORRECTION_ANGLE = new frc.robot.utils.TunableNumber(
                                        "HEADING/Max Error Correction Angle", 120.0);
                }

                /**
                 * Constants for the steer motor gains in the swerve drivetrain.
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
                 */
                public static class driveGainsClass {
                        public static final TunableNumber DRIVE_KP = new TunableNumber("DRIVE PID/kp", 0.03);
                        public static final TunableNumber DRIVE_KI = new TunableNumber("DRIVE PID/ki", 0);
                        public static final TunableNumber DRIVE_KD = new TunableNumber("DRIVE PID/kd", 0.0001);
                        public static final TunableNumber DRIVE_KA = new TunableNumber("DRIVE PID/ka", 0);
                        public static final TunableNumber DRIVE_KV = new TunableNumber("DRIVE PID/kv", 0.12);
                        public static final TunableNumber DRIVE_KS = new TunableNumber("DRIVE PID/ks", 0.045);
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
                public static final int ENDEFFECTOR_MIDDLE_BEAMBREAK_ID = 1;
                public static final int ENDEFFECTOR_EDGE_BEAMBREAK_ID = 3;
                public static final int INTAKER_BEAMBREAK_ID = 0;
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

                public static final TunableNumber INTAKE_RPS = new TunableNumber("EndEffector/indexRPS", -40);
                public static final TunableNumber TRANSFER_RPS = new TunableNumber("EndEffector/transferRPS", -25);
                public static final TunableNumber HOLD_RPS = new TunableNumber("EndEffector/holdRPS", 0.0);
                public static final TunableNumber SHOOT_RPS = new TunableNumber("EndEffector/shootRPS", -80);
                public static final TunableNumber IDLE_RPS = new TunableNumber("EndEffector/idleRPS", -0);

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
        public static class ClimberConstants {
                public static final int CLIMBER_MOTOR_ID = 0;

                public static final int STATOR_CURRENT_LIMIT_AMPS = 60;
                public static final int SUPPLY_CURRENT_LIMIT_AMPS = 20;
                public static final boolean IS_BRAKE = true;
                public static final boolean IS_INVERT = false;
                public static final double REDUCTION = 1;
                public static double CLIMBER_RATIO = 1;
                public static Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(60);
                public static Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);

                public static final TunableNumber CLIMB_RPS = new TunableNumber("Climber/climbRPS", 30);
                public static final TunableNumber DOWN_RPS = new TunableNumber("Climber/climbRPS", -30);
                public static final TunableNumber HOLD_RPS = new TunableNumber("Climber/holdRPS", 0.0);
                public static final TunableNumber IDLE_RPS = new TunableNumber("Climber/idleRPS", -0);

                /**
                 * Constants for the Climber motor gains.
                 */
                public static class ClimberGainsClass {
                        public static final TunableNumber CLIMBER_KP = new TunableNumber("Climber PID/kp", 0.2);
                        public static final TunableNumber CLIMBER_KI = new TunableNumber("Climber PID/ki", 0);
                        public static final TunableNumber CLIMBER_KD = new TunableNumber("Climber PID/kd",
                                        0.001);
                        public static final TunableNumber CLIMBER_KA = new TunableNumber("Climber PID/ka",
                                        0.0037512677);
                        public static final TunableNumber CLIMBER_KV = new TunableNumber("Climber PID/kv",
                                        0.113);// 0.107853495
                        public static final TunableNumber CLIMBER_KS = new TunableNumber("Climber PID/ks",
                                        0.28475008);
                }
        }
        /**
         * Constants related to the robot's intake subsystem.
         */
        public static class intakeConstants {
                public static final int STATOR_CURRENT_LIMIT_AMPS = 60;
                public static final int SUPPLY_CURRENT_LIMIT_AMPS = 20;
                public static final boolean IS_BRAKE = true;
                public static final boolean IS_INVERT = false;
                public static final double REDUCTION = 1;
                public static final int INTAKER_MOTOR_ID = 15;
                public static final int INTAKER_PIVOT_MOTOR_ID = 16;
                public static double PIVOT_RATIO = 1;
                public static Rotation2d RETRACTED_ANGLE = Rotation2d.fromDegrees(90);
                public static Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(120);
                public static Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);

                public static final TunableNumber INTAKE_VELOCITY = new TunableNumber("Intake/intakeVelocity", 600);
                public static final TunableNumber OUTTAKE_VELOCITY = new TunableNumber("Intake/outtakeVelocity", -600);
                public static final TunableNumber INTAKE_VOLTAGE = new TunableNumber("Intake/intakeVoltage", 3.0);
                public static final TunableNumber INTAKE_IDLE_RPS = new TunableNumber("Intake/intakeIdleRPS", 0.0);

                /**
                 * Constants for the intake pivot motor gains in the intake subsystem.
                 */
                public static class intakePivotGainsClass {
                        public static final TunableNumber INTAKE_PIVOT_KP = new TunableNumber("INTAKE_PIVOT PID/kp",
                                        0.03);
                        public static final TunableNumber INTAKE_PIVOT_KI = new TunableNumber("INTAKE_PIVOT PID/ki", 0);
                        public static final TunableNumber INTAKE_PIVOT_KD = new TunableNumber("INTAKE_PIVOT PID/kd",
                                        0.0001);
                        public static final TunableNumber INTAKE_PIVOT_KA = new TunableNumber("INTAKE_PIVOT PID/ka", 0);
                        public static final TunableNumber INTAKE_PIVOT_KV = new TunableNumber("INTAKE_PIVOT PID/kv",
                                        0.12);
                        public static final TunableNumber INTAKE_PIVOT_KS = new TunableNumber("INTAKE_PIVOT PID/ks",
                                        0.045);
                }

                /**
                 * Constants for the intake motor gains in the intake subsystem.
                 */
                public static class intakeGainsClass {
                        public static final TunableNumber INTAKE_KP = new TunableNumber("INTAKE PID/kp", 0.03);
                        public static final TunableNumber INTAKE_KI = new TunableNumber("INTAKE PID/ki", 0);
                        public static final TunableNumber INTAKE_KD = new TunableNumber("INTAKE PID/kd", 0.0001);
                        public static final TunableNumber INTAKE_KA = new TunableNumber("INTAKE PID/ka", 0);
                        public static final TunableNumber INTAKE_KV = new TunableNumber("INTAKE PID/kv", 0.12);
                        public static final TunableNumber INTAKE_KS = new TunableNumber("INTAKE PID/ks", 0.045);
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
                public static final double DEADZONE_DISTANCE = 0.02;

                public static final TunableNumber motionAcceleration = new TunableNumber("Elevator/MotionAcceleration",
                                20.0);
                public static final TunableNumber motionCruiseVelocity = new TunableNumber(
                                "Elevator/MotionCruiseVelocity", 20.0);
                public static final TunableNumber motionJerk = new TunableNumber("Elevator/MotionJerk", 0.0);

                public static final TunableNumber MAX_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/max",
                                2);
                public static final TunableNumber IDLE_EXTENSION_METERS = new TunableNumber(
                                "ELEVATOR GOALS/idle", 0);
                public static final TunableNumber L1_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/L1",
                                0.5);
                public static final TunableNumber L2_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/L2",
                                0.8);
                public static final TunableNumber L3_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/L3",
                                1.1);// 0.107853495
                public static final TunableNumber L4_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/L4",
                                1.4);
                public static final TunableNumber ELEVATOR_ZEROING_CURRENT = new TunableNumber("Elevator zeroing current",
                        15);
                public static final TunableNumber FUNNEL_INTAKE_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/FUNNEL_INTAKE",
                        0.47);
                public static final TunableNumber GROUND_INTAKE_EXTENSION_METERS = new TunableNumber("ELEVATOR SETPOINTS/GROUND_INTAKE",
                        0.1);

        }


                /**
                 * Constants for the elevator motor gains.
                 */
                public static class ElevatorGainsClass {
                        public static final TunableNumber ELEVATOR_KP = new TunableNumber("ELEVATOR PID/kp", 0.2);
                        public static final TunableNumber ELEVATOR_KI = new TunableNumber("ELEVATOR PID/ki", 0);
                        public static final TunableNumber ELEVATOR_KD = new TunableNumber("ELEVATOR PID/kd", 0.001);
                        public static final TunableNumber ELEVATOR_KA = new TunableNumber("ELEVATOR PID/ka",
                                        0.0037512677);
                        public static final TunableNumber ELEVATOR_KV = new TunableNumber("ELEVATOR PID/kv", 0.113);// 0.107853495
                        public static final TunableNumber ELEVATOR_KS = new TunableNumber("ELEVATOR PID/ks",
                                        0.28475008);
                }
        }
