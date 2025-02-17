// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.basics.AutoActions;
import frc.robot.commands.*;
import frc.robot.display.Display;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIONorthstar;
import frc.robot.subsystems.beambreak.BeambreakIOReal;
import frc.robot.subsystems.beambreak.BeambreakIOSim;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorIOReal;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;
import org.frcteam6941.looper.UpdateManager;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.AllianceFlipUtil;

import java.io.IOException;

import static frc.robot.RobotConstants.ElevatorConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // flag states
    public static boolean elevatorIsDanger;
    public static boolean intakeIsDanger;
    public static boolean intakeIsAvoiding;
    public static double elevatorTargetPosition = L3_EXTENSION_METERS.get(); // Used for storing an elevator target position to be used with commands like PutCoralCommand
    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final CommandXboxController testerController = new CommandXboxController(2);
    private final CommandGenericHID streamDeckController = new CommandGenericHID(3);
    // Update Manager
    @Getter
    private final UpdateManager updateManager;
    // Subsystems
    private final AprilTagVision aprilTagVision = new AprilTagVision(
            this::getAprilTagLayoutType,
            new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 0),
            new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 1));
    private final Swerve swerve = Swerve.getInstance();
    private final Display display = Display.getInstance();
    private final ElevatorSubsystem elevatorSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ClimberSubsystem climberSubsystem;
    private double lastResetTime = 0.0;


    public RobotContainer() {
        if (RobotBase.isReal()) {
            elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOReal());
            endEffectorSubsystem = new EndEffectorSubsystem(new EndEffectorIOReal(), new BeambreakIOReal(RobotConstants.BeamBreakConstants.ENDEFFECTOR_MIDDLE_BEAMBREAK_ID), new BeambreakIOReal(RobotConstants.BeamBreakConstants.ENDEFFECTOR_EDGE_BEAMBREAK_ID));
            intakeSubsystem = new IntakeSubsystem(new IntakePivotIOReal(), new IntakeRollerIOReal());
            climberSubsystem = new ClimberSubsystem(new ClimberIOReal());
        } else {
            elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
            endEffectorSubsystem = new EndEffectorSubsystem(new EndEffectorIOSim(), new BeambreakIOSim(RobotConstants.BeamBreakConstants.ENDEFFECTOR_MIDDLE_BEAMBREAK_ID), new BeambreakIOSim(RobotConstants.BeamBreakConstants.ENDEFFECTOR_EDGE_BEAMBREAK_ID));
            intakeSubsystem = new IntakeSubsystem(new IntakePivotIOSim(), new IntakeRollerIOSim());
            climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
        }
        updateManager = new UpdateManager(swerve,
                display);
        updateManager.registerAll();

        new Trigger(RobotController::getUserButton).whileTrue(new ClimbResetCommand(climberSubsystem));

        configureDriverBindings();
        configureOperatorBindings();
        configureTesterBindings();
        configureStreamDeckBindings();
    }

    //Configure all commands for driver
    private void configureDriverBindings() {
        swerve.setDefaultCommand(Commands.runOnce(() -> swerve.drive(
                new Translation2d(
                        Math.abs(driverController.getLeftY()) < RobotConstants.SwerveConstants.deadband ?
                                0 : -driverController.getLeftY() * RobotConstants.SwerveConstants.maxSpeed.magnitude(),
                        Math.abs(driverController.getLeftX()) < RobotConstants.SwerveConstants.deadband ?
                                0 : -driverController.getLeftX() * RobotConstants.SwerveConstants.maxSpeed.magnitude()),
                Math.abs(-driverController.getRightX()) < RobotConstants.SwerveConstants.rotationalDeadband ?
                        0 : -driverController.getRightX() * RobotConstants.SwerveConstants.maxAngularRate.magnitude(),
                true,
                false), swerve));

        driverController.start().onTrue(
                Commands.runOnce(() -> {
                    /*
                        TODO: the reset command will be activated twice when the start button is pressed only once,
                        this is only a temporary solution to avoid execute the command twice within 0.01s,
                        please fix the bug
                    */
                    if (Timer.getFPGATimestamp() - lastResetTime > 0.01) {
                        swerve.resetHeadingController();
                        swerve.resetPose(
                                AllianceFlipUtil.apply(new Pose2d(
                                        new Translation2d(0, 0),
                                        swerve.getLocalizer().getLatestPose().getRotation())));
                    }
                    lastResetTime = Timer.getFPGATimestamp();
                    aprilTagVision.setMeasuerCnt(0);
                }).ignoringDisable(true));


        driverController.leftBumper().whileTrue(new GroundIntakeCommand(intakeSubsystem, endEffectorSubsystem, elevatorSubsystem));
        driverController.leftTrigger().whileTrue(new PutCoralCommand(driverController, endEffectorSubsystem, elevatorSubsystem, intakeSubsystem));
        driverController.rightBumper().whileTrue(new FunnelIntakeCommand(elevatorSubsystem, endEffectorSubsystem, intakeSubsystem));
        driverController.y().onTrue(new ZeroCommand(elevatorSubsystem, intakeSubsystem, endEffectorSubsystem, climberSubsystem));
        driverController.povDown().whileTrue(new ClimbCommand(climberSubsystem, elevatorSubsystem, intakeSubsystem, endEffectorSubsystem));
    }

    private void configureOperatorBindings() {
        //Operator's triggers to change target reef heights

        operatorController.b().onTrue(Commands.runOnce(() -> elevatorTargetPosition = L2_EXTENSION_METERS.get()));
        operatorController.x().onTrue(Commands.runOnce(() -> elevatorTargetPosition = L3_EXTENSION_METERS.get()));

    }

    private void configureTesterBindings() {
        //test of endeffector state machine
        testerController.povLeft().onTrue(Commands.runOnce(() -> endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.FUNNEL_INTAKE)));
        testerController.povRight().onTrue(Commands.runOnce(() -> endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.GROUND_INTAKE)));
        testerController.povUp().onTrue(Commands.runOnce(() -> endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.SHOOT)));

        //test of elevator state machine
        testerController.a().onTrue(Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(L1_EXTENSION_METERS.get())));
        testerController.b().onTrue(Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(L2_EXTENSION_METERS.get())));
        testerController.x().onTrue(Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(L3_EXTENSION_METERS.get())));
        testerController.y().onTrue(Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(L4_EXTENSION_METERS.get())));

        //test of intake states
        testerController.rightBumper().onTrue((Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_INTAKE))));
        testerController.leftBumper().onTrue((Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.GROUNDZERO))));
        testerController.rightTrigger().onTrue((Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.FUNNEL_AVOID))));
        testerController.leftTrigger().onTrue((Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME))));
        testerController.povUp().onTrue((Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.TREMBLE_INTAKE))));
        testerController.povDown().onTrue((Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.OUTTAKE))));

    }

    private void configureStreamDeckBindings() {
        streamDeckController.button(1).onTrue(new ReefAimCommand(7, false, () -> streamDeckController.button(17).getAsBoolean()));
    }

    public Command getAutonomousCommand() throws IOException, ParseException {
        return new SequentialCommandGroup(
                AutoActions.waitFor(0.000001), // FIXME the first auto action will not be run
                AutoActions.followTrajectory(AutoActions.getTrajectory("T_4"), true, true)
        );
    }

    public FieldConstants.AprilTagLayoutType getAprilTagLayoutType() {
        return FieldConstants.defaultAprilTagType;
    }

}