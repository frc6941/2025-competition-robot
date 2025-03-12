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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.basics.AutoActions;
import frc.robot.auto.basics.AutoFile;
import frc.robot.auto.basics.CustomAutoChooser;
import frc.robot.commands.*;
import frc.robot.display.Display;
import frc.robot.drivers.DestinationSupplier;
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
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorIOARGB;
import frc.robot.subsystems.indicator.IndicatorIOSim;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;
import org.frcteam6941.looper.UpdateManager;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
    public static boolean intakeHasCoral = false;

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandGenericHID streamDeckController = new CommandGenericHID(1);
    private final CommandXboxController testerController = new CommandXboxController(2);
    // Update Manager
    @Getter
    private final UpdateManager updateManager;
    // Subsystems
    private final AprilTagVision aprilTagVision = new AprilTagVision(
            this::getAprilTagLayoutType,
            new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 0),
            new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 1),
            new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 2),
            new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 3));
    private final Swerve swerve = Swerve.getInstance();
    private final Display display = Display.getInstance();
    private final DestinationSupplier destinationSupplier = DestinationSupplier.getInstance();
    private final ElevatorSubsystem elevatorSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ClimberSubsystem climberSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final Limelight limelight;
    @Getter
    private final LoggedDashboardChooser<String> autoChooser;
    private final AutoActions autoActions;
    private final AutoFile autoFile;
    private double lastResetTime = 0.0;


    public RobotContainer() {
        if (RobotBase.isReal()) {
            indicatorSubsystem = new IndicatorSubsystem(new IndicatorIOARGB());
            elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOReal());
            endEffectorSubsystem = new EndEffectorSubsystem(new EndEffectorIOReal(), new BeambreakIOReal(RobotConstants.BeamBreakConstants.ENDEFFECTOR_MIDDLE_BEAMBREAK_ID), new BeambreakIOReal(RobotConstants.BeamBreakConstants.ENDEFFECTOR_EDGE_BEAMBREAK_ID));
            intakeSubsystem = new IntakeSubsystem(new IntakePivotIOReal(), new IntakeRollerIOReal(), new BeambreakIOReal(RobotConstants.BeamBreakConstants.INTAKE_BEAMBREAK_ID));
            climberSubsystem = new ClimberSubsystem(new ClimberIOReal());
            limelight = new Limelight();
        } else {
            indicatorSubsystem = new IndicatorSubsystem(new IndicatorIOSim());
            elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
            endEffectorSubsystem = new EndEffectorSubsystem(new EndEffectorIOSim(), new BeambreakIOSim(RobotConstants.BeamBreakConstants.ENDEFFECTOR_MIDDLE_BEAMBREAK_ID), new BeambreakIOSim(RobotConstants.BeamBreakConstants.ENDEFFECTOR_EDGE_BEAMBREAK_ID));
            intakeSubsystem = new IntakeSubsystem(new IntakePivotIOSim(), new IntakeRollerIOSim(), new BeambreakIOSim(RobotConstants.BeamBreakConstants.INTAKE_BEAMBREAK_ID));
            climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
            limelight = new Limelight();
        }
        updateManager = new UpdateManager(swerve,
                display, destinationSupplier);
        updateManager.registerAll();

        autoChooser = new LoggedDashboardChooser<>("Chooser", CustomAutoChooser.buildAutoChooser("New Auto"));
        autoActions = new AutoActions(indicatorSubsystem, elevatorSubsystem, endEffectorSubsystem, intakeSubsystem, aprilTagVision);
        autoFile = new AutoFile(autoActions);

        new Trigger(RobotController::getUserButton).whileTrue(new ClimbResetCommand(climberSubsystem));

        configureAutoChooser();
        configureDriverBindings();
        configureStreamDeckBindings();
        configureTesterBindings();
    }

    private void configureAutoChooser() {
        autoChooser.addOption("4CoralLeft", "4CoralLeft");
        autoChooser.addOption("4CoralRight", "4CoralRight");
        autoChooser.addOption("FunnelRight", "FunnelRight");
        autoChooser.addOption("FunnelLeft", "FunnelLeft");
        autoChooser.addOption("Test", "Test");
        autoChooser.addOption("None", "None");
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

//        driverController.povRight().whileTrue(
//                new InstantCommand(() -> swerve.setKinematicsLimit(DRIVETRAIN_LIMITED)).finallyDo(
//                        () -> swerve.setKinematicsLimit(DRIVETRAIN_UNCAPPED)
//                )
//        );

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
                    aprilTagVision.setMeasureCnt(0);
                    indicatorSubsystem.setPattern(IndicatorIO.Patterns.RESET_ODOM);
                }).ignoringDisable(true));

        driverController.povUp().whileTrue(new PreClimbCommand(climberSubsystem, elevatorSubsystem, intakeSubsystem, endEffectorSubsystem));
        driverController.povLeft().whileTrue(new IdleClimbCommand(climberSubsystem, elevatorSubsystem, intakeSubsystem, endEffectorSubsystem));
        driverController.leftTrigger().toggleOnTrue(switchIntakeModeCommand());
        driverController.leftBumper().toggleOnTrue(new FunnelIntakeCommand(indicatorSubsystem, elevatorSubsystem, endEffectorSubsystem, intakeSubsystem));
        driverController.rightBumper().whileTrue(switchPreMoveModeCommand());
        driverController.povDown().onTrue(new ZeroCommand(elevatorSubsystem, intakeSubsystem, endEffectorSubsystem));
        driverController.a().whileTrue(new PokeCommand(endEffectorSubsystem, intakeSubsystem, elevatorSubsystem));
        driverController.b().toggleOnTrue(new GroundOuttakeCommand(intakeSubsystem, endEffectorSubsystem, elevatorSubsystem));
        driverController.x().onTrue(Commands.runOnce(() -> intakeSubsystem.setLowerAngle(true))).onFalse(Commands.runOnce(() -> intakeSubsystem.setLowerAngle(false)));
        driverController.y().whileTrue(new ClimbCommand(climberSubsystem, elevatorSubsystem, intakeSubsystem, endEffectorSubsystem));
        driverController.back().whileTrue(switchAimingModeCommand());
        driverController.leftStick().onTrue(Commands.runOnce(() -> destinationSupplier.updateBranch(false)).ignoringDisable(true));
        driverController.rightStick().onTrue(Commands.runOnce(() -> destinationSupplier.updateBranch(true)).ignoringDisable(true));
    }

    private void configureStreamDeckBindings() {
        // Operator's controller
        streamDeckController.button(1).onTrue(Commands.runOnce(() -> destinationSupplier.setCurrentControlMode(DestinationSupplier.controlMode.MANUAL)).ignoringDisable(true));
        streamDeckController.button(3).onTrue(Commands.runOnce(() -> destinationSupplier.setCurrentControlMode(DestinationSupplier.controlMode.AUTO)).ignoringDisable(true));
        streamDeckController.button(4).onTrue(Commands.runOnce(() -> destinationSupplier.updateBranch(false)).ignoringDisable(true));
        streamDeckController.button(5).onTrue(Commands.runOnce(() -> destinationSupplier.updateBranch(true)).ignoringDisable(true));
        streamDeckController.button(13).onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L1)).ignoringDisable(true));
        streamDeckController.button(14).onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L2)).ignoringDisable(true));
        streamDeckController.button(15).onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L3)).ignoringDisable(true));
        streamDeckController.button(16).onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L4)).ignoringDisable(true));
        streamDeckController.button(18).onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.P1)).ignoringDisable(true));
        streamDeckController.button(19).onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.P2)).ignoringDisable(true));
        streamDeckController.button(8).whileTrue(Commands.run(() -> destinationSupplier.setCurrentL1Mode(DestinationSupplier.L1Mode.INTAKE))
                .finallyDo(() -> destinationSupplier.setCurrentL1Mode(DestinationSupplier.L1Mode.ELEVATOR)).ignoringDisable(true));
        streamDeckController.button(10).whileTrue(Commands.run(() -> destinationSupplier.setCurrentIntakeMode(DestinationSupplier.IntakeMode.TREMBLE))
                .finallyDo(() -> destinationSupplier.setCurrentIntakeMode(DestinationSupplier.IntakeMode.NORMAL)).ignoringDisable(true));
    }

    public void configureTesterBindings() {
        testerController.a().whileTrue(Commands.run(() -> destinationSupplier.setCurrentL1Mode(DestinationSupplier.L1Mode.INTAKE))
                .finallyDo(() -> destinationSupplier.setCurrentL1Mode(DestinationSupplier.L1Mode.ELEVATOR)).ignoringDisable(true));
        testerController.b().onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L2)));
        testerController.x().onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L3)));
        testerController.y().onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L4)));



        testerController.rightBumper().onTrue(Commands.runOnce(() -> destinationSupplier.setCurrentControlMode(DestinationSupplier.controlMode.MANUAL)).ignoringDisable(true));
        testerController.leftBumper().onTrue(Commands.runOnce(() -> destinationSupplier.setCurrentControlMode(DestinationSupplier.controlMode.AUTO)).ignoringDisable(true));
        testerController.povUp().onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.P2)).ignoringDisable(true));
        testerController.povDown().onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.P1)).ignoringDisable(true));
    }

    public Command getAutonomousCommand() {
        // FIXME: set resetOdometry to false when vision is completed and usable
        //return autoFile.runAuto("4CoralLeft");
        return autoFile.runAuto(autoChooser.get());
    }

    public FieldConstants.AprilTagLayoutType getAprilTagLayoutType() {
        return FieldConstants.defaultAprilTagType;
    }

    public Command switchAimingModeCommand() {
        return new ConditionalCommand(
                // AUTO
                new AutoAimShootCommand(
                        indicatorSubsystem, endEffectorSubsystem, elevatorSubsystem, intakeSubsystem,
                        () -> false, driverController),
                // MANUAL
                new ReefAimCommand(() -> false, elevatorSubsystem, driverController, indicatorSubsystem),
                () -> destinationSupplier.getCurrentControlMode() == DestinationSupplier.controlMode.AUTO);
    }

    public Command switchIntakeModeCommand() {
        return new ConditionalCommand(
                new GroundIntakeCommand(indicatorSubsystem, intakeSubsystem, endEffectorSubsystem, elevatorSubsystem),
                new HoldIntakeCommand(indicatorSubsystem, intakeSubsystem, elevatorSubsystem),
                () -> destinationSupplier.getL1Mode() == DestinationSupplier.L1Mode.ELEVATOR);
    }

    public Command switchPreMoveModeCommand() {
        return new ConditionalCommand(
                // Intake L1
                new ShootHoldCommand(intakeSubsystem, () -> driverController.rightTrigger().getAsBoolean()),
                // Elevator
                new PutCoralCommand(driverController, endEffectorSubsystem, elevatorSubsystem, intakeSubsystem, indicatorSubsystem),
                () -> destinationSupplier.getL1Mode() == DestinationSupplier.L1Mode.INTAKE);
    }

    public void setMegaTag2 (boolean setMegaTag2) {
        limelight.setMegaTag2(setMegaTag2);
    }

}
