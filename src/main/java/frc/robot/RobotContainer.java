// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.auto.basics.AutoActions;
import frc.robot.commands.ElevatorZeroingCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.display.Display;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIONorthstar;
import frc.robot.subsystems.beambreak.BeambreakIOReal;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorIOReal;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.AllianceFlipUtil;
import lombok.Getter;
import org.frcteam6941.looper.UpdateManager;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.RobotConstants.BeamBreakConstants.ENDEFFECTOR_EDGE_BEAMBREAK_ID;
import static frc.robot.RobotConstants.BeamBreakConstants.ENDEFFECTOR_MIDDLE_BEAMBREAK_ID;
import static frc.robot.RobotConstants.ElevatorConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    @Getter
    private final UpdateManager updateManager;
    CommandXboxController driverController = new CommandXboxController(0);
    CommandXboxController operatorController = new CommandXboxController(1);
    CommandXboxController testerController = new CommandXboxController(2);
    CommandGenericHID streamDeckController = new CommandGenericHID(3);
    double lastResetTime = 0.0;

    // The robot's subsystems and commands are defined here...
    AprilTagVision aprilTagVision = new AprilTagVision(
            this::getAprilTagLayoutType,
            new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 0),
            new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 1));
    Swerve swerve = Swerve.getInstance();
    Display display = Display.getInstance();
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOReal());
    EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem(new EndEffectorIOReal(), new BeambreakIOReal(ENDEFFECTOR_MIDDLE_BEAMBREAK_ID), new BeambreakIOReal(ENDEFFECTOR_EDGE_BEAMBREAK_ID));
    ClimberSubsystem  climberSubsystem = new ClimberSubsystem(new ClimberIOReal());

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        updateManager = new UpdateManager(swerve,
                display);
        updateManager.registerAll();

        configureDriverBindings(driverController);
        configureOperatorBindings(operatorController);
        configureTesterBindings(testerController);
        configureStreamDeckBindings(streamDeckController);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link CommandPS4Controller
     * PS4} controllers or {@link CommandJoystick Flight
     * joysticks}.
     */

    //Configure all commands for driver
    private void configureDriverBindings(CommandXboxController driverController) {
        swerve.setDefaultCommand(Commands
                .runOnce(() -> swerve.drive(
                                new Translation2d(
                                        -driverController.getLeftY()
                                                * RobotConstants.SwerveConstants.maxSpeed.magnitude(),
                                        -driverController.getLeftX()
                                                * RobotConstants.SwerveConstants.maxSpeed.magnitude()),
                                -driverController.getRightX()
                                        * RobotConstants.SwerveConstants.maxAngularRate.magnitude(),
                                true,
                                false),
                        swerve));

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
                                new Pose2d(
                                        AllianceFlipUtil.apply(
                                                new Translation2d(0, 0)),
                                        swerve.getLocalizer().getLatestPose().getRotation()));
                    }
                    lastResetTime = Timer.getFPGATimestamp();
                }).ignoringDisable(true));
    }

    //Configure all commands for operator
    private void configureOperatorBindings(CommandXboxController operatorController) {

    }

    //Configure all commands for testing
    private void configureTesterBindings(CommandXboxController controller) {
        //test of endeffector state machine
        // new Trigger(controller.leftBumper())
        //         .onTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.INTAKE_CORAL_FUNNEL));
        // new Trigger(controller.rightBumper())
        //         .onTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.SHOOT_CORAL));
        // new Trigger(controller.start())
        //         .onTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.STOPPED));
        //test of elevator heights
        controller.a().onTrue(Commands.runOnce(() -> elevatorSubsystem.setPosition(L1_EXTENSION_METERS.get()), elevatorSubsystem).until(() -> elevatorSubsystem.isAtSetpoint(L1_EXTENSION_METERS.get())));
        controller.b().onTrue(Commands.runOnce(() -> elevatorSubsystem.setPosition(L2_EXTENSION_METERS.get()), elevatorSubsystem).until(() -> elevatorSubsystem.isAtSetpoint(L2_EXTENSION_METERS.get())));
        controller.x().onTrue(Commands.runOnce(() -> elevatorSubsystem.setPosition(L3_EXTENSION_METERS.get()), elevatorSubsystem).until(() -> elevatorSubsystem.isAtSetpoint(L3_EXTENSION_METERS.get())));
        controller.y().onTrue(Commands.runOnce(() -> elevatorSubsystem.setPosition(L4_EXTENSION_METERS.get()), elevatorSubsystem).until(() -> elevatorSubsystem.isAtSetpoint(L4_EXTENSION_METERS.get())));
        controller.povDown().onTrue(new ElevatorZeroingCommand(elevatorSubsystem));

        controller.start().onTrue(Commands.runOnce(() -> climberSubsystem.resetPosition()));
        controller.leftBumper().onTrue(Commands.runOnce(() -> climberSubsystem.setWantedState(ClimberSubsystem.WantedState.CLIMB)));
        controller.rightBumper().onTrue(Commands.runOnce(() -> climberSubsystem.setWantedState(ClimberSubsystem.WantedState.DEPLOY)));
    }

    //Configure all commands for Stream Deck
    private void configureStreamDeckBindings(CommandGenericHID controller) {
        controller.button(1).onTrue(Commands.runOnce(() -> System.out.println("Stream Deck Controller Test Successful!")));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     * @throws ParseException
     * @throws IOException
     * @throws FileVersionException
     */
    public Command getAutonomousCommand() throws FileVersionException, IOException, ParseException {
        // An example command will be run in autonomous
        return new SequentialCommandGroup(
                AutoActions.waitFor(0.000001),
                AutoActions.followTrajectory(AutoActions.getTrajectory("T_4"), true, true)
        );
    }

    private Command rumbleDriver(double seconds) {
        return new RumbleCommand(Seconds.of(seconds), driverController.getHID());
    }

    public FieldConstants.AprilTagLayoutType getAprilTagLayoutType() {
//        if (aprilTagsSpeakerOnly.getAsBoolean()) {
//            return FieldConstants.AprilTagLayoutType.SPEAKERS_ONLY;
//        } else if (aprilTagsAmpOnly.getAsBoolean()) {
//            return FieldConstants.AprilTagLayoutType.AMPS_ONLY;
//        } else {
        return FieldConstants.defaultAprilTagType;
//        }
    }
}