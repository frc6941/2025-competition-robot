// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.google.flatbuffers.Constants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.MassUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.auto.basics.AutoActions;
import frc.robot.commands.*;
import frc.robot.display.Display;
//import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIONorthstar;
import frc.robot.subsystems.beambreak.BeambreakIOReal;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorIOReal;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem.WantedState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.AllianceFlipUtil;
import lombok.Getter;

import org.frcteam6941.looper.UpdateManager;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.RobotConstants.BeamBreakConstants.*;
import static frc.robot.RobotConstants.ElevatorConstants.*;

import java.io.IOException;
import java.util.function.*;

import javax.print.attribute.standard.Destination;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private enum superState{
        STOPPED,
        L1,
        L2,
        L3,
        L4,
        GROUND_INTAKE,
        FUNNEL_INTAKE,
        SHOOT_CORAL
    }
    CommandXboxController driverController = new CommandXboxController(0);
    CommandXboxController operatorController = new CommandXboxController(1);
    CommandXboxController testerController = new CommandXboxController(2);

    @Getter
    private final UpdateManager updateManager;
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
    @Getter
    private LoggedDashboardChooser<Command> autoChooser;
    private Command TRY_STOPPED(){
        return Commands.parallel(
                        elevatorSubsystem.setElevatorStateCommand(ElevatorSubsystem.WantedState.ZERO),
                        endEffectorSubsystem.setWantedStateCommand(WantedState.IDLE)
                )
        ;
    }
    private Command TRY_L1(){
        return Commands.parallel(
                        elevatorSubsystem.setElevatorPositionCommand(L1_EXTENSION_METERS.get()),
                        endEffectorSubsystem.setWantedStateCommand(WantedState.IDLE)
                )
        ;
    }
    private Command TRY_L2(){
        return Commands.parallel(
                        elevatorSubsystem.setElevatorPositionCommand(L2_EXTENSION_METERS.get()),
                        endEffectorSubsystem.setWantedStateCommand(WantedState.IDLE)
                )
        ;
    }
    private Command TRY_L3(){
        return Commands.parallel(
                        elevatorSubsystem.setElevatorPositionCommand(L3_EXTENSION_METERS.get()),
                        endEffectorSubsystem.setWantedStateCommand(WantedState.IDLE)
                )
        ;
    }private Command TRY_L4(){
        return Commands.parallel(
                        elevatorSubsystem.setElevatorPositionCommand(L4_EXTENSION_METERS.get()),
                        endEffectorSubsystem.setWantedStateCommand(WantedState.IDLE)
                )
        ;
    }
    private Command TRY_GROUND_INTAKE() {
        if (endEffectorSubsystem.isIntakeFinished()) {
                return Commands.parallel(
                        elevatorSubsystem.setElevatorPositionCommand(L1_EXTENSION_METERS.get())
                        .andThen(new WaitUntilCommand(() -> elevatorSubsystem.getIo().isNearExtension(L1_EXTENSION_METERS.get())))
                        .andThen(new WaitCommand(2))
                        .andThen(elevatorSubsystem.setElevatorPositionCommand(INTAKER_INTAKE_METERS.get()))
                        .andThen(endEffectorSubsystem.setWantedStateCommand(WantedState.GROUND_INTAKE)),
                        endEffectorSubsystem.setWantedStateCommand(EndEffectorSubsystem.WantedState.TRANSFER));
        }else {
                return Commands.parallel(
                        elevatorSubsystem.setElevatorPositionCommand(L1_EXTENSION_METERS.get()).withTimeout(2)
                        .andThen(new WaitUntilCommand(() -> elevatorSubsystem.getIo().isNearExtension(L1_EXTENSION_METERS.get())))
                        .andThen(new WaitCommand(2))
                        .andThen(elevatorSubsystem.setElevatorPositionCommand(INTAKER_INTAKE_METERS.get())),
                        // .andThen(endEffectorSubsystem.setWantedStateCommand(WantedState.GROUND_INTAKE)),
                        endEffectorSubsystem.setWantedStateCommand(EndEffectorSubsystem.WantedState.GROUND_INTAKE));
        }
        }
    private Command TRY_FUNNEL_INTAKE(){
        if (endEffectorSubsystem.isIntakeFinished()) {
                return Commands.parallel(
                        elevatorSubsystem.setElevatorPositionCommand(L1_EXTENSION_METERS.get()).withTimeout(2)
                        .andThen(new WaitUntilCommand(() -> elevatorSubsystem.getIo().isNearExtension(L1_EXTENSION_METERS.get())))
                        .andThen(elevatorSubsystem.setElevatorPositionCommand(INTAKER_INTAKE_METERS.get())),
                        // .andThen(endEffectorSubsystem.setWantedStateCommand(WantedState.GROUND_INTAKE)),
                        endEffectorSubsystem.setWantedStateCommand(EndEffectorSubsystem.WantedState.TRANSFER));
        }else {
                return Commands.parallel(
                        elevatorSubsystem.setElevatorPositionCommand(FUNNEL_INTAKE_METERS.get()),                        // .andThen(endEffectorSubsystem.setWantedStateCommand(WantedState.GROUND_INTAKE)),
                        endEffectorSubsystem.setWantedStateCommand(EndEffectorSubsystem.WantedState.FUNNEL_INTAKE));
        }
    }
    private Command TRY_SHOOT_CORAL(){
        return new WaitUntilCommand(()->endEffectorSubsystem.isCoralReady()).andThen(
                        endEffectorSubsystem.setWantedStateCommand(WantedState.SHOOT)
        );
    }
    private Command setSuperState(superState state){
        return switch (state) {
                case STOPPED -> TRY_STOPPED();
                case L1 -> TRY_L1();
                case L2 -> TRY_L2();
                case L3 -> TRY_L3();
                case L4 -> TRY_L4();
                case GROUND_INTAKE -> TRY_GROUND_INTAKE();
                case FUNNEL_INTAKE -> TRY_FUNNEL_INTAKE();
                default -> TRY_STOPPED(); // or some other default value that makes sense in your context
        };
        }
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        updateManager = new UpdateManager(swerve,
                display);
        updateManager.registerAll();
        configureAuto();
        configureDriverBindings(driverController);
        configureOperatorBindings(operatorController);
        configureTesterBindings(testerController);
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
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private void configureSubsystems(){
        
    }
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
        new Trigger(controller.a())
                .onTrue(setSuperState(superState.L1));
        new Trigger(controller.b())
                .onTrue(setSuperState(superState.L2));
        new Trigger(controller.x())
                .onTrue(setSuperState(superState.L3));
        new Trigger(controller.y())
                .onTrue(setSuperState(superState.L4));
        
        new Trigger(controller.leftTrigger())
                .onTrue(setSuperState(superState.STOPPED));

        //test of endeffector state machine
        //Funnel Intake
        // new Trigger(controller.leftBumper()
        //         .onTrue(Commands.parallel(
        //                 // endEffectorSubsystem.setWantedSuperStateCommand(EndEffectorSubsystem.WantedState.FUNNEL_INTAKE),
        //                 elevatorSubsystem.setElevatorWantedPositionCommand("Funnel Intake"),
                        
        //                 elevatorSubsystem.setElevatorStateCommand(ElevatorSubsystem.WantedState.POSITION)
        //         ));

        // //Intaker Intake
        

        
        // new Trigger(controller.rightBumper())
        //         .onTrue(Commands.parallel(
        //                 // endEffectorSubsystem.setWantedSuperStateCommand(EndEffectorSubsystem.WantedState.GROUND_INTAKE),
        //                 elevatorSubsystem.setElevatorWantedPositionCommand("Intaker Intake"),
        //                 elevatorSubsystem.setElevatorStateCommand(ElevatorSubsystem.WantedState.POSITION)
        //         ));

        // //Shoot
        // new Trigger(controller.rightTrigger())
        //         .onTrue(Commands.parallel(
        //                 // endEffectorSubsystem.setWantedSuperStateCommand(EndEffectorSubsystem.WantedState.SHOOT),
        //                 elevatorSubsystem.setElevatorWantedPositionCommand("Shoot"),
        //                 elevatorSubsystem.setElevatorStateCommand(ElevatorSubsystem.WantedState.POSITION)
        //         ));

        // //Change Shoot Position
        // new Trigger(controller.a()).onTrue(elevatorSubsystem.setElevatorShootPositionCommand("L1", L1_EXTENSION_METERS.get()));
        // new Trigger(controller.b()).onTrue(elevatorSubsystem.setElevatorShootPositionCommand("L2", L2_EXTENSION_METERS.get()));
        // new Trigger(controller.x()).onTrue(elevatorSubsystem.setElevatorShootPositionCommand("L3", L3_EXTENSION_METERS.get()));
        // new Trigger(controller.y()).onTrue(elevatorSubsystem.setElevatorShootPositionCommand("L4", L4_EXTENSION_METERS.get()));

        // //Elevator Home
    }

    private void configureAuto() {
        NamedCommands.registerCommand("AutoShoot", TRY_FUNNEL_INTAKE().withTimeout(3.0));
        NamedCommands.registerCommand("AutoGroundIntake", TRY_GROUND_INTAKE().withTimeout(3.0));
        NamedCommands.registerCommand("AutoFunnelIntake", TRY_FUNNEL_INTAKE().withTimeout(3.0));
        NamedCommands.registerCommand("AutoL1", TRY_L1().withTimeout(3.0));
        NamedCommands.registerCommand("AutoL2", TRY_L2().withTimeout(3.0));
        NamedCommands.registerCommand("AutoL3", TRY_L3().withTimeout(3.0));
        NamedCommands.registerCommand("AutoL4", TRY_L4().withTimeout(3.0));
        NamedCommands.registerCommand("AutoStop", TRY_STOPPED().withTimeout(3.0));

        //TODO:change values to match actual robot dimensions
        AutoBuilder.configure(
                () -> Swerve.getInstance().getLocalizer().getCoarseFieldPose(0),
                (Pose2d pose2d) -> Swerve.getInstance().resetPose(pose2d),
                () -> Swerve.getInstance().getChassisSpeeds(),
                (ChassisSpeeds chassisSpeeds) -> Swerve.getInstance().driveSpeed(chassisSpeeds),
                new PathFollowingController() {

                        @Override
                        public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose,
                                        PathPlannerTrajectoryState targetState) {
                                // TODO Auto-generated method stub
                                throw new UnsupportedOperationException("Unimplemented method 'calculateRobotRelativeSpeeds'");
                        }

                        @Override
                        public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
                                // TODO Auto-generated method stub
                                throw new UnsupportedOperationException("Unimplemented method 'reset'");
                        }

                        @Override
                        public boolean isHolonomic() {
                                // TODO Auto-generated method stub
                                throw new UnsupportedOperationException("Unimplemented method 'isHolonomic'");
                        }
                        
                },
                new RobotConfig(60, 1/20, new ModuleConfig(
                        0.55, 
                        RobotConstants.SwerveConstants.maxSpeed.magnitude(), 
                        0.5, 
                        new DCMotor(
                                5, 
                                10, 
                                40, 
                                80, 
                                90,
                                8), 
                        80,
                        8),
                          new Translation2d(0, 0)),
                AllianceFlipUtil::shouldFlip,
                swerve
        );

        autoChooser = new LoggedDashboardChooser<>("Chooser", AutoBuilder.buildAutoChooser());

        // dashboard.registerAutoSelector(autoChooser.getSendableChooser());
    }

    private Command rumbleDriver(double seconds) {
        return new RumbleCommand(Seconds.of(seconds), driverController.getHID());
    }

    /**
     * Returns the current AprilTag layout type.
     */
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