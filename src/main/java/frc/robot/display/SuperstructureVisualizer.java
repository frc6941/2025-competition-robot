package frc.robot.display;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static frc.robot.RobotContainer.intakeHasCoral;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import frc.robot.drivers.GamepieceTracker;
import frc.robot.subsystems.swerve.Swerve;

public class SuperstructureVisualizer {
        //todo: change the mechenism location to fit the real robot
    private static SuperstructureVisualizer instance;

    // Conversion helper
    private static double mmToM(double mm) {
        return mm / 1000.0;
    }

    // Elevator constants
    private static final Translation3d ELEVATOR_START = new Translation3d(
            mmToM(-200), mmToM(0), mmToM(300));
    private static final Translation3d ELEVATOR_END = new Translation3d(
            mmToM(-200), mmToM(0), mmToM(300));
    private static final Translation3d ELEVATOR_CENTER = ELEVATOR_START.interpolate(ELEVATOR_END, 0.5);

    // Stage specifications
    private static final double STAGE1_RETRACT_LENGTH = mmToM(958 + 8 + 8);
    private static final double STAGE1_TRAVEL = mmToM(790);
    private static final double STAGE2_RETRACTED_OFFSET = mmToM(18);
    private static final double STAGE2_TRAVEL = mmToM(820);
    private static final double STAGE2_LENGTH = mmToM(352);
    private static final double STAGE3_LENGTH = mmToM(314.29);

    // Intake constants
    private static final Translation3d INTAKE_PIVOT_START = new Translation3d(
            mmToM(280), mmToM(-181), mmToM(250));
    private static final Translation3d INTAKE_PIVOT_END = new Translation3d(
            mmToM(280), mmToM(181), mmToM(250));
    private static final Translation3d INTAKE_CENTER = INTAKE_PIVOT_START.interpolate(INTAKE_PIVOT_END, 0.5);
    private static final double INTAKE_LENGTH = mmToM(452);

    // EndEffectorArm constants

    private static final Translation3d END_EFFECTOR_CENTER = ELEVATOR_CENTER;
    private static final double END_EFFECTOR_LENGTH_CORAL = mmToM(170);
    private static final double END_EFFECTOR_LENGTH_ALGAE = mmToM(325);
    private static final double END_EFFECTOR_MOUNT_ARM_LENGTH = mmToM(242);

    // Visualization components
    private final LoggedMechanism2d elevatorMechanism;
    private final LoggedMechanismLigament2d elevatorHeight;
    private final LoggedMechanismLigament2d endEffectorMountArm;
    private final LoggedMechanismLigament2d elevatorStage3;
    private final LoggedMechanismLigament2d endEffectorArmCoral;
    private final LoggedMechanismLigament2d endEffectorArmAlgae;

    private final LoggedMechanism2d intakeMechanism;
    private final LoggedMechanismLigament2d intakeArm;

    // Current state tracking
    private double currentElevatorHeight = 0.0;
    private double currentIntakeAngleDeg = 0.0;
    private double currentEndEffectorAngleDeg = 0;

    // Coral diameter in meters
    private static final double CORAL_DIAMETER = mmToM(100); // Adjust this value based on actual coral size


    public static SuperstructureVisualizer getInstance() {
        if (instance == null) {
            instance = new SuperstructureVisualizer();
        }
        return instance;
    }

    public SuperstructureVisualizer() {
        // Elevator mechanism setup
        elevatorMechanism = new LoggedMechanism2d(
                0,
                0,
                new Color8Bit(Color.kWhite));

        LoggedMechanismRoot2d elevatorRoot = elevatorMechanism.getRoot(
                "ElevatorBase",
                ELEVATOR_CENTER.getX(),
                ELEVATOR_CENTER.getZ());

        elevatorHeight = new LoggedMechanismLigament2d(
                "stage1",
                STAGE1_RETRACT_LENGTH,
                90,
                10,
                new Color8Bit(Color.kBlue));

        elevatorStage3 = new LoggedMechanismLigament2d(
                "elevatorStage3",
                STAGE3_LENGTH,
                0,
                8,
                new Color8Bit(Color.kBlue));

        endEffectorMountArm = new LoggedMechanismLigament2d(
                "endEffectorMountArm",
                END_EFFECTOR_MOUNT_ARM_LENGTH,
                0,
                8,
                new Color8Bit(Color.kPurple));

            endEffectorArmCoral = new LoggedMechanismLigament2d(
                "endEffectorArmCoral",
                END_EFFECTOR_LENGTH_CORAL,
                90,
                8,
                new Color8Bit(Color.kGreen));

        endEffectorArmAlgae = new LoggedMechanismLigament2d(
                "endEffectorArmAlgae",
                END_EFFECTOR_LENGTH_ALGAE,
                270,
                8,
                new Color8Bit(Color.kYellow));

        elevatorRoot.append(elevatorHeight);
        elevatorHeight.append(elevatorStage3);
        elevatorStage3.append(endEffectorMountArm);
        endEffectorMountArm.append(endEffectorArmCoral);
        endEffectorMountArm.append(endEffectorArmAlgae);

        // Intake mechanism setup
        intakeMechanism = new LoggedMechanism2d(
                0,
                0,
                new Color8Bit(Color.kWhite));

        LoggedMechanismRoot2d intakeRoot = intakeMechanism.getRoot(
                "IntakePivot",
                0.25,
                INTAKE_CENTER.getZ());

        intakeArm = new LoggedMechanismLigament2d(
                "intakeArm",
                INTAKE_LENGTH,
                90,
                8,
                new Color8Bit(Color.kRed));

        intakeRoot.append(intakeArm);
    }

    // Full update
    public void update(double elevatorHeight, double intakeAngleRad, double endEffectorAngleRad) {
        this.currentElevatorHeight = elevatorHeight;
        this.currentIntakeAngleDeg = intakeAngleRad;
        this.currentEndEffectorAngleDeg = endEffectorAngleRad;
        updateVisuals();
    }

    // Elevator-only update
    public void updateElevator(double elevatorHeight) {
        this.currentElevatorHeight = elevatorHeight;
        updateVisuals();
    }

    // Intake-only update
    public void updateIntake(double intakeAngleDeg) {
        this.currentIntakeAngleDeg = intakeAngleDeg;
        updateVisuals();
    }

    // EndEffectorArm-only update
    public void updateEndEffector(double endEffectorAngleDeg) {
        this.currentEndEffectorAngleDeg = endEffectorAngleDeg;
        updateVisuals();
    }

    /**
     * Logs the Coral pose 3D if coral is detected in the intake
     */
    private void logCoralPose3D() {
        // Get the GamepieceTracker instance

        
        // Check if coral is detected in the intake
        if (GamepieceTracker.getInstance().isIntakeHasCoral()) {
        Pose3d robotPose = new Pose3d(Swerve.getInstance().getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp()));
            // Calculate the position of the coral at the middle of the intake arm
            // The coral is positioned at the end of the intake arm
            double intakeAngleRad = Math.toRadians(currentIntakeAngleDeg - 90);
            
            // Create a rotation matrix for the intake angle
            Rotation3d intakeRotation = new Rotation3d(0, intakeAngleRad, 0);
            
            // Calculate the position of the coral at the end of the intake arm
            Pose3d coralPosition = robotPose.transformBy(new Transform3d(INTAKE_CENTER.plus(new Translation3d(INTAKE_LENGTH/2,0,0).rotateBy(intakeRotation)), intakeRotation));
            
            // Log the Coral pose 3D
            Logger.recordOutput("Superstructure/Coral/InakeCoral", 
                    coralPosition);
        } else {
            // If no coral is detected, log an empty pose
            Logger.recordOutput("Superstructure/Coral/InakeCoral", new Pose3d());
        }
        if (GamepieceTracker.getInstance().isEndeffectorHasCoral()) {
            Pose3d robotPose = new Pose3d(Swerve.getInstance().getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp()));
            
            // Calculate the position of the coral at the middle of the end effector arm coral
            double endEffectorAngleRad = Math.toRadians(-currentEndEffectorAngleDeg-180);
            
            // Create a rotation matrix for the end effector angle
            Rotation3d endEffectorRotation = new Rotation3d(0, endEffectorAngleRad, 0);

            Translation3d endEffectorPosition = END_EFFECTOR_CENTER.plus(new Translation3d(0, 0, currentElevatorHeight+STAGE3_LENGTH));
            
            // Calculate the position of the coral at the middle of the end effector arm coral
            Pose3d coralPosition = robotPose.transformBy(new Transform3d(
                endEffectorPosition.plus(new Translation3d(-END_EFFECTOR_LENGTH_CORAL, 0, END_EFFECTOR_MOUNT_ARM_LENGTH).rotateBy(endEffectorRotation)),
                endEffectorRotation));
            
            Logger.recordOutput("Superstructure/Coral/EECoral", coralPosition);
        } else {
            Logger.recordOutput("Superstructure/Coral/EECoral", new Pose3d());
        }
        if (GamepieceTracker.getInstance().isEndeffectorHasAlgae()) {
                Pose3d robotPose = new Pose3d(Swerve.getInstance().getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp()));
                
                // Calculate the position of the coral at the middle of the end effector arm coral
                double endEffectorAngleRad = Math.toRadians(-currentEndEffectorAngleDeg-180);
                
                // Create a rotation matrix for the end effector angle
                Rotation3d endEffectorRotation = new Rotation3d(0, endEffectorAngleRad, 0);
    
                Translation3d endEffectorPosition = END_EFFECTOR_CENTER.plus(new Translation3d(0, 0, currentElevatorHeight+STAGE3_LENGTH));
                
                // Calculate the position of the coral at the middle of the end effector arm coral
                Pose3d coralPosition = robotPose.transformBy(new Transform3d(
                    endEffectorPosition.plus(new Translation3d(END_EFFECTOR_LENGTH_ALGAE, 0, END_EFFECTOR_MOUNT_ARM_LENGTH).rotateBy(endEffectorRotation)),
                    endEffectorRotation));
                
                Logger.recordOutput("Superstructure/Coral/EEAlgae", coralPosition);
            } else {
                Logger.recordOutput("Superstructure/Coral/EEAlgae", new Pose3d());
            }
    }

    private void updateVisuals() {
        // Update elevator components
        elevatorHeight.setLength(currentElevatorHeight); // Stage 1 extends

        // Update intake components
        intakeArm.setAngle(Rotation2d.fromRadians(Math.toRadians(-currentIntakeAngleDeg+90)));

        // Update end effector components
        endEffectorMountArm.setAngle(Rotation2d.fromRadians(Math.toRadians(currentEndEffectorAngleDeg+180)));

        // Log Coral pose 3D
        logCoralPose3D();

        // Log 2D mechanisms
        Logger.recordOutput("Superstructure/Elevator/Mechanism2d", elevatorMechanism);
        Logger.recordOutput("Superstructure/Intake/Mechanism2d", intakeMechanism);
    }
}