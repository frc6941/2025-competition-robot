package frc.robot.drivers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.RobotConstants;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;
import lombok.Setter;
import org.frcteam6941.looper.Updatable;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class DestinationSupplier implements Updatable {
    private static DestinationSupplier instance;
    @Getter
    @Setter
    public boolean useSuperCycle = true;
    Swerve swerve;
    @Getter
    private L1Mode l1Mode = L1Mode.ELEVATOR;
    @Getter
    private IntakeMode intakeMode = IntakeMode.NORMAL;
    @Getter
    private controlMode currentControlMode = controlMode.AUTO;
    @Getter
    @Setter
    private int targetTagID = 0;
    private boolean coralRight = false;
    private boolean useCoral = false;
    @Getter
    private elevatorSetpoint currentElevSetpointCoral = elevatorSetpoint.L2;
    @Getter
    private elevatorSetpoint currentElevSetpointAlgae = elevatorSetpoint.P1;
    @Getter
    private AlgaeScoringMode algaeScoringMode = AlgaeScoringMode.NET;
    @Getter
    private GamePiece currentGamePiece = GamePiece.CORAL_SCORING;

    private DestinationSupplier() {
        swerve = Swerve.getInstance();
    }

    public static DestinationSupplier getInstance() {
        if (instance == null) {
            instance = new DestinationSupplier();
        }
        return instance;
    }

    /**
     * Calculates the optimal drive target position based on the robot's current position and goal position
     *
     * @param robot The current pose (position and rotation) of the robot
     * @param goal  The target pose to drive towards
     * @return A modified goal pose that accounts for optimal approach positioning
     */
    public static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
        Transform2d offset = new Transform2d(goal, new Pose2d(robot.getTranslation(), goal.getRotation()));
        double yDistance = Math.abs(offset.getY());
        double xDistance = Math.abs(offset.getX());
        double shiftXT =
                MathUtil.clamp(
                        (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
                        0.0,
                        1.0);
        double shiftYT = MathUtil.clamp(yDistance <= 0.2 ? 0.0 : -offset.getX() / Reef.faceLength, 0.0, 1.0);
        goal = goal.transformBy(
                new Transform2d(
                        shiftXT * RobotConstants.ReefAimConstants.MAX_DISTANCE_REEF_LINEUP.get(),
                        Math.copySign(shiftYT * RobotConstants.ReefAimConstants.MAX_DISTANCE_REEF_LINEUP.get() * 0.8, offset.getY()),
                        new Rotation2d()));

        return goal;
    }

    /**
     * Calculates the final target position for coral scoring based on the tag pose
     *
     * @param goal      The initial goal pose
     * @param rightReef Whether to target the right reef relative to the AprilTag
     * @return Modified goal pose to tag pose accounting for coral scoring position
     */
    public static Pose2d getFinalCoralTarget(Pose2d goal, boolean rightReef) {
        goal = goal.transformBy(new Transform2d(
                new Translation2d(
                        RobotConstants.ReefAimConstants.ROBOT_TO_PIPE_METERS.get(),
                        RobotConstants.ReefAimConstants.PIPE_TO_TAG.magnitude() * (rightReef ? 1 : -1)),
                new Rotation2d()));
        return goal;
    }

    /**
     * Calculates the final target position for algae scoring based on the tag pose
     *
     * @param goal The initial goal pose
     * @return Modified goal pose to tag pose accounting for algae scoring position
     */
    public static Pose2d getFinalAlgaeTarget(Pose2d goal) {
        goal = goal.transformBy(new Transform2d(
                new Translation2d(
                        RobotConstants.ReefAimConstants.ROBOT_TO_ALGAE_METERS.get(),
                        RobotConstants.ReefAimConstants.ALGAE_TO_TAG_METERS.get()),
                new Rotation2d()));
        return goal;
    }

    /**
     * Determines if it's safe to raise the elevator based on robot position
     *
     * @param robotPose Current pose of the robot
     * @param rightReef Whether targeting the right reef relative to the AprilTag
     * @return true if the robot is within safe distance to raise elevator, false otherwise
     */
    public static boolean isSafeToRaise(Pose2d robotPose, boolean rightReef) {
        Pose2d tag = getNearestTag(robotPose);
        Pose2d goal = getFinalCoralTarget(tag, rightReef);
        return goal.getTranslation().getDistance(robotPose.getTranslation()) < RobotConstants.ReefAimConstants.RAISE_LIMIT_METERS.get();
    }

    public static void isEdgeCase(Pose2d robotPose) {
        XboxController driverController = new XboxController(0);
        double ControllerX = driverController.getLeftX();
        double ControllerY = driverController.getLeftY();
        double minDistance = Double.MAX_VALUE;
        double secondMinDistance = Double.MAX_VALUE;
        int ReefTagMin = AllianceFlipUtil.shouldFlip() ? 6 : 17;
        int ReefTagMax = AllianceFlipUtil.shouldFlip() ? 11 : 22;
        int minDistanceID = ReefTagMin;
        int secondMinDistanceID = ReefTagMin;
        for (int i = ReefTagMin; i <= ReefTagMax; i++) {
            double distance = FieldConstants.officialAprilTagType.getLayout().getTagPose(i).get().
                    toPose2d().getTranslation().getDistance(robotPose.getTranslation());
            if (distance < secondMinDistance) {
                secondMinDistanceID = i;
                secondMinDistance = distance;
            }
            if (distance < minDistance) {
                secondMinDistanceID = minDistanceID;
                secondMinDistance = minDistance;
                minDistanceID = i;
                minDistance = distance;
            }
        }
        Logger.recordOutput("EdgeCase/DeltaDistance", secondMinDistance - minDistance);
        Logger.recordOutput("EdgeCase/ControllerX", ControllerX);
        Logger.recordOutput("EdgeCase/ControllerY", ControllerY);
        if ((secondMinDistance - minDistance) < RobotConstants.ReefAimConstants.Edge_Case_Max_Delta.get()) {
            Logger.recordOutput("EdgeCase/IsEdgeCase", true);
            if (Math.abs(ControllerX) >= 0.05 || Math.abs(ControllerY) >= 0.05) {
                minDistanceID = solveEdgeCase(ControllerX, ControllerY, minDistanceID, secondMinDistanceID);
            }
        } else {
            Logger.recordOutput("EdgeCase/IsEdgeCase", false);
        }
        Logger.recordOutput("EdgeCase/TargetChanged", minDistanceID == secondMinDistanceID);
    }

    /**
     * Gets the nearest AprilTag pose to the robot's current position
     *
     * @param robotPose Current pose of the robot
     * @return Pose2d of the nearest AprilTag, accounting for edge cases and controller input
     */
    public static Pose2d getNearestTag(Pose2d robotPose) {
        return FieldConstants.officialAprilTagType.getLayout().getTagPose(getNearestTagID(robotPose)).get().toPose2d();
    }

    /**
     * Gets the ID of the nearest AprilTag to the robot's current position
     *
     * @param robotPose Current pose of the robot
     * @return ID of the nearest AprilTag, accounting for edge cases and controller input
     */
    public static int getNearestTagID(Pose2d robotPose) {
        XboxController driverController = new XboxController(0);
        double ControllerX = driverController.getLeftX();
        double ControllerY = driverController.getLeftY();
        double minDistance = Double.MAX_VALUE;
        double secondMinDistance = Double.MAX_VALUE;
        int ReefTagMin = AllianceFlipUtil.shouldFlip() ? 6 : 17;
        int ReefTagMax = AllianceFlipUtil.shouldFlip() ? 11 : 22;
        int minDistanceID = ReefTagMin;
        int secondMinDistanceID = ReefTagMin;
        for (int i = ReefTagMin; i <= ReefTagMax; i++) {
            double distance = FieldConstants.officialAprilTagType.getLayout().getTagPose(i).get().
                    toPose2d().getTranslation().getDistance(robotPose.getTranslation());
            if (distance < secondMinDistance) {
                secondMinDistanceID = i;
                secondMinDistance = distance;
            }
            if (distance < minDistance) {
                secondMinDistanceID = minDistanceID;
                secondMinDistance = minDistance;
                minDistanceID = i;
                minDistance = distance;
            }
        }
        if ((secondMinDistance - minDistance) < RobotConstants.ReefAimConstants.Edge_Case_Max_Delta.get() && (Math.abs(ControllerX) >= 0.05 || Math.abs(ControllerY) >= 0.05)) {
            minDistanceID = solveEdgeCase(ControllerX, ControllerY, minDistanceID, secondMinDistanceID);
        }
        return minDistanceID;
    }

    private static int solveEdgeCase(double controllerX, double controllerY, int minDistanceID, int secondMinDistanceID) {
        record TagCondition(int tagA, int tagB, char axis, int positiveResult, int negativeResult) {
        }
        List<TagCondition> conditions = AllianceFlipUtil.shouldFlip() ?
                List.of(
                        new TagCondition(6, 11, 'Y', 6, 11),
                        new TagCondition(8, 9, 'Y', 8, 9),
                        new TagCondition(6, 7, 'X', 7, 6),
                        new TagCondition(7, 8, 'X', 8, 7),
                        new TagCondition(9, 10, 'X', 9, 10),
                        new TagCondition(10, 11, 'X', 10, 11)
                ) :
                List.of(
                        new TagCondition(20, 19, 'Y', 19, 20),
                        new TagCondition(17, 22, 'Y', 17, 22),
                        new TagCondition(17, 18, 'X', 17, 18),
                        new TagCondition(18, 19, 'X', 18, 19),
                        new TagCondition(21, 22, 'X', 22, 21),
                        new TagCondition(20, 21, 'X', 21, 20)
                );
        for (TagCondition condition : conditions) {
            if (correctTagPair(secondMinDistanceID, minDistanceID, condition.tagA(), condition.tagB())) {
                double value = condition.axis() == 'X' ? controllerX : controllerY;
                minDistanceID = value > 0 ? condition.positiveResult() : condition.negativeResult();
                break;
            }
        }
        return minDistanceID;
    }

    private static boolean correctTagPair(double tag1, double tag2, double wantedTag1, double wantedTag2) {
        return (tag1 == wantedTag1 && tag2 == wantedTag2) || (tag1 == wantedTag2 && tag2 == wantedTag1);
    }

    /**
     * Updates the elevator setpoint for either coral or poke positions
     *
     * @param setpoint The desired elevator setpoint (L1-L4 for coral, P1-P2 for poke)
     */
    public void updateElevatorSetpoint(elevatorSetpoint setpoint) {
        switch (setpoint) {
            case L1, L2, L3, L4:
                currentElevSetpointCoral = setpoint;
                //Logger.recordOutput("DestinationSupplier/currentElevSetpointCoral", setpoint);
                SmartDashboard.putString("DestinationSupplier/currentElevSetpointCoral", setpoint.toString());
                break;
            case P1, P2:
                currentElevSetpointAlgae = setpoint;
                //Logger.recordOutput("DestinationSupplier/currentElevSetpointPoke", setpoint);
                SmartDashboard.putString("DestinationSupplier/currentElevSetpointPoke", setpoint.toString());
                break;
            default:
                System.out.println("Unknown elevator setpoint: " + setpoint);
        }
    }

    /**
     * Gets the current elevator setpoint value in meters
     *
     * @param useCoral Whether to use coral scoring position (true) or poke position (false)
     * @return The elevator extension distance in meters
     */
    public double getElevatorSetpoint(boolean useCoral) {
        this.useCoral = useCoral;
        if (useCoral) {
            return switch (currentElevSetpointCoral) {
                case L1 -> RobotConstants.ElevatorConstants.L1_EXTENSION_METERS.get();
                case L2 -> RobotConstants.ElevatorConstants.L2_EXTENSION_METERS.get();
                case L3 -> RobotConstants.ElevatorConstants.L3_EXTENSION_METERS.get();
                case L4 -> RobotConstants.ElevatorConstants.L4_EXTENSION_METERS.get();
                default -> RobotConstants.ElevatorConstants.L2_EXTENSION_METERS.get();
            };
        } else {
            return switch (currentElevSetpointAlgae) {
                case P1 -> RobotConstants.ElevatorConstants.P1_EXTENSION_METERS.get();
                case P2 -> RobotConstants.ElevatorConstants.P2_EXTENSION_METERS.get();
                default -> RobotConstants.ElevatorConstants.P1_EXTENSION_METERS.get();
            };
        }
    }

    /**
     * Updates which reef branch to target
     *
     * @param coralRight When true, targets the right reef relative to the AprilTag when facing it
     *                   (i.e. when you are facing the tag, rightReef = true means the tag on your right is the target)
     */
    public void updateBranch(boolean coralRight) {
        this.coralRight = coralRight;
        //Logger.recordOutput("DestinationSupplier/Pipe", coralRight ? "Right" : "Left");
        SmartDashboard.putString("DestinationSupplier/Pipe", coralRight ? "Right" : "Left");
    }

    public void setAlgaeScoringMode(AlgaeScoringMode algaeScoringMode) {
        this.algaeScoringMode = algaeScoringMode;
        SmartDashboard.putString("DestinationSupplier/algaeScoringMode", algaeScoringMode.toString());
    }

    /**
     * Gets the current branch setting
     *
     * @return true if targeting right reef, false if targeting left reef
     */
    public boolean getCurrentBranch() {
        return coralRight;
    }

    /**
     * Sets the current control mode for the robot
     *
     * @param mode The desired control mode (MANUAL, SEMI, or AUTO)
     */
    public void setCurrentControlMode(controlMode mode) {
        this.currentControlMode = mode;
        //Logger.recordOutput("DestinationSupplier/CurrentControlMode", mode);
        SmartDashboard.putString("DestinationSupplier/CurrentControlMode", mode.name());
    }

    /**
     * Sets the current L1 operation mode
     *
     * @param mode The desired L1 mode (ELEVATOR or INTAKE)
     */
    public void setCurrentL1Mode(L1Mode mode) {
        this.l1Mode = mode;
        SmartDashboard.putString("DestinationSupplier/CurrentL1Mode", mode.name());
    }

    /**
     * Sets the current intake operation mode
     *
     * @param mode The desired intake mode (TREMBLE or NORMAL)
     */
    public void setCurrentIntakeMode(IntakeMode mode) {
        this.intakeMode = mode;
        SmartDashboard.putString("DestinationSupplier/CurrentIntakeMode", mode.name());
    }

    public void updatePokeSetpointByTag(int tagNumber) {
        switch (tagNumber) {
            case 6, 8, 10, 17, 19, 21:
                updateElevatorSetpoint(elevatorSetpoint.P1);
                break;
            case 7, 9, 11, 18, 20, 22:
                updateElevatorSetpoint(elevatorSetpoint.P2);
                break;
            default:
                System.out.println("Tag number does not correspond to a valid elevator setpoint.");
        }
    }

    /**
     * Sets the current game piece type
     *
     * @param gamePiece The desired game piece (CORAL_SCORING or ALGAE_INTAKING)
     */
    public void setCurrentGamePiece(GamePiece gamePiece) {
        this.currentGamePiece = gamePiece;
        SmartDashboard.putString("DestinationSupplier/CurrentGamePiece", gamePiece.name());
    }

    /**
     * Switch if using super cycle
     */
    public void switchUseSuperCycle() {
        useSuperCycle = !useSuperCycle;
        SmartDashboard.putBoolean("DestinationSupplier/UseSuperCycle", useSuperCycle);
    }


    public enum elevatorSetpoint {
        L1, L2, L3, L4, P1, P2
    }

    public enum controlMode {
        MANUAL, SEMI, AUTO
    }

    public enum L1Mode {
        ELEVATOR,
        INTAKE
    }

    public enum IntakeMode {
        TREMBLE,
        NORMAL
    }

    public enum GamePiece {
        CORAL_SCORING,
        ALGAE_INTAKING
    }

    public enum AlgaeScoringMode {
        NET,
        PROCESSOR
    }
}
