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

public class DestinationSupplier implements Updatable {
    private static DestinationSupplier instance;
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
    @Getter
    private boolean useVision = true;
    private boolean coralRight = false;
    private boolean useCoral = false;
    private elevatorSetpoint currentElevSetpointCoral = elevatorSetpoint.L2;
    private elevatorSetpoint currentElevSetpointPoke = elevatorSetpoint.P1;

    private DestinationSupplier() {
        swerve = Swerve.getInstance();
    }

    public static DestinationSupplier getInstance() {
        if (instance == null) {
            instance = new DestinationSupplier();
        }
        return instance;
    }

    public static Pose2d getDriveTarget(Pose2d robot, Pose2d goal, boolean rightReef) {
        goal = getFinalDriveTarget(goal, rightReef);
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

    public static Pose2d getFinalDriveTarget(Pose2d goal, boolean rightReef) {
        goal = goal.transformBy(new Transform2d(
                new Translation2d(
                        RobotConstants.ReefAimConstants.ROBOT_TO_PIPE_METERS.get(),
                        RobotConstants.ReefAimConstants.PIPE_TO_TAG.magnitude() * (rightReef ? 1 : -1)),
                new Rotation2d()));
        return goal;
    }

    public static boolean isSafeToRaise(Pose2d robotPose, boolean rightReef) {
        Pose2d tag = getNearestTag(robotPose);
        Pose2d goal = getFinalDriveTarget(tag, rightReef);
        return goal.getTranslation().getDistance(robotPose.getTranslation()) < RobotConstants.ReefAimConstants.RAISE_LIMIT_METERS.get();
    }

    public static Pose2d getNearestTag(Pose2d robotPose) {
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
            double distance =FieldConstants.officialAprilTagType.getLayout().getTagPose(i).get().
                    toPose2d().getTranslation().getDistance(robotPose.getTranslation());
            if (distance < secondMinDistance){
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
        Logger.recordOutput("EdgeCase/DeltaDistance",secondMinDistance - minDistance);
        Logger.recordOutput("EdgeCase/ControllerX", ControllerX);
        Logger.recordOutput("EdgeCase/ControllerY", ControllerY);
        if ((secondMinDistance - minDistance) < RobotConstants.ReefAimConstants.Edge_Case_Max_Delta.get() && ControllerX!=0 && ControllerY!=0){
            Logger.recordOutput("EdgeCase/IsEdgeCase",true);
            if(AllianceFlipUtil.shouldFlip()){
                if (correctTagPair(secondMinDistanceID, minDistanceID, 6,11)){
                    minDistanceID = ControllerY>0?6:11;
                }
                else if (correctTagPair(secondMinDistanceID, minDistanceID, 8,9)){
                    minDistanceID = ControllerY>0?8:9;
                }
                else if (correctTagPair(secondMinDistanceID, minDistanceID, 6,7)){
                    minDistanceID = ControllerX>0?7:6;
                }
                else if (correctTagPair(secondMinDistanceID, minDistanceID, 7,8)){
                    minDistanceID = ControllerX>0?8:7;
                }
                else if (correctTagPair(secondMinDistanceID, minDistanceID, 9,10)){
                    minDistanceID = ControllerX>0?9:10;
                }
                else if (correctTagPair(secondMinDistanceID, minDistanceID, 10,11)){
                    minDistanceID = ControllerX>0?10:11;
                }
            }
            else {
                if (correctTagPair(secondMinDistanceID, minDistanceID, 20,19)){
                    minDistanceID = ControllerY>0?19:20;
                }
                else if (correctTagPair(secondMinDistanceID, minDistanceID, 17,22)){
                    minDistanceID = ControllerY>0?17:22;
                }
                else if (correctTagPair(secondMinDistanceID, minDistanceID, 17,18)){
                    minDistanceID = ControllerX>0?17:18;
                }
                else if (correctTagPair(secondMinDistanceID, minDistanceID, 18,19)){
                    minDistanceID = ControllerX>0?18:19;
                }
                else if (correctTagPair(secondMinDistanceID, minDistanceID, 21,22)){
                    minDistanceID = ControllerX>0?22:21;
                }
                else if (correctTagPair(secondMinDistanceID, minDistanceID, 20,21)){
                    minDistanceID = ControllerX>0?21:20;
                }
            }
        }
        else{
            Logger.recordOutput("EdgeCase/IsEdgeCase",false);
        }
        Logger.recordOutput("EdgeCase/ChangedTarget", minDistanceID == secondMinDistanceID);
        Pose2d goal = FieldConstants.officialAprilTagType.getLayout().getTagPose(minDistanceID).get().toPose2d();
        return goal;
    }

    private static boolean correctTagPair(double tag1, double tag2, double wantedTag1, double wantedTag2) {
        if(tag1 == wantedTag1 && tag2 == wantedTag2){
            return true;
        }
        else if(tag1 == wantedTag2 && tag2 == wantedTag1){
            return true;
        }
        return false;
    }

    public void updateElevatorSetpoint(elevatorSetpoint setpoint) {
        switch (setpoint) {
            case L1, L2, L3, L4:
                currentElevSetpointCoral = setpoint;
                //Logger.recordOutput("DestinationSupplier/currentElevSetpointCoral", setpoint);
                SmartDashboard.putString("DestinationSupplier/currentElevSetpointCoral", setpoint.toString());
                break;
            case P1, P2:
                currentElevSetpointPoke = setpoint;
                //Logger.recordOutput("DestinationSupplier/currentElevSetpointPoke", setpoint);
                SmartDashboard.putString("DestinationSupplier/currentElevSetpointPoke", setpoint.toString());
                break;
            default:
                System.out.println("Unknown elevator setpoint: " + setpoint);
        }
    }

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
            return switch (currentElevSetpointPoke) {
                case P1 -> RobotConstants.ElevatorConstants.P1_EXTENSION_METERS.get();
                case P2 -> RobotConstants.ElevatorConstants.P2_EXTENSION_METERS.get();
                default -> RobotConstants.ElevatorConstants.P1_EXTENSION_METERS.get();
            };
        }
    }

    /**
     * @param coralRight: It always means the right reef RELATIVE TO TAG
     *                    (i.e when you are facing the tag, rightReef = true means the tag on your right is the target)
     */

    public void updateBranch(boolean coralRight) {
        this.coralRight = coralRight;
        //Logger.recordOutput("DestinationSupplier/Pipe", coralRight ? "Right" : "Left");
        SmartDashboard.putString("DestinationSupplier/Pipe", coralRight ? "Right" : "Left");
    }

    public boolean getCurrentBranch() {
        return coralRight;
    }

    public void setCurrentControlMode(controlMode mode) {
        this.currentControlMode = mode;
        //Logger.recordOutput("DestinationSupplier/CurrentControlMode", mode);
        SmartDashboard.putString("DestinationSupplier/CurrentControlMode", mode.name());
    }

    public void setCurrentL1Mode(L1Mode mode) {
        this.l1Mode = mode;
        SmartDashboard.putString("DestinationSupplier/CurrentL1Mode", mode.name());
    }

    public void setCurrentIntakeMode(IntakeMode mode) {
        this.intakeMode = mode;
        SmartDashboard.putString("DestinationSupplier/CurrentIntakeMode", mode.name());
    }

    public void setUseVision(boolean useVision) {
        this.useVision = useVision;
        SmartDashboard.putBoolean("DestinationSupplier/UseVision", useVision);
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
}