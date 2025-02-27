package frc.robot.drivers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.RobotConstants;
import frc.robot.subsystems.swerve.Swerve;
import org.frcteam6941.looper.Updatable;
import org.littletonrobotics.junction.Logger;

public class DestinationSupplier implements Updatable {
    private static DestinationSupplier instance;
    Swerve swerve;
    private int targetTagID = 0;
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

    public static Pose2d getNearestTag(Pose2d robotPose) {
        double minDistance = Double.MAX_VALUE;
        int minDistanceID = 0;
        // Loop from 6 to 11
        for (int i = 6; i <= 11; i++) {
            if (minDistance > FieldConstants.officialAprilTagType.getLayout().getTagPose(i).get().
                    toPose2d().getTranslation().getDistance(robotPose.getTranslation())) {
                minDistanceID = i;
                minDistance = FieldConstants.officialAprilTagType.getLayout().getTagPose(i).get().
                        toPose2d().getTranslation().getDistance(robotPose.getTranslation());
            }
        }

        // Loop from 17 to 22
        for (int i = 17; i <= 22; i++) {
            if (minDistance > FieldConstants.officialAprilTagType.getLayout().getTagPose(i).get().
                    toPose2d().getTranslation().getDistance(robotPose.getTranslation())) {
                minDistanceID = i;
                minDistance = FieldConstants.officialAprilTagType.getLayout().getTagPose(i).get().
                        toPose2d().getTranslation().getDistance(robotPose.getTranslation());
            }
        }
        minDistance = Double.MAX_VALUE;
        Pose2d goal = FieldConstants.officialAprilTagType.getLayout().getTagPose(minDistanceID).get().toPose2d();

        return goal;
    }

    public static Pose2d getDriveTarget(Pose2d robot, Pose2d goal, boolean rightReef) {
        goal = getFinalDriveTarget(goal,rightReef);
        var offset = goal.relativeTo(robot);
        double yDistance = Math.abs(offset.getY());
        double xDistance = Math.abs(offset.getX());
        double shiftXT =
                MathUtil.clamp(
                        (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
                        0.0,
                        1.0);
        double shiftYT =
                MathUtil.clamp(yDistance <= 0.2 ? 0.0 : -offset.getX() / Reef.faceLength, 0.0, 1.0);
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
                        getInstance().currentElevSetpointCoral == elevatorSetpoint.L4 ?
                                RobotConstants.ReefAimConstants.ROBOT_TO_PIPE_L4_METERS.get():
                                RobotConstants.ReefAimConstants.ROBOT_TO_PIPE_METERS.get(),
                        RobotConstants.ReefAimConstants.PIPE_TO_TAG.magnitude() * (rightReef ? 1 : -1)),
                new Rotation2d()));
        return goal;
    }

    public static boolean isSafeToRaise(Pose2d robotPose, boolean rightReef) {
        Pose2d tag = getNearestTag(robotPose);
        Pose2d goal =  getFinalDriveTarget(tag,rightReef);
        return goal.getTranslation().getDistance(robotPose.getTranslation()) < RobotConstants.ReefAimConstants.RAISE_LIMIT_METERS.get();
    }

    public void updateElevatorSetpoint(elevatorSetpoint setpoint) {
        switch (setpoint) {
            case L1, L2, L3, L4:
                currentElevSetpointCoral = setpoint;
                Logger.recordOutput("DestinationSupplier/currentElevSetpointCoral", setpoint);
                break;
            case P1, P2:
                currentElevSetpointPoke = setpoint;
                Logger.recordOutput("DestinationSupplier/currentElevSetpointPoke", setpoint);
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
        Logger.recordOutput("DestinationSupplier/RightCoral", coralRight);
    }

    public void updateTagID(int tagID) {
        this.targetTagID = tagID;
    }

    public int getTargetTagID() {
        return targetTagID;
    }

    public boolean getCurrentBranch() {
        return coralRight;
    }


    @Override
    public void update(double time, double dt) {
    }

    public enum elevatorSetpoint {
        L1, L2, L3, L4, P1, P2
    }


}

