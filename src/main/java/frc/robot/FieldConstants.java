// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import lombok.Getter;

import java.io.IOException;
import java.nio.file.Path;
import java.util.*;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
    //field layout
    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);
    public static final double startingLineX =
            Units.inchesToMeters(299.438); // Measured from the inside of starting line
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);
    public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.ANDYMARK;
    public static final AprilTagLayoutType officialAprilTagType = AprilTagLayoutType.ANDYMARK;

    @Getter
    public enum AprilTagLayoutType {
        OFFICIAL_OLD("2025-official"),
        WELDED("2025-reefscape-welded"),
        ANDYMARK("2025-reefscape-andymark"),
        CUSTOM("2025-custom");

        private final AprilTagFieldLayout layout;
        private final String layoutString;

        AprilTagLayoutType(String name) {
            if (RobotConstants.disableHAL) {
                layout = null;
            } else {
                try {
                    layout =
                            new AprilTagFieldLayout(
                                    Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json"));
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }
            if (layout == null) {
                layoutString = "";
            } else {
                try {
                    layoutString = new ObjectMapper().writeValueAsString(layout);
                } catch (JsonProcessingException e) {
                    throw new RuntimeException(
                            "Failed to serialize AprilTag layout JSON " + "for Northstar");
                }
            }
        }
    }

    public enum ReefHeight {
        L4(Units.inchesToMeters(72), -90),
        L3(Units.inchesToMeters(47.625), -35),
        L2(Units.inchesToMeters(31.875), -35),
        L1(Units.inchesToMeters(18), 0);

        public final double height;
        public final double pitch;

        ReefHeight(double height, double pitch) {
            this.height = height;
            this.pitch = pitch; // in degrees
        }
    }

    public enum ReefLevel {
        L1(Units.inchesToMeters(25.0), 0),
        L2(Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
        L3(Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
        L4(Units.inchesToMeters(72), -90);

        public final double height;
        public final double pitch;

        ReefLevel(double height, double pitch) {
            this.height = height;
            this.pitch = pitch; // Degrees
        }

        public static ReefLevel fromLevel(int level) {
            return Arrays.stream(values())
                    .filter(height -> height.ordinal() == level)
                    .findFirst()
                    .orElse(L4);
        }
    }

    public static class Processor {
        public static final Pose2d centerFace =
                new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
    }

    public static class Barge {
        public static final Translation2d farCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
        public static final Translation2d middleCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
        public static final Translation2d closeCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

        // Measured from floor to bottom of cage
        public static final double deepHeight = Units.inchesToMeters(3.125);
        public static final double shallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {
        public static final Pose2d leftCenterFace =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(291.176),
                        Rotation2d.fromDegrees(90 - 144.011));
        public static final Pose2d rightCenterFace =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(25.824),
                        Rotation2d.fromDegrees(144.011 - 90));
    }

    public static class Reef {
        public static final double faceLength = Units.inchesToMeters(36.792600);
        public static final Translation2d center =
                new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0);
        public static final double faceToZoneLine =
                Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

        public static final Pose2d[] centerFaces =
                new Pose2d[6]; // Starting facing the driver station in clockwise order
        public static final List<Map<ReefLevel, Pose3d>> branchPositions =
                new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise
        public static final List<Map<ReefLevel, Pose2d>> branchPositions2d = new ArrayList<>();

        static {
            // Initialize faces
            var aprilTagLayout = AprilTagLayoutType.ANDYMARK.getLayout();
            centerFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
            centerFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
            centerFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
            centerFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
            centerFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
            centerFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();

            // Initialize branch positions
            for (int face = 0; face < 6; face++) {
                Map<ReefLevel, Pose3d> fillRight = new HashMap<>();
                Map<ReefLevel, Pose3d> fillLeft = new HashMap<>();
                Map<ReefLevel, Pose2d> fillRight2d = new HashMap<>();
                Map<ReefLevel, Pose2d> fillLeft2d = new HashMap<>();
                for (var level : ReefLevel.values()) {
                    Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
                    double adjustX = Units.inchesToMeters(30.738);
                    double adjustY = Units.inchesToMeters(6.469);

                    var rightBranchPose =
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians()));
                    var leftBranchPose =
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians()));

                    fillRight.put(level, rightBranchPose);
                    fillLeft.put(level, leftBranchPose);
                    fillRight2d.put(level, rightBranchPose.toPose2d());
                    fillLeft2d.put(level, leftBranchPose.toPose2d());
                }
                branchPositions.add(fillRight);
                branchPositions.add(fillLeft);
                branchPositions2d.add(fillRight2d);
                branchPositions2d.add(fillLeft2d);
            }
        }
    }

    public static class StagingPositions {
        // Measured from the center of the ice cream
        public static final Pose2d leftIceCream =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
        public static final Pose2d middleIceCream =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
        public static final Pose2d rightIceCream =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
    }
}