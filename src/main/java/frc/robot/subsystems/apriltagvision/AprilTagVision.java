// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//TODO: fix loop time overrun (ApriltagVision cost ~0.025s)

package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.GeomUtil;
import org.littletonrobotics.LoggedTunableNumber;
import org.littletonrobotics.RobotState;
import org.littletonrobotics.junction.Logger;

import java.util.*;
import java.util.function.Supplier;

import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.*;
import static frc.robot.subsystems.apriltagvision.AprilTagVisionIO.AprilTagVisionIOInputs;
import static org.littletonrobotics.RobotState.VisionObservation;

/**
 * Vision subsystem for AprilTag vision.
 */
@ExtensionMethod({GeomUtil.class})
public class AprilTagVision extends SubsystemBase {
    // Serial
    // public static final SerialPort serial = new SerialPort(RobotConstants.baudRate, RobotConstants.VisionConstants.serialPort);

    private static final LoggedTunableNumber timestampOffset =
            new LoggedTunableNumber("AprilTagVision/TimestampOffset", -(1.0 / 50.0));
    private static final double demoTagPosePersistenceSecs = 0.5;
    private final Supplier<AprilTagLayoutType> aprilTagTypeSupplier;
    private final AprilTagVisionIO[] io;
    private final AprilTagVisionIOInputs[] inputs;
    private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
    private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();
    private final DestinationSupplier destinationSupplier = DestinationSupplier.getInstance();
    private Pose3d demoTagPose = null;
    private double lastDemoTagPoseTimestamp = 0.0;
    private double lastPrint;
    private double frameUpdateCount;
    @Getter
    private ArrayList<Pose3d> allTagPoses;
    @Getter
    private Pose3d cameraPose;
    @Getter
    private Pose3d robotPose3d;
    @Setter
    private int measureCnt = 0;

    /**
     * Constructs the AprilTagVision subsystem with a supplier for the AprilTag layout type and an array of IO instances.
     */
    public AprilTagVision(Supplier<AprilTagLayoutType> aprilTagTypeSupplier, AprilTagVisionIO... io) {
        this.aprilTagTypeSupplier = aprilTagTypeSupplier;
        this.io = io;
        inputs = new AprilTagVisionIOInputs[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new AprilTagVisionIOInputs();
        }

        // Create map of last frame times for instances
        for (int i = 0; i < io.length; i++) {
            lastFrameTimes.put(i, 0.0);
        }

        // Disable serial termination of \n
//        serial.disableTermination();
//        serial.reset();
    }

    /**
     * Called periodically to update the vision subsystem with new data from IO instances.
     */
    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("AprilTagVision/Inst" + i, inputs[i]);
        }


        // Loop over instances to process all frames and poses
        List<Pose2d> allRobotPoses = new ArrayList<>();
        List<Pose3d> allRobotPoses3d = new ArrayList<>();
        List<VisionObservation> allVisionObservations = new ArrayList<>();

        for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
            Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/CameraPoseConstants", cameraPoses[instanceIndex]);
            // Loop over frames to extract and process data
            for (int frameIndex = 0; frameIndex < inputs[instanceIndex].timestamps.length; frameIndex++) {
                lastFrameTimes.put(instanceIndex, Timer.getFPGATimestamp());
                var timestamp = inputs[instanceIndex].timestamps[frameIndex] + timestampOffset.get();
                var values = inputs[instanceIndex].frames[frameIndex];

                // Handle blank frame by continuing to the next iteration
                if (values.length == 0 || values[0] == 0) {
                    continue;
                }

                // Determine how to process the poses based on the number of poses detected in the frame
                cameraPose = null;
                robotPose3d = null;
                boolean useVisionRotation = false;
                switch ((int) values[0]) {
                    case 1:
                        // Process a single pose (multi-tag scenario)
                        cameraPose =
                                new Pose3d(
                                        values[2],
                                        values[3],
                                        values[4],
                                        new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                        robotPose3d =
                                cameraPose.transformBy(cameraPoses[instanceIndex].toTransform3d().inverse());
                        useVisionRotation = true;
                        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/CameraPose", cameraPose);

                        break;
                    case 2:
                        // Process two poses and disambiguate based on error
                        double error0 = values[1];
                        double error1 = values[9];
                        Pose3d cameraPose0 =
                                new Pose3d(
                                        values[2],
                                        values[3],
                                        values[4],
                                        new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                        Pose3d cameraPose1 =
                                new Pose3d(
                                        values[10],
                                        values[11],
                                        values[12],
                                        new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
                        Pose3d robotPose3d0 =
                                cameraPose0.transformBy(cameraPoses[instanceIndex].toTransform3d().inverse());
                        Pose3d robotPose3d1 =
                                cameraPose1.transformBy(cameraPoses[instanceIndex].toTransform3d().inverse());

                        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/robotPose3d0", robotPose3d0);
                        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/robotPose3d1", robotPose3d1);

                        // Select the most likely pose based on the estimated rotation
                        if (error0 < error1 * ambiguityThreshold || error1 < error0 * ambiguityThreshold) {
                            Rotation2d currentRotation =
                                    Swerve.getInstance().getLocalizer().getCoarseFieldPose(timestamp).getRotation();
                            Rotation2d visionRotation0 = robotPose3d0.toPose2d().getRotation();
                            Rotation2d visionRotation1 = robotPose3d1.toPose2d().getRotation();
                            if (Math.abs(currentRotation.minus(visionRotation0).getRadians())
                                    < Math.abs(currentRotation.minus(visionRotation1).getRadians())) {
                                cameraPose = cameraPose0;
                                robotPose3d = robotPose3d0;
                            } else {
                                cameraPose = cameraPose1;
                                robotPose3d = robotPose3d1;
                            }
                        }
                        break;
                }

                // Skip further processing if no valid data is available
                if (cameraPose == null || robotPose3d == null) {
                    continue;
                }

                // Validate that the robot pose is within the field boundaries
                if (robotPose3d.getX() < -fieldBorderMargin
                        || robotPose3d.getX() > frc.robot.FieldConstants.fieldLength + fieldBorderMargin
                        || robotPose3d.getY() < -fieldBorderMargin
                        || robotPose3d.getY() > frc.robot.FieldConstants.fieldWidth + fieldBorderMargin
                        || robotPose3d.getZ() < -zMargin
                        || robotPose3d.getZ() > zMargin) {
                    continue;
                }

                // Convert the 3D robot pose to a 2D pose
                Pose2d robotPose = robotPose3d.plus(cameraError[instanceIndex]).toPose2d();

                // Collect tag poses and update the last detection times for each tag
                List<Pose3d> tagPoses = new ArrayList<>();
                for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i++) {
                    int tagId = (int) values[i];
                    lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
                    Optional<Pose3d> tagPose =
                            aprilTagTypeSupplier.get().getLayout().getTagPose((int) values[i]);
                    tagPose.ifPresent(tagPoses::add);
                }
                if (tagPoses.isEmpty()) continue;

                // Compute the average distance from the camera to the detected tags
                double totalDistance = 0.0;
                for (Pose3d tagPose : tagPoses) {
                    totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
                }
                double avgDistance = totalDistance / tagPoses.size();

                // Create a vision observation based on the robot pose, timestamp, and calculated standard deviations
                double xyStdDev =
                        /*instanceIndex <= 1 ? xyStdDevCoefficient * 4 : */xyStdDevCoefficient
                        * Math.pow(avgDistance, 2.0)
                        / tagPoses.size()
                        * stdDevFactors[instanceIndex];
                double thetaStdDev =
                        useVisionRotation
                                ? thetaStdDevCoefficient
                                * Math.pow(avgDistance, 2.0)
                                / tagPoses.size()
                                * stdDevFactors[instanceIndex]
                                : Double.POSITIVE_INFINITY;
                allVisionObservations.add(
                        new VisionObservation(
                                robotPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
                allRobotPoses.add(robotPose);
                allRobotPoses3d.add(robotPose3d);
                if (robotPose3d != null) {
                    if (measureCnt <= 5 && Swerve.getInstance().getState() == Swerve.State.PATH_FOLLOWING) {
                        measureCnt++;
                        xyStdDev = 0.01;
                    }
                    if (instanceIndex >= 2 && destinationSupplier.isUseVision()) {
                        Swerve.getInstance().getLocalizer().addMeasurement(
                                timestamp,
                                robotPose3d.toPose2d(),
                                new Pose2d(new Translation2d(xyStdDev, xyStdDev),
                                        Rotation2d.fromDegrees(thetaStdDev)));
                    }
                    SmartDashboard.putBoolean("TargetUpdated", true);
                } else {
                    SmartDashboard.putBoolean("TargetUpdated", false);
                }
                frameUpdateCount += 1;

                // Log the latency and robot pose information for the current instance
                Logger.recordOutput(
                        "AprilTagVision/Inst" + instanceIndex + "/LatencySecs",
                        Timer.getFPGATimestamp() - timestamp);
                Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", robotPose);
                Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", robotPose3d);
                Logger.recordOutput(
                        "AprilTagVision/Inst" + instanceIndex + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
            }
            // Record demo tag pose if available
            if (inputs[instanceIndex].demoFrame.length > 0) {
                var values = inputs[instanceIndex].demoFrame;
                double error0 = values[0];
                double error1 = values[8];
                Pose3d fieldToCameraPose =
                        new Pose3d(RobotState.getInstance().getEstimatedPose())
                                .transformBy(cameraPoses[instanceIndex].toTransform3d());
                Pose3d fieldToTagPose0 =
                        fieldToCameraPose.transformBy(
                                new Transform3d(
                                        new Translation3d(values[1], values[2], values[3]),
                                        new Rotation3d(new Quaternion(values[4], values[5], values[6], values[7]))));
                Pose3d fieldToTagPose1 =
                        fieldToCameraPose.transformBy(
                                new Transform3d(
                                        new Translation3d(values[9], values[10], values[11]),
                                        new Rotation3d(
                                                new Quaternion(values[12], values[13], values[14], values[15]))));
                Pose3d fieldToTagPose;

                // Determine the best pose based on error values and ambiguity threshold
                if (demoTagPose == null && error0 < error1) {
                    fieldToTagPose = fieldToTagPose0;
                } else if (demoTagPose == null && error0 >= error1) {
                    fieldToTagPose = fieldToTagPose1;
                } else if (error0 < error1 * ambiguityThreshold) {
                    fieldToTagPose = fieldToTagPose0;
                } else if (error1 < error0 * ambiguityThreshold) {
                    fieldToTagPose = fieldToTagPose1;
                } else {
                    var pose0Quaternion = fieldToTagPose0.getRotation().getQuaternion();
                    var pose1Quaternion = fieldToTagPose1.getRotation().getQuaternion();
                    var referenceQuaternion = demoTagPose.getRotation().getQuaternion();
                    double pose0Distance =
                            Math.acos(
                                    pose0Quaternion.getW() * referenceQuaternion.getW()
                                            + pose0Quaternion.getX() * referenceQuaternion.getX()
                                            + pose0Quaternion.getY() * referenceQuaternion.getY()
                                            + pose0Quaternion.getZ() * referenceQuaternion.getZ());
                    double pose1Distance =
                            Math.acos(
                                    pose1Quaternion.getW() * referenceQuaternion.getW()
                                            + pose1Quaternion.getX() * referenceQuaternion.getX()
                                            + pose1Quaternion.getY() * referenceQuaternion.getY()
                                            + pose1Quaternion.getZ() * referenceQuaternion.getZ());
                    if (pose0Distance > Math.PI / 2) {
                        pose0Distance = Math.PI - pose0Distance;
                    }
                    if (pose1Distance > Math.PI / 2) {
                        pose1Distance = Math.PI - pose1Distance;
                    }
                    if (pose0Distance < pose1Distance) {
                        fieldToTagPose = fieldToTagPose0;
                    } else {
                        fieldToTagPose = fieldToTagPose1;
                    }
                }

                // Save the determined pose if it's not null
                if (fieldToTagPose != null) {
                    demoTagPose = fieldToTagPose;
                    lastDemoTagPoseTimestamp = Timer.getFPGATimestamp();
                }

                // Clear robot pose if no frames from instances
                if (inputs[instanceIndex].timestamps.length == 0) {
                    Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", new Pose2d());
                    Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", new Pose3d());
                }

                // Clear tag poses if no recent frames from instance
                if (Timer.getFPGATimestamp() - lastFrameTimes.get(instanceIndex) > targetLogTimeSecs) {
                    Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/TagPoses", new Pose3d[]{});
                }
            }

            // Clear demo tag pose if it's been too long since last detection
            if (Timer.getFPGATimestamp() - lastDemoTagPoseTimestamp > demoTagPosePersistenceSecs) {
                demoTagPose = null;
            }

            // Log all detected robot poses
            Logger.recordOutput("AprilTagVision/RobotPoses", allRobotPoses.toArray(Pose2d[]::new));
            Logger.recordOutput("AprilTagVision/RobotPoses3d", allRobotPoses3d.toArray(Pose3d[]::new));

            // Log all recently detected tag poses
            List<Pose3d> allTagPoses = new ArrayList<>();
            for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
                if (Timer.getFPGATimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
                    aprilTagTypeSupplier
                            .get()
                            .getLayout()
                            .getTagPose(detectionEntry.getKey())
                            .ifPresent(allTagPoses::add);
                }
            }
            Logger.recordOutput("AprilTagVision/TagPoses", allTagPoses.toArray(Pose3d[]::new));

            // Log the demo tag pose and its ID
            if (demoTagPose == null) {
                Logger.recordOutput("AprilTagVision/DemoTagPose", new Pose3d[]{});
            } else {
                Logger.recordOutput("AprilTagVision/DemoTagPose", demoTagPose);
            }
            Logger.recordOutput("AprilTagVision/DemoTagPoseId", new long[]{29});

        }
        RobotState.getInstance().setDemoTagPose(demoTagPose);

    }


    // Return the closest detected tag pose to the robot's current pose
    public Pose3d getClosestTagPose() {
        // If no tag poses are detected, return null
        if (allTagPoses.isEmpty()) {
            return null;
        }

        // Get the robot's current pose
        Pose3d robotPose = this.robotPose3d;

        // Initialize variables to track the closest tag pose and minimum distance
        Pose3d closestTagPose = null;
        double minDistance = Double.MAX_VALUE;

        // Loop through all tag poses and find the closest one
        for (Pose3d tagPose : allTagPoses) {
            double distance = robotPose.getTranslation().getDistance(tagPose.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                closestTagPose = tagPose;
            }
        }

        // Return the closest tag pose
        return closestTagPose;
    }
}
