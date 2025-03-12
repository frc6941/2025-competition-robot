// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

// Constants related to AprilTag vision processing and camera configuration for the robot
public class AprilTagVisionConstants {
    // Threshold for determining if there is ambiguity in the detected AprilTag
    public static final double ambiguityThreshold = 0.2;
    // Time interval for logging target information in seconds
    public static final double targetLogTimeSecs = 0.1;
    // Margin to avoid field borders in meters
    public static final double fieldBorderMargin = 0.5;
    // Margin along the Z-axis for detecting targets in meters
    public static final double zMargin = 0.75;
    // Coefficient for calculating standard deviation in X and Y position estimates
    public static final double xyStdDevCoefficient = 0.006;
    // Coefficient for calculating standard deviation in theta (rotation) estimate
    public static final double thetaStdDevCoefficient = 0.01;

    // Factors used for scaling standard deviations in the camera measurements
    public static final double[] stdDevFactors = new double[]{1.0, 1, 1.0, 1};

    // Array of camera poses relative to the robot's coordinate system
    // forward as X+, leftward as Y+, upward as Z+, clockwise as yaw+
    public static final Pose3d[] cameraPoses =
            new Pose3d[]{
                    new Pose3d(
                            -0.19651,
                            0.29981,
                            0.31560,
                            new Rotation3d(0.0, 0.0, Units.degreesToRadians(201))),
                    new Pose3d(
                            -0.298739,
                            -0.293440,
                            0.198300,
                            new Rotation3d(0.0, Units.degreesToRadians(-35), Units.degreesToRadians(200))),
                    new Pose3d(
                            0.20556,
                            0.33419,
                            0.31560,
                            new Rotation3d(0.0, 0.0, Units.degreesToRadians(-21))),
                    new Pose3d(
                            0.19651,
                            -0.29981,
                            0.31560,
                            new Rotation3d(0.0, 0.0, Units.degreesToRadians(20))),
//                    new Pose3d(
//                            0.15540948732209714,
//                            -0.24762365813825055,
//                            0.37630667446147353,
//                            new Rotation3d(new Quaternion(0.9238472145814227, 0.06441497987427977, -0.3412469688366055, -0.16095819562163288))),
            };

    public static final Transform3d[] cameraError =
            new Transform3d[]{
                    new Transform3d(),
                    new Transform3d(),
                    new Transform3d(
                            0.01,
                            0.02,
                            0,
                            new Rotation3d()),
                    new Transform3d(
                            -0.01,
                            -0.02,
                            0,
                            new Rotation3d()),
            };

    // Instance names for the installed AprilTag vision cameras
    public static final String[] instanceNames =
            new String[]{"northstar_0", "northstar_1", "northstar_2", "northstar_3"};

    // Camera device IDs for the installed AprilTag vision cameras
    public static final String[] cameraIds =
            new String[]{
                    "/dev/video_cam1",
                    "/dev/video_cam2",
                    "/dev/video_cam3",
                    "/dev/video_cam4"
            };
}