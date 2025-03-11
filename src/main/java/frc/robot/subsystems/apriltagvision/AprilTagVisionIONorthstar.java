// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.AprilTagLayoutType;

import java.util.function.Supplier;

import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.cameraIds;
import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.instanceNames;

// This class implements the AprilTagVisionIO interface for the Northstar vision system.
// It handles the configuration and data retrieval for AprilTag detection using NetworkTables.
public class AprilTagVisionIONorthstar implements AprilTagVisionIO {
    // use v4l2-ctl -d /dev/video0 --list-ctrls
    private static final int cameraResolutionWidth = 1280;
    private static final int cameraResolutionHeight = 720;
    private static final int cameraAutoExposure = 1;
    private static final int cameraExposure = 50;
    private static final int cameraGain = 1;
    private static final int fps = 120;
    private static final int brightness = 0;
    private static final int contrast = 60;
    private static final int buffersize = 1;
    private static final double disconnectedTimeout = 0.5;
    private final Supplier<AprilTagLayoutType> aprilTagTypeSupplier;
    private final DoubleArraySubscriber observationSubscriber;
    private final DoubleArraySubscriber demoObservationSubscriber;
    private final IntegerSubscriber fpsSubscriber;
    private final Timer disconnectedTimer = new Timer();
    private final AprilTagLayoutType lastAprilTagType = null;

    // Constructor for AprilTagVisionIONorthstar.
    // Initializes the configuration and subscribers for the given camera index.
    public AprilTagVisionIONorthstar(Supplier<AprilTagLayoutType> aprilTagTypeSupplier, int index) {
        this.aprilTagTypeSupplier = aprilTagTypeSupplier;
        var northstarTable = NetworkTableInstance.getDefault().getTable(instanceNames[index]);
        var configTable = northstarTable.getSubTable("config");
        configTable.getStringTopic("camera_id").publish().set(cameraIds[index]);
        configTable.getIntegerTopic("camera_resolution_width").publish().set(cameraResolutionWidth);
        configTable.getIntegerTopic("camera_resolution_height").publish().set(cameraResolutionHeight);
        configTable.getIntegerTopic("camera_auto_exposure").publish().set(cameraAutoExposure);
        configTable.getIntegerTopic("camera_exposure").publish().set(cameraExposure);
        configTable.getIntegerTopic("camera_gain").publish().set(cameraGain);
        configTable.getIntegerTopic("fps").publish().set(fps);
        configTable.getIntegerTopic("brightness").publish().set(brightness);
        configTable.getIntegerTopic("contrast").publish().set(contrast);
        configTable.getIntegerTopic("buffersize").publish().set(buffersize);
        configTable.getDoubleTopic("fiducial_size_m").publish().set(FieldConstants.aprilTagWidth);

        var outputTable = northstarTable.getSubTable("output");
        observationSubscriber =
                outputTable
                        .getDoubleArrayTopic("observations")
                        .subscribe(
                                new double[]{}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        demoObservationSubscriber =
                outputTable
                        .getDoubleArrayTopic("demo_observations")
                        .subscribe(
                                new double[]{}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);

        disconnectedTimer.start();
    }

    // Updates the inputs with the latest data from the NetworkTables.
    // Publishes the current AprilTag layout and retrieves observations and FPS information.
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        // Get observations
        var queue = observationSubscriber.readQueue();
        inputs.timestamps = new double[queue.length];
        inputs.frames = new double[queue.length][];
        for (int i = 0; i < queue.length; i++) {
            inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
            inputs.frames[i] = queue[i].value;
        }
        inputs.demoFrame = new double[]{};
        for (double[] demoFrame : demoObservationSubscriber.readQueueValues()) {
            inputs.demoFrame = demoFrame;
        }
        inputs.fps = fpsSubscriber.get();

        // Update disconnected alert
        if (queue.length > 0) {
            disconnectedTimer.reset();
        }
    }
}