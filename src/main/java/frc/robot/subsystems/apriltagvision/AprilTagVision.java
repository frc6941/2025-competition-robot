package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.subsystems.swerve.Swerve;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.GeomUtil;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;

import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.ambiguityThreshold;
import static frc.robot.subsystems.apriltagvision.AprilTagVisionIO.AprilTagVisionIOInputs;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;

/**
 * Vision subsystem for AprilTag vision.
 */
@ExtensionMethod({GeomUtil.class})
public class AprilTagVision extends SubsystemBase {

    private final AprilTagLayoutType aprilTagType;
    private final AprilTagVisionIO[] io;
    private final AprilTagVisionIOInputs[] inputs;
    private final PhotonPoseEstimator[] poseEstimator;
    private final Swerve swerve = Swerve.getInstance();

    /**
     * Constructs the AprilTagVision subsystem with a supplier for the AprilTag layout type and an array of IO instances.
     */
    public AprilTagVision(AprilTagLayoutType aprilTagType, AprilTagVisionIO... io) {
        this.aprilTagType = aprilTagType;
        this.io = io;
        inputs = new AprilTagVisionIOInputs[io.length];
        poseEstimator = new PhotonPoseEstimator[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new AprilTagVisionIOInputs();
            poseEstimator[i] = new PhotonPoseEstimator(
                    aprilTagType.getLayout(),
                    MULTI_TAG_PNP_ON_COPROCESSOR,
                    AprilTagVisionConstants.cameraToRobot[i]);
            poseEstimator[i].setMultiTagFallbackStrategy(PNP_DISTANCE_TRIG_SOLVE);
        }
    }

    /**
     * Called periodically to update the vision subsystem with new data from IO instances.
     */
    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            List<PhotonPipelineResult> results = inputs[i].results;
            for (PhotonPipelineResult result : results) {
                // Reject single tag with high ambiguity
                if (result.getMultiTagResult().isEmpty() && result.getBestTarget().poseAmbiguity > ambiguityThreshold) {
                    Logger.recordOutput("AprilTagVision/" + inputs[i].cameraName + "/VisionUpdated", false);
                    continue;
                }
                Logger.recordOutput("AprilTagVision/" + inputs[i].cameraName + "/VisionUpdated", true);

                // Add Swerve Heading for MultiTagFallbackStrategy
                poseEstimator[i].addHeadingData(Timer.getFPGATimestamp(), swerve.getGyro().getYaw());
                Optional<EstimatedRobotPose> estimatedPose = poseEstimator[i].update(result);
                if (estimatedPose.isPresent()) {
                    swerve.getLocalizer().addMeasurement(estimatedPose.get().timestampSeconds,
                            estimatedPose.get().estimatedPose.toPose2d(),
                            new Pose2d(0.7, 0.7, new Rotation2d(Double.POSITIVE_INFINITY)));
                    Logger.recordOutput("AprilTagVision/" + inputs[i].cameraName + "/EstimatedRobotPose", estimatedPose.get().estimatedPose);
                    Logger.recordOutput("AprilTagVision/" + inputs[i].cameraName + "/EstimatedStrategy", estimatedPose.get().strategy);
                    Logger.recordOutput("AprilTagVision/" + inputs[i].cameraName + "/EstimatedTime", estimatedPose.get().timestampSeconds);
                }
                Logger.recordOutput("AprilTagVision/" + inputs[i].cameraName + "/Result", result);
            }
        }
    }
}