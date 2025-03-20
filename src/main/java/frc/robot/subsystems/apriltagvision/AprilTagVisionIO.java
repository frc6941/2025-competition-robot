package frc.robot.subsystems.apriltagvision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;

public interface AprilTagVisionIO {
    void updateInputs(AprilTagVisionIOInputs inputs);

    @AutoLog
    class AprilTagVisionIOInputs {
        public List<PhotonPipelineResult> results;
        public String cameraName;
    }
}