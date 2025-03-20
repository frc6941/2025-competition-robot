package frc.robot.subsystems.apriltagvision;

import frc.robot.FieldConstants.AprilTagLayoutType;
import org.photonvision.PhotonCamera;

public class AprilTagVisionIOPhotonVision implements AprilTagVisionIO {

    private final PhotonCamera camera;
    private final AprilTagLayoutType aprilTagType;

    public AprilTagVisionIOPhotonVision(AprilTagLayoutType aprilTagType, String cameraName) {
        this.aprilTagType = aprilTagType;
        this.camera = new PhotonCamera(cameraName);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        inputs.results = camera.getAllUnreadResults();
        inputs.cameraName = camera.getName();
    }

}