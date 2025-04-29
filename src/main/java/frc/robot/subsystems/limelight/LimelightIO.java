package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {
    void setRobotOrientation(double yaw, double yawRate,
                             double pitch, double pitchRate,
                             double roll, double rollRate);

    default void clearNewEstimate(LimelightIOInputs inputs) {
        inputs.newEstimate = false;
    }

    void updateInputs(LimelightIOInputs inputs, AngularVelocity gyroRate);

    void setMegaTag2(boolean useMegaTag2);

    @AutoLog
    class LimelightIOInputs {
        public PoseEstimate poseRed;
        public PoseEstimate poseBlue;
        public Pose3d cameraPose;
        public boolean useMegaTag2;
        public boolean newEstimate;
    }

    record RawFiducial(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot,
                       double ambiguity) {
    }

    record PoseEstimate(Pose2d pose, double timestampSeconds, double latency,
                        int tagCount, double tagSpan, double avgTagDist,
                        double avgTagArea, RawFiducial[] rawFiducials, boolean isMegaTag2) {
    }
}
