package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class LimelightIOReal implements LimelightIO {
    private final String name;
    private final Map<String, DoubleArrayEntry> doubleArrayEntries = new ConcurrentHashMap<>();
    private boolean useMegaTag2 = true;

    public LimelightIOReal(String name) {
        this.name = name;
    }

    // Helpers
    public static Pose3d toPose3D(double[] inData) {
        if (inData.length < 6) {
            // bad data
            return new Pose3d();
        }
        return new Pose3d(
                new Translation3d(inData[0], inData[1], inData[2]),
                new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                        Units.degreesToRadians(inData[5])));
    }

    public static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            // bad data
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    private static double extractArrayEntry(double[] inData, int position) {
        if (inData.length < position + 1) {
            return 0;
        }
        return inData[position];
    }

    // Retrievals
    private NetworkTableEntry getEntry(String entryName) {
        return NetworkTableInstance.getDefault().getTable(name).getEntry(entryName);
    }

    public DoubleArrayEntry getDoubleArrayEntry(String entryName) {
        String key = name + "/" + entryName;
        return doubleArrayEntries.computeIfAbsent(key, k -> {
            NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
            return table.getDoubleArrayTopic(entryName).getEntry(new double[0]);
        });
    }

    private PoseEstimate getBotPoseEstimate(String entryName, boolean isMegaTag2) {
        DoubleArrayEntry poseEntry = getDoubleArrayEntry(entryName);

        TimestampedDoubleArray tsValue = poseEntry.getAtomic();
        double[] poseArray = tsValue.value;
        long timestamp = tsValue.timestamp;

        if (poseArray.length == 0) {
            // Handle the case where no data is available
            return null; // or some default PoseEstimate
        }

        var pose = toPose2D(poseArray);
        double latency = extractArrayEntry(poseArray, 6);
        int tagCount = (int) extractArrayEntry(poseArray, 7);
        double tagSpan = extractArrayEntry(poseArray, 8);
        double tagDist = extractArrayEntry(poseArray, 9);
        double tagArea = extractArrayEntry(poseArray, 10);

        // Convert server timestamp from microseconds to seconds and adjust for latency
        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

        RawFiducial[] rawFiducials = new RawFiducial[tagCount];
        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;

        if (poseArray.length != expectedTotalVals) {
            // Don't populate fiducials
        } else {
            for (int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = (int) poseArray[baseIndex];
                double txnc = poseArray[baseIndex + 1];
                double tync = poseArray[baseIndex + 2];
                double ta = poseArray[baseIndex + 3];
                double distToCamera = poseArray[baseIndex + 4];
                double distToRobot = poseArray[baseIndex + 5];
                double ambiguity = poseArray[baseIndex + 6];
                rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }

        return new PoseEstimate(pose, adjustedTimestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials, isMegaTag2);
    }

    public Pose3d getCameraPose() {
        double[] poseArray = getEntry("camerapose_robotspace").getDoubleArray(new double[0]);
        return toPose3D(poseArray);
    }

    @Override
    public void setRobotOrientation(double yaw, double yawRate,
                                    double pitch, double pitchRate,
                                    double roll, double rollRate) {
        double[] entries = new double[6];
        entries[0] = yaw;
        entries[1] = yawRate;
        entries[2] = pitch;
        entries[3] = pitchRate;
        entries[4] = roll;
        entries[5] = rollRate;
        getEntry("robot_orientation_set").setDoubleArray(entries);
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void setMegaTag2(boolean useMegaTag2) {
        this.useMegaTag2 = useMegaTag2;
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs, AngularVelocity gyroRate) {
        PoseEstimate poseBlue;
        PoseEstimate poseRed;
        if (useMegaTag2) {
            poseBlue = getBotPoseEstimate("botpose_orb_wpiblue", true);
            poseRed = getBotPoseEstimate("botpose_orb_wpired", true);
        } else {
            poseBlue = getBotPoseEstimate("botpose_wpiblue", false);
            poseRed = getBotPoseEstimate("botpose_wpired", false);
        }
        inputs.poseRed = poseRed;
        inputs.poseBlue = poseBlue;
        inputs.cameraPose = getCameraPose();
        inputs.useMegaTag2 = useMegaTag2;
    }
}
