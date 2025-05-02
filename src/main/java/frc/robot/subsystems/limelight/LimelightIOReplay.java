package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;

public class LimelightIOReplay implements LimelightIO {
    private LogTable outputTable;

    public LimelightIOReplay(String name) {
        // dirty hacks to get realOutputs...
        // (all necessary alerts about using reflection here)
        try {
            Class<Logger> loggerClass = Logger.class;
            Field realOutputField = loggerClass.getDeclaredField("entry");
            realOutputField.setAccessible(true);
            LogTable entry = (LogTable) realOutputField.get(null);
            if (entry == null) {
                throw new NoSuchFieldException();
            }
            outputTable = entry.getSubtable("RealOutputs").getSubtable(name);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            e.printStackTrace();
            System.out.println("should not happen: illegal access via reflection to get realOutputs");
        }
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs, AngularVelocity gyroRate) {
        inputs.useMegaTag2 = outputTable.get("useMegaTag2").getBoolean();
        inputs.newEstimate = outputTable.get("hasEstimate").getBoolean();
        inputs.poseRed = new PoseEstimate(
                outputTable.get("estimatedPose", new Pose2d()),
                outputTable.getTimestamp() / 1000000.0,

                // following fields are not logged and have no way of restoring.
                0,
                999, // many tag counts to avoid update rejection
                0,
                0,
                0,
                new RawFiducial[]{},
                inputs.useMegaTag2
        );
        inputs.poseBlue = new PoseEstimate(
                outputTable.get("estimatedPose", new Pose2d()),
                outputTable.getTimestamp() / 1000000.0,

                // following fields are not logged and have no way of restoring.
                0,
                999, // many tag counts to avoid update rejection
                0,
                0,
                0,
                new RawFiducial[]{},
                inputs.useMegaTag2
        );
    }
}
