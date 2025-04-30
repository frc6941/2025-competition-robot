package frc.robot.subsystems.limelight;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;

public class LimelightIOReplay implements LimelightIO {
    private final String name;
    private LogTable outputTable;

    public LimelightIOReplay(String name) {
        this.name = name;
        // dirty hacks to get realOutputs...
        // (all necessary alerts about using reflection here)
        try {
            Class<Logger> loggerClass = Logger.class;
            Field realOutputField = loggerClass.getDeclaredField("outputTable");
            realOutputField.setAccessible(true);
            outputTable = (LogTable) realOutputField.get(null);
            if (outputTable == null) {
                throw new NoSuchFieldException();
            }
        } catch (NoSuchFieldException | IllegalAccessException e) {
            e.printStackTrace();
            System.out.println("should not happen: illegal access via reflection to get realOutputs");
        }
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs, AngularVelocity gyroRate) {
//        System.out.println(Arrays.toString(outputTable.getSubtable(name).get("estimatedPose").getDoubleArray()));
    }
}
