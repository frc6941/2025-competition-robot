package frc.robot.auto.basics;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.json.simple.parser.ParseException;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class AutoFile {
    private final AutoActions autoActions;
    private final Map<String, PathPlannerPath> autoPaths = new HashMap<>();

    public AutoFile(AutoActions autoActions) {
        this.autoActions = autoActions;
        initializeAutoPaths();
    }

    private void initializeAutoPaths() {
        File[] files = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();
        assert files != null;
        for (File file : files) {
            try {
                // path files without extension
                PathPlannerPath path = PathPlannerPath.fromPathFile(file.getName().replaceFirst("[.][^.]+$", ""));
                autoPaths.put(path.name, path);
            } catch (IOException | ParseException e) {
                throw new IllegalArgumentException("Failed to parse path file: " + file.getName(), e);
            }
        }
    }

    private PathPlannerPath getAutoPath(String path) {
        assert autoPaths.containsKey(path);
        return autoPaths.get(path);
    }

    public Command runAuto(String autoName) {
        switch (autoName) {
            case "4CoralUp":
                return build4CoralUp();
            default:
                throw new IllegalArgumentException("No corresponding auto named " + autoName);
        }
    }

    private Command build4CoralUp() {
        return new SequentialCommandGroup(
                autoActions.followPath(getAutoPath("S1-P3-1"), true, true, true),
                autoActions.shootCoralAtSetpoint(),
                autoActions.followPath(getAutoPath("P3-I1"), true, true, false)
                // FIXME: not intaken?
//                autoActions.deployIntake(),
//                autoActions.followPath(getAutoPath("I1-P2-1"), true, true, false),
//                autoActions.shootCoralAtSetpoint(),
//                autoActions.followPath(getAutoPath("P2-1-I2"), true, true, false),
//                autoActions.followPath(getAutoPath("I2-P1-2"), true, true, false),
//                autoActions.shootCoralAtSetpoint(),
//                autoActions.followPath(getAutoPath("P1-2-I3"), true, true, false),
//                autoActions.followPath(getAutoPath("I3-P1-1"), true, true, false),
//                autoActions.shootCoralAtSetpoint()
        );
    }
}
