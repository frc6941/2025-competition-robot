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

import static frc.robot.drivers.DestinationSupplier.elevatorSetpoint.L4;

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
        return switch (autoName) {
            case "4CoralLeft" -> build4CoralLeft();
            case "4CoralRight" -> build4CoralRight();
            case "FunnelRight" -> buildFunnelRight();
            case "FunnelLeft" -> buildFunnelLeft();
            case "Test" -> buildTest();
            default -> autoActions.ReverseEndEffector();
            //default -> throw new IllegalArgumentException("No corresponding auto named " + autoName);
        };
    }

    private Command buildTest() {
        //return autoActions.zeroAndIntake();
        return autoActions.AutoAimShoot(L4, 'C');
    }

    private Command build4CoralLeft() {
        return new SequentialCommandGroup(
                autoActions.ReverseEndEffector(),
                autoActions.AutoAimShoot(L4, 'I'),
                autoActions.disableVision(),
                autoActions.followPath(getAutoPath("IJ-L1"), true, true, false),
                autoActions.enableVision(),
                autoActions.AutoAimShoot(L4, 'L'),
                autoActions.disableVision(),
                autoActions.followPath(getAutoPath("L-I2"), true, true, false),
                autoActions.enableVision(),
                autoActions.AutoAimShoot(L4, 'B'),
                autoActions.disableVision(),
                autoActions.followPath(getAutoPath("B-I3"), true, true, false),
                autoActions.enableVision(),
                autoActions.AutoAimShoot(L4, 'C')
        );
    }

    private Command build4CoralRight() {
        return new SequentialCommandGroup(
                autoActions.ReverseEndEffector(),
                autoActions.AutoAimShoot(L4, 'F'),
                autoActions.disableVision(),
                autoActions.followPath(getAutoPath("EF-I3"), true, true, false),
                autoActions.enableVision(),
                autoActions.AutoAimShoot(L4, 'C'),
                autoActions.disableVision(),
                autoActions.followPath(getAutoPath("C-I2"), true, true, false),
                autoActions.enableVision(),
                autoActions.AutoAimShoot(L4, 'A'),
                autoActions.disableVision(),
                autoActions.followPath(getAutoPath("A-I1"), true, true, false),
                autoActions.enableVision(),
                autoActions.AutoAimShoot(L4, 'L')
        );
    }

    private Command buildFunnelRight() {
        return new SequentialCommandGroup(
                autoActions.ReverseEndEffector(),
                autoActions.AutoAimShoot(L4, 'F'),
                autoActions.followPath(getAutoPath("EF-FUNNEL-R"), true, true, false),
                autoActions.AutoAimShoot(L4, 'D'),
                autoActions.followPath(getAutoPath("D-I3"), true, true, false),
                autoActions.AutoAimShoot(L4, 'C'),
                autoActions.followPath(getAutoPath("C-I2"), true, true, false),
                autoActions.AutoAimShoot(L4, 'B')
        );
    }

    private Command buildFunnelLeft() {
        return new SequentialCommandGroup(
                autoActions.ReverseEndEffector(),
                autoActions.AutoAimShoot(L4, 'I'),
                autoActions.followPath(getAutoPath("IJ-FUNNEL-L"), true, true, false),
                autoActions.AutoAimShoot(L4, 'K'),
                autoActions.followPath(getAutoPath("K-I3"), true, true, false),
                autoActions.AutoAimShoot(L4, 'L'),
                autoActions.followPath(getAutoPath("L-I2"), true, true, false),
                autoActions.AutoAimShoot(L4, 'A')
        );
    }
}
