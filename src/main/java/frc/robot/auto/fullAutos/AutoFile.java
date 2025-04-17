package frc.robot.auto.fullAutos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
            case "1Coral1AlgaeMiddle" -> build1Coral1AlgaeMiddle();
            case "1Coral3AlgaeMiddleLeftFirst" -> build1Coral3AlgaeMiddleLeftFirst();
            case "1Coral3AlgaeMiddleRightFirst" -> build1Coral3AlgaeMiddleRightFirst();
            case "Test" -> buildTest();
            default -> Commands.none();
        };
    }

    private Command buildTest() {
        return autoActions.shootAlgaeNet();
    }

    private Command build1Coral1AlgaeMiddle() {
        return new SequentialCommandGroup(
                autoActions.AutoAimShoot(L4, 'H'), // H-left G-right
                autoActions.intakeAlgae(),
                autoActions.followPath(getAutoPath("HG-Net"), true, true, false),
                autoActions.shootAlgaeNet(),
                autoActions.followPath(getAutoPath("Net-Leave"), true, true, false)
        );
    }

    private Command build1Coral3AlgaeMiddleLeftFirst() {
        return new SequentialCommandGroup(
                autoActions.AutoAimShoot(L4, 'H'), // H-left G-right
                autoActions.intakeAlgae(),
                autoActions.followPath(getAutoPath("HG-Net"), true, true, false),
                autoActions.shootAlgaeNet(),
                autoActions.followPath(getAutoPath("Net-IJ"), true, true, false),
                autoActions.intakeAlgae(),
                autoActions.followPath(getAutoPath("IJ-Net"), true, true, false),
                autoActions.shootAlgaeNet(),
                autoActions.followPath(getAutoPath("Net-EF"), true, true, false),
                autoActions.intakeAlgae(),
                autoActions.followPath(getAutoPath("EF-Net"), true, true, false),
                autoActions.shootAlgaeNet()
        );
    }

    private Command build1Coral3AlgaeMiddleRightFirst() {
        return new SequentialCommandGroup(
                autoActions.AutoAimShoot(L4, 'H'), // H-left G-right
                autoActions.intakeAlgae(),
                autoActions.followPath(getAutoPath("HG-Net"), true, true, false),
                autoActions.shootAlgaeNet(),
                autoActions.followPath(getAutoPath("Net-EF"), true, true, false),
                autoActions.intakeAlgae(),
                autoActions.followPath(getAutoPath("EF-Net"), true, true, false),
                autoActions.shootAlgaeNet(),
                autoActions.followPath(getAutoPath("Net-IJ"), true, true, false),
                autoActions.intakeAlgae(),
                autoActions.followPath(getAutoPath("IJ-Net"), true, true, false),
                autoActions.shootAlgaeNet()
        );
    }

    private Command build4CoralLeft() {
        return new SequentialCommandGroup(
                autoActions.AutoAimShoot(L4, 'I'),
                autoActions.followPath(getAutoPath("IJ-I1"), true, true, false),
                autoActions.waitFor(0.5).until(autoActions::isIntakeFinished),
                Commands.either(
                        Commands.sequence(
                                autoActions.homeEverything(),
                                autoActions.AutoAimShoot(L4, 'L'),
                                autoActions.followPath(getAutoPath("L-I2"), true, true, false)
                        ),
                        autoActions.followPath(getAutoPath("I1-I2"), true, true, false).until(autoActions::isIntakeFinished),
                        autoActions::isIntakeFinished
                ),
                autoActions.waitFor(0.5).until(autoActions::isIntakeFinished),
                Commands.either(
                        Commands.sequence(
                                autoActions.homeEverything(),
                                autoActions.AutoAimShoot(L4, 'B'),
                                autoActions.followPath(getAutoPath("B-I3"), true, true, false)
                        ),
                        //autoActions.waitFor(3),
                        autoActions.followPath(getAutoPath("I2-I3"), true, true, false).until(autoActions::isIntakeFinished),
                        autoActions::isIntakeFinished
                ),
                autoActions.AutoAimShoot(L4, 'D')
        );
    }

    private Command build4CoralRight() {
        return new SequentialCommandGroup(
                autoActions.AutoAimShoot(L4, 'F'),
                autoActions.followPath(getAutoPath("EF-I3"), true, true, false),
                autoActions.waitFor(0.5).until(autoActions::isIntakeFinished),
                Commands.either(
                        Commands.sequence(
                                autoActions.homeEverything(),
                                autoActions.AutoAimShoot(L4, 'C'),
                                autoActions.followPath(getAutoPath("C-I2"), true, true, false)
                        ),
                        autoActions.followPath(getAutoPath("I3-I2"), true, true, false).until(autoActions::isIntakeFinished),
                        autoActions::isIntakeFinished
                ),
                autoActions.waitFor(0.5).until(autoActions::isIntakeFinished),
                Commands.either(
                        Commands.sequence(
                                autoActions.homeEverything(),
                                autoActions.AutoAimShoot(L4, 'A'),
                                autoActions.followPath(getAutoPath("A-I1"), true, true, false)
                        ),
                        //autoActions.waitFor(3),
                        autoActions.followPath(getAutoPath("I2-I1"), true, true, false).until(autoActions::isIntakeFinished),
                        autoActions::isIntakeFinished
                ),
                autoActions.AutoAimShoot(L4, 'K')
        );
    }
}
