package frc.robot.auto.basics;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.auto.modes.AutoFile;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;

import static com.pathplanner.lib.auto.AutoBuilder.getAllAutoNames;


public class CustomAutoChooser {

    private AutoFile autoFile;
    private AutoActions autoActions;
    private static LoggedDashboardChooser<String> chooser;

    public CustomAutoChooser(RobotContainer container) {
        autoActions = new AutoActions(container);
        autoFile = new AutoFile(autoActions);
        chooser= new LoggedDashboardChooser<>("Chooser", CustomAutoChooser.buildAutoChooser("New Auto"));

    }

    public static SendableChooser<String> buildAutoChooser(String defaultAutoName) {
        SendableChooser<String> chooser = new SendableChooser<>();

        List<String> autoNames = getAllAutoNames();
        String defaultOption = null;

        for (String autoName : autoNames) {
            if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
                defaultOption = autoName;
            }
            chooser.addOption(autoName, autoName);
        }

        if (defaultOption == null) {
            chooser.setDefaultOption("None", "");
        } else {
            chooser.setDefaultOption(defaultOption, defaultOption);
            chooser.addOption("None", "");
        }

        return chooser;
    }

    public Command getAuto(){
        return autoFile.getAuto(chooser.get());
    }
}
