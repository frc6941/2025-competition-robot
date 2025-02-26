package frc.robot.auto.basics;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.List;

import static com.pathplanner.lib.auto.AutoBuilder.getAllAutoNames;

// adapted from AutoBuilder since we do not initialize AutoBuilder.
public class CustomAutoChooser {
    // FIXME: choose auto based on AutoFile
    public static SendableChooser<String> buildAutoChooser() {
        return buildAutoChooser("");
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
}
