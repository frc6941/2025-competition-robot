package frc.robot.subsystems.limelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.limelight.LimelightIO.PoseEstimate;
import frc.robot.subsystems.swerve.Swerve;
import org.frcteam6941.localization.Localizer;
import org.littletonrobotics.junction.Logger;

import java.util.*;

import static frc.robot.RobotConstants.LimelightConstants.*;

public class LimelightSubsystem extends SubsystemBase {
    private final Map<String, LimelightIO> limelightIOs;
    private final Map<String, LimelightIOInputsAutoLogged> limelightInputs;

    private final Localizer swerveLocalizer = Swerve.getInstance().getLocalizer();
    private boolean useMegaTag2 = false;

    public LimelightSubsystem(Map<String, LimelightIO> limelightIOs) {
        this.limelightIOs = limelightIOs;

        limelightInputs = new HashMap<>();
        this.limelightIOs.forEach((key, value) -> limelightInputs.put(key, new LimelightIOInputsAutoLogged()));
    }

    public static boolean rejectUpdate(PoseEstimate poseEstimate, AngularVelocity gyroRate) {
        if (poseEstimate == null) {
            return true;
        }

        // Angular velocity is too high to have accurate vision
        if (gyroRate.compareTo(RobotConstants.SwerveConstants.maxAngularRate) > 0) {
            return true;
        }
        //TODO: verify this condition whether usable

        // No tags :<
        if (poseEstimate.tagCount() == 0) {
            return true;
        }

        // 1 Tag with a large area
        if (poseEstimate.tagCount() == 1 && poseEstimate.avgTagArea() > AREA_THRESHOLD) {
            // TODO: BUG: area threshold is wayyyy to small the tag area is 0-100% of original tag
            return false;
            // 2 tags or more
        } else return poseEstimate.tagCount() <= 1;
    }

    public PoseEstimate[] getLastPoseEstimates() {
        List<PoseEstimate> poseEstimates = new ArrayList<>();
        limelightInputs.forEach((key, input) -> {
            poseEstimates.add(input.poseBlue);
        });
        return poseEstimates.toArray(new PoseEstimate[0]);
    }

    public void setMegaTag2(boolean value) {
        this.useMegaTag2 = value;
        limelightIOs.forEach((key, io) -> {
            io.setMegaTag2(useMegaTag2);
        });
    }

    public Optional<PoseEstimate[]> determinePoseEstimate(AngularVelocity gyroRate) {
        limelightIOs.forEach((key, io) -> io.setNewEstimate(limelightInputs.get(key), !rejectUpdate(limelightInputs.get(key).poseBlue, gyroRate)));

        boolean newRightEstimate = limelightInputs.get(LIMELIGHT_RIGHT).newEstimate;
        boolean newLeftEstimate = limelightInputs.get(LIMELIGHT_LEFT).newEstimate;
        PoseEstimate lastEstimateRight = limelightInputs.get(LIMELIGHT_RIGHT).poseBlue;
        PoseEstimate lastEstimateLeft = limelightInputs.get(LIMELIGHT_LEFT).poseBlue;
        // No valid pose estimates :(
        if (!newRightEstimate && !newLeftEstimate) {
            return Optional.empty();

        } else if (newRightEstimate && !newLeftEstimate) {
            // One valid pose estimate (right)
            limelightIOs.get(LIMELIGHT_RIGHT).setNewEstimate(limelightInputs.get(LIMELIGHT_RIGHT), false);
            return Optional.of(new PoseEstimate[]{lastEstimateRight, null});

        } else if (!newRightEstimate) {
            // One valid pose estimate (left)
            limelightIOs.get(LIMELIGHT_LEFT).setNewEstimate(limelightInputs.get(LIMELIGHT_LEFT), false);
            return Optional.of(new PoseEstimate[]{lastEstimateLeft, null});

        } else {
            // Two valid pose estimates, disgard the one that's further
            limelightIOs.get(LIMELIGHT_RIGHT).setNewEstimate(limelightInputs.get(LIMELIGHT_RIGHT), false);
            limelightIOs.get(LIMELIGHT_LEFT).setNewEstimate(limelightInputs.get(LIMELIGHT_LEFT), false);
            return Optional.of(new PoseEstimate[]{lastEstimateRight, lastEstimateLeft});
        }
    }

    private void addVisionMeasurement() {
        limelightIOs.forEach((name, io) -> {
            io.setRobotOrientation(swerveLocalizer.getLatestPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        });

        AngularVelocity gyroRate = Units.DegreesPerSecond.of(swerveLocalizer.getSmoothedVelocity().getRotation().getDegrees());

        limelightIOs.forEach((name, io) -> {
            io.updateInputs(limelightInputs.get(name), gyroRate);
            Logger.processInputs(name, limelightInputs.get(name));
        });

        Optional<PoseEstimate[]> estimatedPose = determinePoseEstimate(gyroRate);

        if (estimatedPose.isPresent()) {
            if (estimatedPose.get()[0] != null) {
                if (useMegaTag2) {
                    swerveLocalizer.addMeasurement(estimatedPose.get()[0].timestampSeconds(), estimatedPose.get()[0].pose(), VecBuilder.fill(.7, .7, 9999999));
                } else {
                    swerveLocalizer.addMeasurement(estimatedPose.get()[0].timestampSeconds(), estimatedPose.get()[0].pose(), VecBuilder.fill(.5, .5, 9999999));
                }
                Logger.recordOutput(LIMELIGHT_LEFT + "/estimatedPose", estimatedPose.get()[0].pose());
            }
            if (estimatedPose.get()[1] != null) {
                if (useMegaTag2) {
                    swerveLocalizer.addMeasurement(estimatedPose.get()[1].timestampSeconds(), estimatedPose.get()[1].pose(), VecBuilder.fill(.7, .7, 9999999));
                } else {
                    swerveLocalizer.addMeasurement(estimatedPose.get()[1].timestampSeconds(), estimatedPose.get()[1].pose(), VecBuilder.fill(.5, .5, 9999999));
                }
                Logger.recordOutput(LIMELIGHT_RIGHT + "/estimatedPose", estimatedPose.get()[1].pose());
            }
        }
    }

    @Override
    public void periodic() {
        addVisionMeasurement();
    }
}

