package frc.robot.subsystems.limelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.RobotConstants;
import frc.robot.subsystems.swerve.Swerve;
import lombok.extern.java.Log;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import static frc.robot.RobotConstants.LimelightConstants.*;

public class Limelight extends SubsystemBase {
    PoseEstimate lastEstimateRight = new PoseEstimate();
    PoseEstimate lastEstimateLeft = new PoseEstimate();

    boolean newRightEstimate = false;
    boolean newLeftEstimate = false;

    Pose2d rightPose = new Pose2d();
    Pose2d leftPose = new Pose2d();

    private boolean useMegaTag2 = false;

    private boolean rejectEstimate = false;

    public Limelight() {
    }

    public PoseEstimate[] getLastPoseEstimates() {
        return new PoseEstimate[] {lastEstimateRight, lastEstimateLeft};
    }

    public void setMegaTag2(boolean useMegaTag2) {
        this.useMegaTag2 = useMegaTag2;
    }

    /**
     * Determines if a given pose estimate should be rejected.
     *
     *
     * @param poseEstimate The pose estimate to check
     * @param gyroRate     The current rate of rotation observed by our gyro.
     *
     * @return True if the estimate should be rejected
     */

    public boolean rejectUpdate(PoseEstimate poseEstimate, AngularVelocity gyroRate) {
        // Angular velocity is too high to have accurate vision
        if (gyroRate.compareTo(RobotConstants.SwerveConstants.maxAngularRate) > 0) {
            rejectEstimate = true;
            return true;
        }
        //TODO: verify this condition whether usable

        // No tags :<
        if (poseEstimate.tagCount == 0) {
            rejectEstimate = true;
            return true;
        }

        // 1 Tag with a large area
        if (poseEstimate.tagCount == 1 && poseEstimate.avgTagArea > AREA_THRESHOLD) {
            rejectEstimate = false;
            return false;
            // 2 tags or more
        } else if (poseEstimate.tagCount > 1) {
            rejectEstimate = false;
            return false;
        }

        rejectEstimate = true;
        return true;
    }

    /**
     * Updates the current pose estimates for the left and right of the robot using
     * data from Limelight cameras.
     *
     * @param gyroRate The current angular velocity of the robot, used to validate
     *                 the pose estimates.
     *
     *                 This method retrieves pose estimates from two Limelight
     *                 cameras (left and right) and updates the
     *                 corresponding pose estimates if they are valid. The method
     *                 supports two modes of operation:
     *                 one using MegaTag2 and one without. The appropriate pose
     *                 estimate retrieval method is chosen
     *                 based on the value of the `useMegaTag2` flag.
     *
     *                 If the retrieved pose estimates are valid and not rejected
     *                 based on the current angular velocity,
     *                 the method updates the last known estimates and sets flags
     *                 indicating new estimates are available.
     */
    public void setCurrentEstimates(AngularVelocity gyroRate) {
        PoseEstimate currentEstimateRight = new PoseEstimate();
        PoseEstimate currentEstimateLeft = new PoseEstimate();

        if (useMegaTag2) {
            currentEstimateRight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_RIGHT);
            currentEstimateLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_LEFT);
        } else {
            currentEstimateRight = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_RIGHT);
            currentEstimateLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_LEFT);
        }

        if (currentEstimateRight != null && !rejectUpdate(currentEstimateRight, gyroRate)) {
            lastEstimateRight = currentEstimateRight;
            rightPose = currentEstimateRight.pose;
            newRightEstimate = true;
        }
        if (currentEstimateLeft != null && !rejectUpdate(currentEstimateLeft, gyroRate)) {
            lastEstimateLeft = currentEstimateLeft;
            leftPose = currentEstimateLeft.pose;
            newLeftEstimate = true;
        }
    }

    public Optional<PoseEstimate[]> determinePoseEstimate(AngularVelocity gyroRate) {
        setCurrentEstimates(gyroRate);

        // No valid pose estimates :(
        if (!newRightEstimate && !newLeftEstimate) {
            return Optional.empty();

        } else if (newRightEstimate && !newLeftEstimate) {
            // One valid pose estimate (right)
            newRightEstimate = false;
            return Optional.of(new PoseEstimate[]{lastEstimateRight, null});

        } else if (!newRightEstimate && newLeftEstimate) {
            // One valid pose estimate (left)
            newLeftEstimate = false;
            return Optional.of(new PoseEstimate[]{lastEstimateLeft, null});

        } else {
            // Two valid pose estimates, disgard the one that's further
            newRightEstimate = false;
            newLeftEstimate = false;
            return Optional.of(new PoseEstimate[]{lastEstimateRight, lastEstimateLeft});
        }
    }

    private void addVisionMeasurement(){
        LimelightHelpers.SetRobotOrientation(LIMELIGHT_LEFT,
                Swerve.getInstance().getLocalizer().getLatestPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(LIMELIGHT_RIGHT,
                Swerve.getInstance().getLocalizer().getLatestPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        AngularVelocity gyroRate = Units.DegreesPerSecond.of(Swerve.getInstance().getLocalizer().getSmoothedVelocity().getRotation().getDegrees());

        Optional<PoseEstimate[]> estimatedPose = determinePoseEstimate(gyroRate);
        if (estimatedPose.isPresent()) {
            if(estimatedPose.get()[0]!=null){
                if (useMegaTag2) {
                    Swerve.getInstance().getLocalizer().addMeasurement(estimatedPose.get()[0].timestampSeconds, estimatedPose.get()[0].pose, VecBuilder.fill(.7, .7, 9999999));
                } else {
                    Swerve.getInstance().getLocalizer().addMeasurement(estimatedPose.get()[0].timestampSeconds, estimatedPose.get()[0].pose, VecBuilder.fill(.5, .5, 9999999));
                }
                Logger.recordOutput("LimelightL/estimatedPose",estimatedPose.get()[0].pose);
            }
            Logger.recordOutput("LimelightL/hasEstimate", estimatedPose.get()[0]!=null);
            if(estimatedPose.get()[1] != null){
                if (useMegaTag2) {
                    Swerve.getInstance().getLocalizer().addMeasurement(estimatedPose.get()[1].timestampSeconds, estimatedPose.get()[1].pose, VecBuilder.fill(.7, .7, 9999999));
                } else {
                    Swerve.getInstance().getLocalizer().addMeasurement(estimatedPose.get()[1].timestampSeconds, estimatedPose.get()[1].pose, VecBuilder.fill(.5, .5, 9999999));
                }
                Logger.recordOutput("LimelightR/estimatedPose",estimatedPose.get()[1].pose);
            }
            Logger.recordOutput("LimelightR/hasEstimate", estimatedPose.get()[1]!=null);

        } else {
            Logger.recordOutput("LimelightL/hasEstimate",false);
            Logger.recordOutput("LimelightR/hasEstimate",false);
        }
    }

    @Override
    public void periodic() {
        addVisionMeasurement();
        Logger.recordOutput("Limelight/rejectEstimate", rejectEstimate);
    }
}

