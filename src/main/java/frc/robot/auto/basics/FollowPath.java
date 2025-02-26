package frc.robot.auto.basics;

import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.AllianceFlipUtil;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Comparator;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.Queue;

import static frc.robot.RobotConstants.SwerveConstants.AUTO_SWERVE_TOLERANCE_SECS;

// adapted from FollowPathCommand in pathplanner
public class FollowPath extends Command {
    private final AutoActions autoActions;
    private final Swerve swerve;
    private final Timer timer = new Timer();
    private final boolean angleLock;
    private final boolean requiredOnTarget;
    private final boolean resetOdometry;
    private final Queue<Event> events;
    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    private FollowTrajectory followTrajectoryCommand;

    public FollowPath(
            AutoActions autoActions,
            Swerve swerve, PathPlannerPath path, boolean angleLock, boolean requiredOnTarget, boolean resetOdometry) {
        this.swerve = swerve;
        this.path = path;
        this.angleLock = angleLock;
        this.requiredOnTarget = requiredOnTarget;
        this.resetOdometry = resetOdometry;
        this.events =
                new PriorityQueue<>(Comparator.comparingDouble(Event::getTimestampSeconds));
        this.autoActions = autoActions;
    }

    private void initializeTrajectory() {
        Pose2d currentPose = this.swerve.getLocalizer().getCoarseFieldPose(0);
        ChassisSpeeds currentSpeeds = this.swerve.getChassisSpeeds();
        double linearVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        if (this.path.getIdealStartingState() != null) {
            boolean idealVelocity = Math.abs(linearVel - this.path.getIdealStartingState().velocityMPS()) <= (double) 0.25F;
            boolean idealRotation = !this.swerve.getAutoConfig().isHolonomic ||
                    Math.abs(currentPose.getRotation().minus(this.path.getIdealStartingState().rotation()).getDegrees()) <= (double) 30.0F;
            if (idealVelocity && idealRotation) {
                this.trajectory = this.path.getIdealTrajectory(this.swerve.getAutoConfig()).orElseThrow();
            } else {
                this.trajectory = this.path.generateTrajectory(currentSpeeds, currentPose.getRotation(), this.swerve.getAutoConfig());
            }
        } else {
            this.trajectory = this.path.generateTrajectory(currentSpeeds, currentPose.getRotation(), this.swerve.getAutoConfig());
        }
    }

    @Override
    public void initialize() {
        if (AllianceFlipUtil.shouldFlip() && !this.path.preventFlipping) {
            this.path = this.path.flipPath();
        }

        this.trajectory = this.path.getIdealTrajectory(this.swerve.getAutoConfig()).orElseThrow();

        if (resetOdometry) {
            swerve.resetPose(trajectory.getInitialPose());
        }

        initializeTrajectory();

        followTrajectoryCommand = new FollowTrajectory(this.swerve, this.trajectory, this.angleLock, this.requiredOnTarget);
        followTrajectoryCommand.initialize();

        events.clear();
        events.addAll(trajectory.getEvents());
        this.timer.reset();
        this.timer.start();
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();
        while (!events.isEmpty() && currentTime >= events.peek().getTimestampSeconds()) {
            try {
                Event event = Objects.requireNonNull(events.poll());
                if (!event.getClass().getName().equals("com.pathplanner.lib.events.OneShotTriggerEvent")) {
                    System.out.println("No matching event handler found for event " + event.getClass().getName());
                    continue;
                }
                Method cmd = event.getClass().getMethod("getEventName");
                autoActions.invokeCommand((String) cmd.invoke(event), () -> followTrajectoryCommand.isFinished());
            } catch (NoSuchMethodException | InvocationTargetException | IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return followTrajectoryCommand.isFinished() && this.timer.get() >= AUTO_SWERVE_TOLERANCE_SECS;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopMovement();
        this.timer.stop();
        this.events.clear();
    }
}
