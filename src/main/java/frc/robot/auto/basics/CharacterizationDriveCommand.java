package frc.robot.auto.basics;
 
import com.team254.lib.util.PolynomialRegression;
 
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
 
import frc.robot.RobotConstants;
import frc.robot.subsystems.swerve.Swerve;
 
import java.util.ArrayList;
 
// Command to characterize the drivetrain by applying increasing voltage and recording the resulting velocities
public class CharacterizationDriveCommand extends Command {
    private final double maxVoltage;
    private final Timer prepTimer = new Timer();
    private final Timer timer = new Timer();
    private final ArrayList<Double> yVoltages = new ArrayList<>();
    private final ArrayList<Double> xVelocities = new ArrayList<>();
    private final ArrayList<Double> xKrakenVelocities = new ArrayList<>();
    private Swerve drivetrain;
    private double startVoltage;
    private double deltaVoltage;
    private final Runnable r = () -> {
        double prepTime = 3.0;
        if (prepTimer.get() < prepTime) {
            timer.stop();
            SwerveModuleState individualState = new SwerveModuleState(
                    0.0, new Rotation2d());
            drivetrain.setModuleStates(new SwerveModuleState[]{
                    individualState, individualState, individualState, individualState
            }, true, true);
            return;
        }
 
        timer.start();
        double targetVoltage = startVoltage + deltaVoltage * timer.get();
 
        SwerveModuleState individualState = new SwerveModuleState(
                targetVoltage, new Rotation2d()
        );
        System.out.println("REQ = " + individualState);
        drivetrain.setModuleStates(new SwerveModuleState[]{
                individualState,
                individualState,
                individualState,
                individualState
        }, true, true);
        yVoltages.add(targetVoltage);
 
        SwerveModuleState[] moduleStates = drivetrain.getModuleStates();
        double averageVelocity = 0.0;
        double averageKrakenVelocity = 0.0;
        for (SwerveModuleState state : moduleStates) {
            averageVelocity += Math.abs(state.speedMetersPerSecond);
            averageKrakenVelocity += Math.abs(
                    (state.speedMetersPerSecond * 60)
                            / RobotConstants.SwerveConstants.wheelCircumferenceMeters.magnitude()
                            * RobotConstants.SwerveConstants.DRIVE_GEAR_RATIO * (2048.0 / 600.0));
        }
        averageVelocity /= moduleStates.length;
        averageKrakenVelocity /= moduleStates.length;
        xVelocities.add(averageVelocity);
        xKrakenVelocities.add(averageKrakenVelocity);
 
    };
    private final Notifier n = new Notifier(r);
 
    // Constructor to initialize the command with the drivetrain, start voltage, delta voltage, and maximum voltage
    public CharacterizationDriveCommand(Swerve drivetrain, double startVoltage, double deltaVoltage, double maxVoltage) {
        this.drivetrain = drivetrain;
        this.startVoltage = startVoltage;
        this.deltaVoltage = deltaVoltage;
        this.maxVoltage = maxVoltage;
    }
 
    // Initialize the command by resetting timers, clearing drivetrain data, and starting the preparation timer
    @Override
    public void initialize() {
        prepTimer.reset();
        timer.reset();
        drivetrain.empty();
        System.out.println("--- Linear Characterization of the Drivetrain Starts ---");
        prepTimer.start();
        n.startPeriodic(0.01);
    }
 
    // End the command by stopping the notifier, halting drivetrain movement, and resetting timers
    // Also, perform polynomial regression on collected data to determine drivetrain characteristics
    @Override
    public void end(boolean interrupted) {
        n.stop();
        drivetrain.stopMovement();
        drivetrain.normal();
 
        System.out.println("--- Linear Characterization of the Drivetrain Ends ---");
        System.out.println("Total Time Taken: " + timer.get());
        prepTimer.reset();
        timer.stop();
 
        if (xVelocities.isEmpty() || yVoltages.isEmpty() || xKrakenVelocities.isEmpty()) return;
 
        PolynomialRegression regression = new PolynomialRegression(
                xVelocities.stream().mapToDouble(Math::abs).toArray(),
                yVoltages.stream().mapToDouble(Math::abs).toArray(), 1);
        //System.out.println(regression);
        System.out.println("Fit R2: " + regression.R2());
        System.out.println("Drivetrain KS: " + regression.beta(0) + " V");
        System.out.println("Drivetrain kV: " + regression.beta(1) + " V / ms^{-1}");
        System.out.println(
                "Converted Module kV: "
                        + regression.beta(1) / RobotConstants.SwerveConstants.DRIVE_GEAR_RATIO
                        * RobotConstants.SwerveConstants.wheelCircumferenceMeters.magnitude()
                        + " V / rps");
 
        PolynomialRegression regressionKraken = new PolynomialRegression(
                xKrakenVelocities.stream().mapToDouble(Math::abs).toArray(),
                yVoltages.stream().mapToDouble(Math::abs).toArray(), 1);
        System.out.println(
                "Converted Module kV in Kraken Units:"
                        + 1024.0 * regressionKraken.beta(0) + "Kraken Output Units / Kraken Encoder Units / 100ms");
        double travelTicks = 0;
        System.out.println(
                "Travelled Ticks: " + travelTicks
        );
    }
 
    // Determine if the command is finished based on whether the target voltage has reached the maximum voltage
    @Override
    public boolean isFinished() {
        return (startVoltage + deltaVoltage * timer.get()) >= maxVoltage;
    }
}