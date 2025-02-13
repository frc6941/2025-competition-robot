package frc.robot.subsystems.roller;

import static frc.robot.RobotConstants.LOOPER_DT;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

//TODO incomplete sim
public class RollerIOSim implements RollerIO {
    private final DCMotorSim motorSim;


    public RollerIOSim(
            double jKgMetersSquared,
            double gearRatio) {
        this.motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60Foc(1), jKgMetersSquared, gearRatio),
                DCMotor.getKrakenX60Foc(1));
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        motorSim.update(LOOPER_DT);
        inputs.velocityRotPerSec = motorSim.getAngularVelocityRPM() / 60;
        inputs.appliedVolts = motorSim.getInputVoltage();
        inputs.statorCurrentAmps = motorSim.getCurrentDrawAmps();
        inputs.supplyCurrentAmps = 0.0;
        inputs.tempCelsius = 0.0;
    }

    @Override
    public void setVoltage(double voltage) {
        motorSim.setInputVoltage(voltage);
    }

    @Override
    public void setVelocity(double rads) {
        motorSim.setAngularVelocity(rads);
    }

}
