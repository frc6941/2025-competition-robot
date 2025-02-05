package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void setPosition(double heightMeters) {
        io.setPosition(heightMeters);
        Logger.recordOutput("Elevator/Setpoint", heightMeters);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public double getPositionMeters() {
        return inputs.positionMeters;
    }

    public boolean isAtSetpoint(double setpoint) {
        return MathUtil.isNear(setpoint, inputs.positionMeters, DEADZONE_DISTANCE);
    }
}
