package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.*;
import edu.wpi.first.math.MathUtil;
import frc.robot.RobotConstants;


public class ElevatorZeroingCommand extends Command {
    private ElevatorSubsystem elevator;

    public ElevatorZeroingCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute(){
        elevator.setVoltage(-0.5);
    }

    @Override
    public boolean isFinished() {
        return elevator.getLeaderCurrent() > RobotConstants.ElevatorConstants.ELEVATOR_ZEROING_CURRENT.get();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setVoltage(0);
        elevator.resetPosition();
    }


}
