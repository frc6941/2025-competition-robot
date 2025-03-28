package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import org.littletonrobotics.junction.Logger;

public class AutonomousShootCoralCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final Timer timer;
    private boolean isShootFinished = false;

    public AutonomousShootCoralCommand(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        Logger.recordOutput("Commands/shoot", "execute");
        if(elevatorSubsystem.getIo().isNearExtension(DestinationSupplier.getInstance().getElevatorSetpoint(true))){
            endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.SHOOT);
        }
//        if (endEffectorSubsystem.isShootFinished()) {
//            timer.reset();
//            isShootFinished = true;
//        }
    }

    @Override
    public boolean isFinished(){
        //FIXME: only for sim
        return endEffectorSubsystem.isShootFinished();
        //return false;
    }

    @Override
    public void end(boolean interrupted){
        Logger.recordOutput("Commands/shoot", "end");
        endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.IDLE);
    }
}
