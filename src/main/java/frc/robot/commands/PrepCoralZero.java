package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.statemachine.StateMachine;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepCoralZero extends Command {
  /** Creates a new CoralPrep0. */
  StateMachine stateMachine;
  ElevatorSubsystem elevatorSubsystem;


  public PrepCoralZero(StateMachine stateMachine, ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.stateMachine = stateMachine;
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(stateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (stateMachine == null || elevatorSubsystem == null) {
      throw new IllegalStateException("State machine or subsystems are not initialized");
    }
    stateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_ZERO);
    elevatorSubsystem.setPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtSetpoint(0);
  }
}
