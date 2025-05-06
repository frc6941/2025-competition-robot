package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    private final ElevatorSubsystem elevator;
    private final EndEffectorArmSubsystem endEffectorArm;
    private final IntakeSubsystem intake;

    private final Graph<SuperstructureState, EdgeCommand> graph = new DefaultDirectedGraph<>(EdgeCommand.class);

    @Getter private SuperstructureState state = SuperstructureState.START;
    private SuperstructureState next = null;
    @Getter private SuperstructureState goal = SuperstructureState.START;

    public Superstructure(
            ElevatorSubsystem elevator,
            EndEffectorArmSubsystem endEffectorArm,
            IntakeSubsystem intake) {
        this.elevator = elevator;
        this.endEffectorArm = endEffectorArm;
        this.intake = intake;

        // Add states as vertices
        for (var state : SuperstructureState.values()) {
            graph.addVertex(state);
        }

        // Add edge from start to idle
        graph.addEdge(
            SuperstructureState.START,
            SuperstructureState.STOW,
            EdgeCommand.builder()
                .command(() -> {})
                .build());
    
    }

    @Override
    public void periodic() {
        // Run periodic
        elevator.periodic();
        endEffectorArm.periodic();
        intake.periodic();

        // Log state
        Logger.recordOutput("Superstructure/State", state);
        Logger.recordOutput("Superstructure/Next", next);
        Logger.recordOutput("Superstructure/Goal", goal);
    }

    public void setGoal(SuperstructureState goal) {
        // Don't do anything if goal is the same
        if (this.goal == goal) return;
        this.goal = goal;
    }

    /** All edge commands should finish and exit properly. */
    @lombok.Builder(toBuilder = true)
    @lombok.Getter
    public static class EdgeCommand extends DefaultEdge {
        private final Runnable command;
    }
} 