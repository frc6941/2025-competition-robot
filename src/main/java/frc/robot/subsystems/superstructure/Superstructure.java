package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.*;

public class Superstructure extends SubsystemBase {
    private final ElevatorSubsystem elevator;
    private final EndEffectorArmSubsystem endEffectorArm;
    private final IntakeSubsystem intake;

    private final Graph<SuperstructureState, EdgeCommand> graph = new DefaultDirectedGraph<>(EdgeCommand.class);

    @Getter private SuperstructureState state = SuperstructureState.START;
    private SuperstructureState next = null;
    @Getter private SuperstructureState goal = SuperstructureState.START;
    private EdgeCommand edgeCommand;

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
                .command(Commands.none())   
                .build());
    }

    @Override
    public void periodic() {
        // Run periodic
        elevator.periodic();
        endEffectorArm.periodic();
        intake.periodic();

        if (edgeCommand == null || !edgeCommand.getCommand().isScheduled()) {
            // Update edge to new state
            if (next != null) {
                state = next;
                next = null;
            }

            // Schedule next command in sequence
            if (state != goal) {
                bfs(state, goal)
                    .ifPresent(next -> {
                        this.next = next;
                        edgeCommand = graph.getEdge(state, next);
                        edgeCommand.getCommand().schedule();
                    });
            }
        }

        // Log state
        Logger.recordOutput("Superstructure/State", state);
        Logger.recordOutput("Superstructure/Next", next);
        Logger.recordOutput("Superstructure/Goal", goal);
        if (edgeCommand != null) {
            Logger.recordOutput(
                "Superstructure/EdgeCommand",
                graph.getEdgeSource(edgeCommand) + " --> " + graph.getEdgeTarget(edgeCommand));
        } else {
            Logger.recordOutput("Superstructure/EdgeCommand", "");
        }
    }

    public void setGoal(SuperstructureState goal) {
        // Don't do anything if goal is the same
        if (this.goal == goal) return;
        this.goal = goal;

        if (next == null) return;

        var edgeToCurrentState = graph.getEdge(next, state);
        // Figure out if we should schedule a different command to get to goal faster
        if (edgeCommand.getCommand().isScheduled()
            && edgeToCurrentState != null
            && isEdgeAllowed(edgeToCurrentState, goal)) {
            // Figure out where we would have gone from the previous state
            bfs(state, goal)
                .ifPresent(newNext -> {
                    if (newNext == next) {
                        // We are already on track
                        return;
                    }

                    if (newNext != state && graph.getEdge(next, newNext) != null) {
                        // We can skip directly to the newNext edge
                        edgeCommand.getCommand().cancel();
                        edgeCommand = graph.getEdge(state, newNext);
                        edgeCommand.getCommand().schedule();
                        next = newNext;
                    } else {
                        // Follow the reverse edge from next back to the current edge
                        edgeCommand.getCommand().cancel();
                        edgeCommand = graph.getEdge(next, state);
                        edgeCommand.getCommand().schedule();
                        var temp = state;
                        state = next;
                        next = temp;
                    }
                });
        }
    }

    private Optional<SuperstructureState> bfs(SuperstructureState start, SuperstructureState goal) {
        // Map to track the parent of each visited node
        Map<SuperstructureState, SuperstructureState> parents = new HashMap<>();
        Queue<SuperstructureState> queue = new LinkedList<>();
        queue.add(start);
        parents.put(start, null); // Mark the start node as visited with no parent

        // Perform BFS
        while (!queue.isEmpty()) {
            SuperstructureState current = queue.poll();
            // Check if we've reached the goal
            if (current == goal) {
                break;
            }
            // Process valid neighbors
            for (EdgeCommand edge : graph.outgoingEdgesOf(current)) {
                SuperstructureState neighbor = graph.getEdgeTarget(edge);
                // Only process unvisited neighbors
                if (!parents.containsKey(neighbor)) {
                    parents.put(neighbor, current);
                    queue.add(neighbor);
                }
            }
        }

        // Reconstruct the path to the goal if found
        if (!parents.containsKey(goal)) {
            return Optional.empty(); // Goal not reachable
        }

        // Trace back the path from goal to start
        SuperstructureState nextState = goal;
        while (!nextState.equals(start)) {
            SuperstructureState parent = parents.get(nextState);
            if (parent == null) {
                return Optional.empty(); // No valid path found
            } else if (parent.equals(start)) {
                // Return the edge from start to the next node
                return Optional.of(nextState);
            }
            nextState = parent;
        }
        return Optional.of(nextState);
    }

    private boolean isEdgeAllowed(EdgeCommand edge, SuperstructureState goal) {
        return true; // Add any edge restrictions here
    }

    /** All edge commands should finish and exit properly. */
    @lombok.Builder(toBuilder = true)
    @lombok.Getter
    public static class EdgeCommand extends DefaultEdge {
        private final Command command;
    }
} 