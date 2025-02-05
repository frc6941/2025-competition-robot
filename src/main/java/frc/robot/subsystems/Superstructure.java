package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;



/*
public class Superstructure {
        public static enum SuperState {
                IDLE, READY_CORAL, PRE_L1, PRE_L2, PRE_L3, PRE_L4, SCORE_CORAL, SPIT_CORAL
        }

        private final Trigger preScoreL1Req;
        private final Trigger preScoreL2Req;
        private final Trigger preScoreL3Req;
        private final Trigger preScoreL4Req;
        private final Trigger scoreReq;

        private SuperState state = SuperState.IDLE;
        private SuperState prevState = SuperState.IDLE;
        private Map<SuperState, Trigger> stateTriggers = new HashMap<SuperState, Trigger>();

        private Timer stateTimer = new Timer();

        private final ElevatorSubsystem elevator;
        private final EndEffectorSubsystem endEffector;

        private double L1 = L1_EXTENSION_METERS.get();
        private double L2 = L2_EXTENSION_METERS.get();
        private double L3 = L3_EXTENSION_METERS.get();
        private double L4 = L4_EXTENSION_METERS.get();
        private double idleHeight = IDLE_EXTENSION_METERS.get();
        private double maxHeight = MAX_EXTENSION_METERS.get();

        public Superstructure(
                        ElevatorSubsystem elevator,
                        EndEffectorSubsystem endEffector,
                        Trigger preScoreL1Req,
                        Trigger preScoreL2Req, 
                        Trigger preScoreL3Req, 
                        Trigger preScoreL4Req, 
                        Trigger scoreReq) {

                this.elevator = elevator;
                this.endEffector = endEffector;

                this.scoreReq = scoreReq;
                this.preScoreL1Req = preScoreL1Req;
                this.preScoreL2Req = preScoreL2Req;
                this.preScoreL3Req = preScoreL3Req;
                this.preScoreL4Req = preScoreL4Req;

                stateTimer.start();

                for (var state : SuperState.values()) {
                        stateTriggers.put(state, new Trigger(() -> this.state == state));
                }

                configureStateTransitionCommands();
        }

        public void periodic() {
                Logger.recordOutput("Superstructure State", state);
        }

        private void configureStateTransitionCommands() {
                // IDLE -> READY_CORAL
                stateTriggers.get(SuperState.IDLE)
                        .whileTrue(endEffector.index())
                        .whileTrue(elevator.setExtension(idleHeight))
                        .and(() -> endEffector.getThirdBeambreak() && !endEffector.getSecondBeambreak())
                        .onTrue(this.forceState(SuperState.READY_CORAL));

                // READY_CORAL
                stateTriggers.get(SuperState.READY_CORAL)
                        .whileTrue(endEffector.hold())
                        .whileTrue(elevator.setExtension(idleHeight));

                // READY_CORAL -> PRE_L{1-4}
                stateTriggers.get(SuperState.READY_CORAL)
                        .and(preScoreL1Req)
                        .whileTrue(endEffector.hold())
                        .whileTrue(elevator.setExtension(L1))
                        .and(() -> elevator.isNearExtension(L1))
                        .onTrue(this.forceState(SuperState.PRE_L1));

                stateTriggers.get(SuperState.READY_CORAL)
                        .and(preScoreL2Req)
                        .whileTrue(endEffector.hold())
                        .whileTrue(elevator.setExtension(L2))
                        .and(() -> elevator.isNearExtension(L2))
                        .onTrue(this.forceState(SuperState.PRE_L2));
                        
                stateTriggers.get(SuperState.READY_CORAL)
                        .and(preScoreL3Req)
                        .whileTrue(endEffector.hold())
                        .whileTrue(elevator.setExtension(L3))
                        .and(() -> elevator.isNearExtension(L3))
                        .onTrue(this.forceState(SuperState.PRE_L3));
                                
                stateTriggers.get(SuperState.READY_CORAL)
                        .and(preScoreL4Req)
                        .whileTrue(endEffector.hold())
                        .whileTrue(elevator.setExtension(L4))
                        .and(() -> elevator.isNearExtension(L4))
                        .onTrue(this.forceState(SuperState.PRE_L4));

                // PRE_L{1-4} logic + -> SCORE_CORAL
                stateTriggers.get(SuperState.PRE_L1)
                        .whileTrue(elevator.setExtension(L1))
                        .whileTrue(endEffector.hold())
                        .and(() -> elevator.isNearExtension(L1))
                        .and(scoreReq)
                        .onTrue(this.forceState(SuperState.SCORE_CORAL));

                stateTriggers.get(SuperState.PRE_L2)
                        .whileTrue(elevator.setExtension(L2))
                        .whileTrue(endEffector.hold())
                        .and(() -> elevator.isNearExtension(L2))
                        .and(scoreReq)
                        .onTrue(this.forceState(SuperState.SCORE_CORAL));

                stateTriggers.get(SuperState.PRE_L3)
                        .whileTrue(elevator.setExtension(L3))
                        .whileTrue(endEffector.hold())
                        .and(() -> elevator.isNearExtension(L3))
                        .and(scoreReq)
                        .onTrue(this.forceState(SuperState.SCORE_CORAL));
                        
                stateTriggers.get(SuperState.PRE_L1)
                        .whileTrue(elevator.setExtension(L4))
                        .whileTrue(endEffector.hold())
                        .and(() -> elevator.isNearExtension(L4))
                        .and(scoreReq)
                        .onTrue(this.forceState(SuperState.SCORE_CORAL));

                // SCORE_CORAL -> IDLE
                stateTriggers.get(SuperState.SCORE_CORAL)
                        .whileTrue(endEffector.shoot())
                        .and(() -> !endEffector.getThirdBeambreak())
                        .onTrue(this.forceState(SuperState.IDLE));
        }

        public SuperState getState() {
                return state;
        }

        private Command forceState(SuperState nextState) {
                return Commands.runOnce(() -> {
                        this.prevState = this.state;
                        this.state = nextState;
                });
        }
}
*/