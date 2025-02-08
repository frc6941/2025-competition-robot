// package frc.robot.subsystems.intake;

// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotConstants;
// import frc.robot.RobotConstants.intakeConstants.intakeGainsClass;
// import frc.robot.RobotConstants.intakeConstants.intakePivotGainsClass;
// import frc.robot.subsystems.beambreak.BeambreakIO;

// import static frc.robot.RobotConstants.EndEffectorConstants.INTAKE_RPS;
// import static frc.robot.RobotConstants.intakeConstants.*;
// import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
// import frc.robot.subsystems.roller.RollerSubsystem;

// public class IntakerSubsystem extends RollerSubsystem{
//     public static final String NAME = "Intaker";
//     private final IntakePivotIO intakerPivotIO;
//     private final IntakerRollerIO intakerRollerIO;
//     private final BeambreakIO intakerBBIO;
//     private BeambreakIOInputsAutoLogged intakeBBIOInputs = new BeambreakIOInputsAutoLogged();
//     private IntakePivotIOInputsAutoLogged intakePivotIOInputs = new IntakePivotIOInputsAutoLogged();

//     private WantedState wantedState = WantedState.IDLE;
//     private SystemState systemState = SystemState.IDLING;
//     private boolean hasTransitionedToFunnel = false;

//     public double roller_kp = intakeGainsClass.INTAKE_KP.get();
//     public double roller_ki = intakeGainsClass.INTAKE_KI.get();
//     public double roller_kd = intakeGainsClass.INTAKE_KD.get();
//     public double roller_ka = intakeGainsClass.INTAKE_KA.get();
//     public double roller_kv = intakeGainsClass.INTAKE_KV.get();
//     public double roller_ks = intakeGainsClass.INTAKE_KS.get();

//     public double pivot_kp = intakePivotGainsClass.INTAKE_PIVOT_KP.get();
//     public double pivot_ki = intakePivotGainsClass.INTAKE_PIVOT_KI.get();
//     public double pivot_kd = intakePivotGainsClass.INTAKE_PIVOT_KD.get();
//     public double pivot_ka = intakePivotGainsClass.INTAKE_PIVOT_KA.get();
//     public double pivot_kv = intakePivotGainsClass.INTAKE_PIVOT_KV.get();
//     public double pivot_ks = intakePivotGainsClass.INTAKE_PIVOT_KS.get();

//     private static double idleRPS = INTAKE_IDLE_RPS.get();
//     private static double intakeVelocity = INTAKE_VELOCITY.get();
//     private static double outtakeVelocity = OUTTAKE_VELOCITY.get();


//     public IntakerSubsystem(
//         IntakePivotIO intakerPivotIO,
//         IntakerRollerIO intakerRollerIO,
//         BeambreakIO intakerBBIO
//     ){
//         super(intakerRollerIO,NAME);
//         this.intakerBBIO = intakerBBIO;
//         this.intakerPivotIO = intakerPivotIO;
//         this.intakerRollerIO = intakerRollerIO;
//     }

//     @Override
//     public void periodic() {
//         super.periodic();

//         intakerRollerIO.updateConfigs(roller_kp, roller_ki, roller_kd, roller_ka, roller_kv, roller_ks);
//         intakerPivotIO.updateConfigs(pivot_kp, pivot_ki, pivot_kd, pivot_ka, pivot_kv, pivot_ks);
//         intakerBBIO.updateInputs(intakeBBIOInputs);
//         intakerPivotIO.updateInputs(intakePivotIOInputs);

//         SystemState newState = handleStateTransition();

//         Logger.processInputs(NAME + "/Edge Beambreak", intakeBBIOInputs);
//         Logger.processInputs(NAME + "/Pivot", intakePivotIOInputs);
//         Logger.recordOutput("Intaker/SystemState",newState.toString());
//         if(newState!= systemState) {
//             systemState = newState;
//         }
//         if(DriverStation.isDisabled()){
//             systemState = SystemState.IDLING;
//         }
//         switch (systemState) {
//             case IDLING:
//                 io.setVelocity(idleRPS);
//                 intakerPivotIO.setMotorPosition(MIN_ANGLE);
//                 break;
//             case OUTTAKING:
//                 io.setVelocity(outtakeVelocity);
//                 intakerPivotIO.setMotorPosition(MAX_ANGLE);
//                 break;
//             case INTAKING:
//                 io.setVelocity(intakeVelocity);
//                 intakerPivotIO.setMotorPosition(MAX_ANGLE);
//                 break;
//             case OFF:
//             default:
//                 io.stop();
//                 break;
//         }
//         if (RobotConstants.TUNING) {
//             intakeVelocity = INTAKE_VELOCITY.get();
//             outtakeVelocity = OUTTAKE_VELOCITY.get();
//             idleRPS = INTAKE_IDLE_RPS.get();

//             pivot_ka = intakePivotGainsClass.INTAKE_PIVOT_KA.get();
//             pivot_kp = intakePivotGainsClass.INTAKE_PIVOT_KP.get();
//             pivot_ki = intakePivotGainsClass.INTAKE_PIVOT_KI.get();
//             pivot_kd = intakePivotGainsClass.INTAKE_PIVOT_KD.get();
//             pivot_ks = intakePivotGainsClass.INTAKE_PIVOT_KS.get();
//             pivot_kv = intakePivotGainsClass.INTAKE_PIVOT_KV.get();

//             roller_ka = intakeGainsClass.INTAKE_KA.get();
//             roller_kp = intakeGainsClass.INTAKE_KP.get();
//             roller_ki = intakeGainsClass.INTAKE_KI.get();
//             roller_kd = intakeGainsClass.INTAKE_KD.get();
//             roller_ks = intakeGainsClass.INTAKE_KS.get();
//             roller_kv = intakeGainsClass.INTAKE_KV.get();
//         }

//     };

//     private SystemState handleStateTransition() {
//         return switch (wantedState) {
//             case IDLE -> SystemState.IDLING;
//             case INTAKE -> {
//                 if(intakeBBIOInputs.isBeambreakOn){
//                     hasTransitionedToFunnel = true;
//                     yield SystemState.IDLING;
//                 }
//                 yield SystemState.INTAKING;
//             }
//             case OUTTAKE -> {
//                 if(!intakeBBIOInputs.isBeambreakOn){
//                     hasTransitionedToFunnel = false;
//                     yield SystemState.IDLING;
//                 }
//                 yield SystemState.OUTTAKING;
//             }
//             case OFF -> SystemState.OFF;
//             default -> SystemState.IDLING;
//         };
//     }

//     public boolean isIntakeFinished(){
//         return hasTransitionedToFunnel && !intakeBBIOInputs.isBeambreakOn;
//     }

//     public void setWantedState(WantedState wantedState) {this.wantedState = wantedState;}

//     public enum WantedState{
//         IDLE,
//         INTAKE,
//         OUTTAKE,
//         OFF
//     }
//     public enum SystemState{
//         IDLING,
//         INTAKING,
//         OUTTAKING,
//         OFF
//     }
// }