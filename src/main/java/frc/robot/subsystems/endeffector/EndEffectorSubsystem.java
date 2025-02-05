package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;

import static frc.robot.RobotConstants.EndEffectorConstants.*;
import static frc.robot.RobotConstants.EndEffectorConstants.EndEffectorGainsClass.*;

public class EndEffectorSubsystem extends RollerSubsystem {

    public static final String NAME = "EndEffector";

    private final EndEffectorIO endEffectorIO;
    private final BeambreakIO intakeBBIO, shootBBIO;
    private BeambreakIOInputsAutoLogged intakeBBInputs = new BeambreakIOInputsAutoLogged();
    private BeambreakIOInputsAutoLogged shootBBInputs = new BeambreakIOInputsAutoLogged();

    public double kp = ENDEFFECTOR_KP.get();
    public double ki = ENDEFFECTOR_KI.get();
    public double kd = ENDEFFECTOR_KD.get();
    public double ka = ENDEFFECTOR_KA.get();
    public double kv = ENDEFFECTOR_KV.get();
    public double ks = ENDEFFECTOR_KS.get();

    private double indexRPS = INDEX_RPS.get();
    private double holdRPS = HOLD_RPS.get();
    private double ShootRPS = SHOOT_RPS.get();
    private double SpitRPS = SPIT_RPS.get();

    public EndEffectorSubsystem(EndEffectorIO endEffectorIO , BeambreakIO intakeBBIO, BeambreakIO shootBBIO) {
        super(endEffectorIO, NAME);
        this.endEffectorIO = endEffectorIO;
        this.intakeBBIO = intakeBBIO;
        this.shootBBIO = shootBBIO;
    }

    @Override
    public void periodic() {
        super.periodic();

        endEffectorIO.updateConfigs(kp, ki, kd, ka, kv, ks);
        intakeBBIO.updateInputs(intakeBBInputs);
        shootBBIO.updateInputs(shootBBInputs);

        Logger.processInputs(NAME + "/Intake Beambreak", intakeBBInputs);
        Logger.processInputs(NAME + "/Shoot Beambreak", shootBBInputs);

        if (RobotConstants.TUNING) {
            indexRPS = INDEX_RPS.get();
            holdRPS = HOLD_RPS.get();
            ShootRPS = SHOOT_RPS.get();
            SpitRPS = SPIT_RPS.get();

            kp = ENDEFFECTOR_KP.get();
            ki = ENDEFFECTOR_KI.get();
            kd = ENDEFFECTOR_KD.get();
            ka = ENDEFFECTOR_KA.get();
            kv = ENDEFFECTOR_KV.get();
            ks = ENDEFFECTOR_KS.get();
        }
    }

    public Command directRun() {
        return this.run(() -> setVoltage(8));
    }

    public Command index() {
        return Commands.sequence(setVelocity(indexRPS).until(() -> shootBBInputs.isBeambreakOn),
                setVelocity(holdRPS));
    }

    public Command hold() {
        return this.run(() -> setVelocity(holdRPS));
    }

    public Command shoot() {
        return this.run(() -> setVelocity(ShootRPS).until(() -> !shootBBInputs.isBeambreakOn));
    }

    public Command spit() {
        return this.run(() -> setVelocity(SpitRPS).until(() -> !shootBBInputs.isBeambreakOn));
    }

    public boolean getIntakeBeambreak() {
        return intakeBBInputs.isBeambreakOn;
    }

    public boolean getShootBeambreak() {
        return shootBBInputs.isBeambreakOn;
    }
}
