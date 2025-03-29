package frc.robot.subsystems.endeffectorarm;

import frc.robot.subsystems.roller.RollerIOReal;

import static frc.robot.RobotConstants.CANIVORE_CAN_BUS_NAME;
import static frc.robot.RobotConstants.EndEffectorArmConstants.*;
import static frc.robot.RobotConstants.EndEffectorArmConstants.EndEffectorArmRollerGainsClass.*;

public class EndEffectorArmRollerIOReal extends RollerIOReal implements EndEffectorArmRollerIO {
    private static final int id = END_EFFECTOR_ARM_ROLLER_MOTOR_ID;
    private static final String canbus = CANIVORE_CAN_BUS_NAME;
    private static final int statorCurrentLimitAmps = STATOR_CURRENT_LIMIT_AMPS;
    private static final int supplyCurrentLimitAmps = SUPPLY_CURRENT_LIMIT_AMPS;
    private static final boolean invert = IS_INVERT;
    private static final boolean brake = IS_BRAKE;

    public EndEffectorArmRollerIOReal() {
        super(id, canbus, statorCurrentLimitAmps, supplyCurrentLimitAmps, invert, brake);

        super.updateConfigs(END_EFFECTOR_ARM_ROLLER_KP.get(), END_EFFECTOR_ARM_ROLLER_KI.get(), END_EFFECTOR_ARM_ROLLER_KD.get(),
                END_EFFECTOR_ARM_ROLLER_KA.get(), END_EFFECTOR_ARM_ROLLER_KV.get(), END_EFFECTOR_ARM_ROLLER_KS.get());
    }
}
