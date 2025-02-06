package frc.robot.subsystems.endeffector;

import frc.robot.subsystems.roller.RollerIOReal;

import static frc.robot.RobotConstants.*;
import static frc.robot.RobotConstants.EndEffectorConstants.*;

public class EndEffectorIOReal extends RollerIOReal implements EndEffectorIO {
    private static final int id = ENDEFFECTOR_MOTOR_ID;
    private static final String canbus = CAN_BUS_NAME;
    private static final int statorCurrentLimitAmps = STATOR_CURRENT_LIMIT_AMPS;
    private static final int supplyCurrentLimitAmps = SUPPLY_CURRENT_LIMIT_AMPS;
    private static final boolean invert = IS_INVERT;
    private static final boolean brake = IS_BRAKE;
    private static final double reduction = REDUCTION;

    public EndEffectorIOReal() {
        super(id, canbus, statorCurrentLimitAmps, supplyCurrentLimitAmps, invert, brake, reduction);
    }
}
