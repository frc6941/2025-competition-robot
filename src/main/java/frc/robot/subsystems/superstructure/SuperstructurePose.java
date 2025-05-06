package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class SuperstructurePose {
    private final DoubleSupplier elevatorHeight;
    private final Supplier<Rotation2d> endEffectorAngle;

    public static class Preset {
        public static final SuperstructurePose STOW = new SuperstructurePose(() -> 0.0, () -> Rotation2d.kZero);
        public static final SuperstructurePose START = new SuperstructurePose(() -> 0.0, () -> Rotation2d.kZero);
    }
} 