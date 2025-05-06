package frc.robot.subsystems.superstructure;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import frc.robot.subsystems.superstructure.SuperstructurePose.Preset;

@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
    START(SuperstructureStateData.builder().build()),
    STOW(SuperstructureStateData.builder().pose(Preset.STOW).build());

    private final SuperstructureStateData value;
} 