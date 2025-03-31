# FRC 10541 && 6941 IronPulse 2025 Competition Robot

## Troubleshooting

- If Stream Deck doesn't work properly
    - the profile of stream deck is in **profiles/FRC10541.streamDeckProfile**
    - see https://github.com/ashupp/Streamdeck-vJoy
- If simulation failed with **MSVC error**, set the project JDK to wpilib2025 and add simulate to gradle run/debug
  config in IDEA, see https://www.chiefdelphi.com/t/error-when-running-simulation-wpilib-2025-2-1/481435

## Stream Deck

Configure it in *Configure vJoy*, monitor it in *vJoy Monitor*

### Keyboard layout

- ID 1: Manual (elevator only)
- ID 2: Semi (swerve & elevator)
- ID 3: Auto (swerve & elevator & endeffector)
- ID 4: Left coral
- ID 5: Right coral
- ID 13-16: L1-4
- ID 17: Stop button for aiming
- ID 18-19: P1-2

## Calib

- Apriltag: WPICal
- Branch Height: L2\L3\L4
- Camera: save exposure

## Checklist

- Camera: pipeline
- OBS: on
- 
