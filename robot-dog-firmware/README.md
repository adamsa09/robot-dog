# Robot Dog Firmware

ESP32-based quadruped robot firmware with PS5 DualSense Bluetooth control, analytical inverse kinematics, and phase-based diagonal trot gait. Built with PlatformIO / Arduino framework.

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | ESP32 (NodeMCU-32S), 240 MHz dual-core |
| Servo Driver | PCA9685 16-channel PWM @ I2C 0x40 |
| Servos | 8× (2 DOF per leg: hip + knee), 0–270° range |
| I2C Bus | SDA = GPIO 21, SCL = GPIO 22 |
| Controller | PS5 DualSense over Bluetooth |

**Leg geometry:** Hip-knee segment belt_L2 = 115.7 mm, knee-foot segment belt_L3 = 115.0 mm.

**Servo PWM range:** 537–2930 µs at 50 Hz.

## Leg Layout

```
FL (Front-Left)  = leg 0   FR (Front-Right) = leg 1
RL (Rear-Left)   = leg 2   RR (Rear-Right)  = leg 3
```

PCA9685 channels are assigned in pairs: leg N uses channels 2N (hip) and 2N+1 (knee).

## Directory Structure

```
robot-dog-firmware/
├── platformio.ini             # Build system: 3 environments
├── include/
│   └── config.h               # Central hardware & gait constants
├── lib/
│   ├── Calibration/           # Per-servo slope/intercept tuning
│   ├── Gait/                  # Phase-based foot trajectory interpolation
│   ├── IK/                    # 2-DOF analytical inverse kinematics
│   └── ServoDriver/           # PCA9685 abstraction
└── src/
    ├── main-directdrive.cpp   # Production gait engine (PS5 control)
    ├── main-16ch-servo.cpp    # Manual servo tuning tool (serial)
    └── main-ik.cpp            # IK solver validation (serial)
```

## Build Environments

Three PlatformIO environments share the same ESP32/Arduino platform:

| Environment | Source | Purpose |
|-------------|--------|---------|
| `directdrive` | main-directdrive.cpp | **Production**: full gait with PS5 controller |
| `16ch-servo` | main-16ch-servo.cpp | Servo tuning: manual angle control over serial |
| `ik` | main-ik.cpp | IK validation: test solver on front-right leg |

Build and flash a specific environment:

```bash
pio run -e directdrive -t upload
pio device monitor   # 115200 baud
```

## Algorithms

### Diagonal Trot Gait

The gait is driven by a single normalized phase value `gaitPhase ∈ [0, 1)` that increments each loop iteration:

```
gaitPhase += (Δt_ms / 1000) × speed_factor
```

Each leg has a fixed phase offset so that diagonally opposite pairs move together:

| Leg | Phase offset |
|-----|-------------|
| FL  | 0.0 (swing with RR) |
| FR  | 0.5 (swing with RL) |
| RL  | 0.5 |
| RR  | 0.0 |

### Foot Trajectory

Four keyframes define one complete stride cycle. The gait library linearly interpolates between them:

| Phase | x (mm) | z (mm) | Description |
|-------|--------|--------|-------------|
| 0.00  | 50     | 200    | Stance start |
| 0.25  | 50     | 155    | Swing: retract |
| 0.50  | −50    | 155    | Swing: reach forward |
| 0.75  | −50    | 200    | Swing end: extend |

### Inverse Kinematics (2-DOF Analytical)

Given a target foot position (x, z) in the leg's local frame:

1. **Reach distance:** `belt_L23 = sqrt(x² + z²)`
2. **Elbow angle:** `θ23 = acos((belt_L2² + belt_L3² − belt_L23²) / (2·belt_L2·belt_L3))`  (law of cosines)
3. **Knee raw:** `θ3 = 180° − θ23`
4. **Hip raw:** `θ2 = 90° − (asin(sin(θ23)·belt_L3/belt_L23) + atan(x/z))`
5. **Apply calibration:** `θ_servo = slope × θ_raw + intercept`

## Libraries

### `lib/Gait`

- `getFootPositionForPhase(phase, offset, keyframes, numKeys)`: applies phase offset, wraps to [0,1), linearly interpolates (x, z) between adjacent keyframes.

### `lib/IK`

- `solveIK(x, z, belt_L2, belt_L3, cal)` → `{theta2, theta3}`: returns calibrated servo angles or `NaN` if the point is unreachable.

### `lib/ServoDriver`

- `servoDriverBegin()`: initializes I2C and PCA9685 at 50 Hz.
- `writeServo(channel, degrees)`: converts 0–270° to a PWM pulse and writes to the PCA9685.

### `lib/Calibration`

Per-leg `LegCalibration` structs with `hip.slope`, `hip.intercept`, `knee.slope`, `knee.intercept`.

**Current status:** Front-right leg (leg 1) is calibrated. The other three legs have zero slopes and need tuning.

## PS5 Controller Mapping

| Input | Action |
|-------|--------|
| Left stick Y-axis | Speed factor (0–3×) |
| Triangle button | Cycle through Home → Stand → Walk modes |

## Configuration (`include/config.h`)

Key constants to adjust for mechanical changes:

```cpp
belt_L2                115.7 mm   // Hip-to-knee link length
belt_L3                115.0 mm   // Knee-to-foot link length
SERVO_US_MIN      537        // Pulse width at 0°
SERVO_US_MAX      2930       // Pulse width at 270°
GAIT_PERIOD_MS    1000       // Base stride period
NUM_KEYS          4          // Keyframes per gait cycle
```

## Calibration Workflow

1. Flash the `16ch-servo` environment.
2. Open serial monitor at 115200 baud.
3. Send `<channel>,<angle>` (e.g., `2,90`) to command individual servos.
4. Measure actual joint angles and record corresponding command angles.
5. Fit a linear slope/intercept and update `lib/Calibration/calibration.cpp`.
6. Verify with the `ik` environment by sending `x,z` coordinates and checking that the leg reaches the target.

## Known Limitations

- **2-DOF only**: no hip abduction/adduction; cannot adjust body height or step sideways.
- **Single gait**: diagonal trot only; no gallop, pace, or crawl modes.
- **Open-loop**: no IMU or foot-force sensors; unstable on uneven terrain.
- **No steering**: left-stick Y controls speed only; turning is not implemented.
- **Partial calibration**: only FR leg tuned; other legs need calibration before reliable walking.

## Dependencies

```ini
arminjo/ServoEasing @ ^3.6.0
adafruit/Adafruit PWM Servo Driver Library @ ^3.0.3
https://github.com/rodneybakiskan/ps5-esp32.git
```
