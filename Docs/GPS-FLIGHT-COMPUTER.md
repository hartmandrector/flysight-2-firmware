# GPS ↔ Flight Computer Communication

Runtime control of the u-blox MAX-M8 GPS via BLE control point commands from the flight computer (VR headset or companion app).

## GPS Module

- **Chip**: u-blox MAX-M8 (GNSS receiver)
- **Interface**: UART at 921600 baud to STM32WB5M
- **Protocol**: UBX binary (not NMEA — all NMEA messages disabled at init)
- **Current output**: NAV-PVT + NAV-VELNED + TIM-TP + TIM-TM2 (every fix cycle)
- **Default rate**: 200ms (5 Hz), configurable down to 40ms (25 Hz)
- **Default model**: Airborne 2G (model 7)

## Dynamic Platform Models

The dynModel controls the u-blox internal Extended Kalman Filter — process noise, altitude/velocity limits, and static hold behavior.

| Value | Model | Max Alt | Max Vel | Max Accel | Notes |
|-------|-------|---------|---------|-----------|-------|
| 0 | Portable | 12 km | 310 m/s | 2g | General purpose |
| 2 | Stationary | 9 km | 10 m/s | 2g | Fixed position, tight filtering |
| 3 | Pedestrian | 9 km | 30 m/s | 2g | Walking, low dynamics |
| 4 | Automotive | 9 km | 100 m/s | 2g | Road vehicle |
| 5 | Sea | 9 km | 25 m/s | 2g | Boat, static hold at sea level |
| 6 | Airborne 1G | 50 km | 100 m/s | 1g | Slow aircraft, gentle maneuvers |
| 7 | Airborne 2G | 50 km | 250 m/s | 2g | **Current default** |
| 8 | Airborne 4G | 50 km | 500 m/s | 4g | Aerobatic, highest dynamics |

### What dynModel controls in the EKF

- **Process noise**: Higher accel models → larger innovation gates, faster velocity tracking
- **Altitude ceiling**: Airborne = 50 km; non-airborne = 9-12 km
- **Velocity constraints**: Pedestrian (30 m/s) would reject wingsuit terminal velocity
- **Static hold**: Non-airborne models lock position below a velocity threshold; airborne disables this
- **3D fix preference**: Airborne strongly prefers 3D; others allow 2D fallback

### Runtime switching

`UBX-CFG-NAV5` with `mask = 0x0001` applies immediately — no GPS reset needed. The firmware already sends this during `FS_GNSS_InitMessages()`. Same message can be sent at any time.

## Human Flight Use Cases

The device is helmet-mounted for all flight modes. A flight computer (BASElineXR VR headset or companion app) detects the current phase and commands the optimal GPS configuration.

### Flight Phases

| Phase | Duration | Model | Rate | Why |
|-------|----------|-------|------|-----|
| **Ground / hiking** | Minutes–hours | Pedestrian (3) | 1 Hz | Tight filtering, low power, good position at walking speed |
| **Airplane ride** | 15-30 min | Automotive (4) | 1 Hz | Low power, 100 m/s ceiling covers climb |
| **Exit detection** | Instant | → Airborne 2G (7) | 5-25 Hz | Flight computer detects jump — acceleration spike or altitude loss |
| **Freefall / wingsuit** | 30s–3 min | Airborne 2G (7) | 10-25 Hz | High dynamics, max accuracy needed |
| **Deployment** | 2-5 sec | Airborne 4G (8) | 25 Hz | **Critical**: violent deceleration (3-5g opening shock), need highest rate + widest innovation gates for snatch force capture |
| **Canopy flight** | 2-10 min | Airborne 1G (6) | 5 Hz | Gentle maneuvers, save power/bandwidth |
| **Landing** | 5-15 sec | Airborne 2G (7) | 10-25 Hz | Flare dynamics, ground approach, useful for landing analysis |
| **Post-landing** | — | Stationary (2) | 1 Hz | Lock position, minimal power |

### Deployment Phase — High Priority

Deployment is the most data-critical phase for safety analysis:
- **Opening shock**: 3-5g deceleration over 1-3 seconds
- **Head/neck loading**: Gyro data captures rotational violence during snatch force
- **Off-heading openings**: Yaw rate + acceleration → injury correlation
- **Line twist detection**: Sustained rotation post-opening
- GPS alone is too slow to capture deployment subtleties — high-rate IMU (accel + gyro) fills the gap
- Combined GPS + IMU during deployment enables reconstruction of the full 6DOF opening sequence

### Sensor Rate Strategy Per Phase

| Phase | GPS | Accel | Gyro | Mag | Baro |
|-------|-----|-------|------|-----|------|
| Ground | 1 Hz | 10 Hz | 10 Hz | 1 Hz | 1 Hz |
| Airplane | 1 Hz | 10 Hz | 10 Hz | 0.5 Hz | 1 Hz |
| Freefall | 10-25 Hz | 200 Hz | 200 Hz | 5 Hz | 10 Hz |
| Deployment | 25 Hz | **max** | **max** | 5 Hz | 10 Hz |
| Canopy | 5 Hz | 50 Hz | 50 Hz | 5 Hz | 5 Hz |
| Landing | 10-25 Hz | 200 Hz | 200 Hz | 5 Hz | 10 Hz |

## Implementation: Control Point Commands

### Existing

| Command | Code | Payload | Status |
|---------|------|---------|--------|
| SET_BLE_DIVIDER | 0x10 | sensor_id, divider_lo, divider_hi | ✅ Working |
| MAG_CALIBRATION | TBD | hard iron offsets, gain, rejection | ✅ Built |

### Proposed

| Command | Code | Payload | Description |
|---------|------|---------|-------------|
| SET_GNSS_MODEL | 0x12 | model_id (0-8) | Switch dynModel at runtime via UBX-CFG-NAV5 |
| SET_GNSS_RATE | 0x13 | rate_ms_lo, rate_ms_hi | Change measurement rate (40-1000 ms) |
| SET_FLIGHT_PHASE | 0x14 | phase_id | Composite command: sets model + GPS rate + BLE dividers all at once |
| GET_GNSS_STATUS | 0x15 | — | Returns: fix type, numSV, hAcc, vAcc, sAcc |

`SET_FLIGHT_PHASE` is the main interface — the flight computer sends a single command and the firmware applies the full configuration table for that phase. Individual commands (`SET_GNSS_MODEL`, `SET_GNSS_RATE`) available for fine-grained control.

## Available UBX Messages

### Currently Enabled (received every fix cycle)

| Message | Class.ID | Size | Content |
|---------|----------|------|---------|
| NAV-PVT | 01.07 | 92B | Position, velocity (NED), time, fix type, numSV, accuracy estimates, heading, DOP |
| NAV-VELNED | 01.12 | 36B | NED velocity, 3D/ground speed, heading, speed accuracy |
| TIM-TP | 0D.01 | — | Time pulse timing |
| TIM-TM2 | 0D.03 | — | Time mark (external event timing) |

### Available But Not Enabled

These can be enabled at runtime via `UBX-CFG-MSG`:

| Message | Class.ID | Size | Potential Use |
|---------|----------|------|---------------|
| **NAV-STATUS** | 01.03 | 16B | Fix quality flags, spoofing detection, PSM state — useful for fix health monitoring |
| **NAV-DOP** | 01.04 | 18B | All DOP values (GDOP, PDOP, TDOP, VDOP, HDOP, NDOP, EDOP) — geometry quality for Kalman weighting |
| **NAV-POSLLH** | 01.02 | 28B | Position with separate hAcc/vAcc (redundant with PVT) |
| **NAV-SOL** | 01.06 | 52B | ECEF position + velocity, 3D accuracy, DOP, numSV (redundant with PVT+VELNED) |
| **NAV-SAT** | 01.35 | variable | Per-satellite info: elevation, azimuth, signal strength, health — already enabled in raw mode |
| **MON-HW** | 0A.09 | — | Hardware status: antenna, jamming, noise level — useful for detecting bad RF environment |
| **MON-TXBUF** | 0A.08 | — | TX buffer usage — already enabled in raw mode |
| **MON-SPAN** | 0A.31 | — | Spectrum analysis — already enabled in raw mode |

### Configuration Commands (send anytime)

| Message | Class.ID | Description |
|---------|----------|-------------|
| **CFG-NAV5** | 06.24 | Set dynModel, fixMode, DOP masks, static hold threshold |
| **CFG-RATE** | 06.08 | Set measurement rate (ms), navigation rate |
| **CFG-MSG** | 06.01 | Enable/disable individual output messages and set rate |
| **CFG-RXM** | 06.11 | Power management mode (continuous / power save) |
| **CFG-PM2** | 06.3B | Extended power management configuration |
| **CFG-RST** | 06.04 | Reset receiver (hot/warm/cold start) |
| **CFG-VALSET** | 06.8A | Generic configuration (key-value, used for security) |

### High-Value Additions

1. **NAV-DOP** — DOP values let the Kalman filter weight GPS measurements by geometry quality. When PDOP is high (bad geometry), trust IMU more. Cheap to enable (18 bytes/fix).

2. **NAV-STATUS** — Spoofing detection flag. Also reports dead-reckoning vs full-fix status. Useful for integrity monitoring.

3. **MON-HW** — Jamming indicator + antenna status. Detect when GPS is degraded before the position goes bad.

4. **CFG-RXM power save** — Between jumps (packing, dirt diving), drop GPS to power save mode. Wake on motion or flight computer command.

## GPS Rate vs BLE Bandwidth Budget

At 25 Hz GPS: NAV-PVT (92B) + NAV-VELNED (36B) = 128 bytes/fix × 25 = 3,200 bytes/sec from GPS alone.
BLE 4.2 practical throughput: ~20-30 KB/sec with MTU 250.
Adding IMU at max rate consumes most of the remaining budget — the divider system manages this.

## Future: Automatic Flight Phase Detection

The flight computer already has mode detection (FlightComputer.java in BASElineXR). Trigger points:
- **Exit**: Sustained vertical velocity > 10 m/s + altitude loss (or acceleration spike from door)
- **Deployment**: Sudden deceleration (vertical velocity drops from ~50 m/s to ~5 m/s in 2-5 sec)
- **Landing**: Altitude < 100m AGL + low vertical velocity + ground proximity
- **Post-landing**: Ground speed < 1 m/s sustained

These detections send `SET_FLIGHT_PHASE` commands back to the FlySight device, which reconfigures GPS and sensor rates automatically.
