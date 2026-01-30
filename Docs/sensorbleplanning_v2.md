# BLE Sensor Streaming Implementation

**Status**: âœ… Complete  
**Date**: 2026-01-29  
**Pull Request**: #62 - Add BLE sensor streaming and mode control

## Overview

BLE sensor streaming transmits IMU, barometer, magnetometer, humidity, and GPS data over Bluetooth Low Energy with automatic bandwidth management. The system respects user-configured sensor ODR settings while preventing BLE radio overrun through GPS-aware divider auto-calculation.

**Key Features**:
- **GPS-aware auto-calculation**: Reserves GPS bandwidth first, distributes remaining budget among sensors
- **Configurable dividers**: Auto-calculate (divider=0) or manual override per sensor
- **Bandwidth validation**: Ensures total throughput stays < 1500 bytes/sec
- **Runtime adjustment**: Control point commands to modify dividers on-the-fly

---

## Sensor Configuration (ODR Tables)

All sensors have user-configurable Output Data Rates in config.txt:

### GPS/GNSS (u-blox)
**Parameter**: `Rate` (milliseconds between measurements)

| Rate (ms) | Frequency (Hz) | BLE Bandwidth @ 44 bytes |
|-----------|----------------|--------------------------|
| 1000 | 1 | 44 bytes/sec |
| 200 | 5 | 220 bytes/sec |
| 50 | 20 | 880 bytes/sec |
| 40 | 25 | 1,100 bytes/sec |

**Note**: GPS has no divider - user controls rate directly. Tested and accurate up to 25Hz.

### Barometer (LPS22HH)
**Parameter**: `Baro_ODR` (0-7)

| ODR | Rate (Hz) | Notes |
|-----|-----------|-------|
| 0 | 0 | Power-down |
| 1 | 1 | |
| 2 | 10 | Default |
| 3 | 25 | |
| 4 | 50 | |
| 5 | 75 | |
| 6 | 100 | |
| 7 | 200 | High current |

### Humidity (HTS221/SHT4x)
**Parameter**: `Hum_ODR` (0-3)

| ODR | Rate (Hz) | Notes |
|-----|-----------|-------|
| 0 | 0 | Power-down |
| 1 | 1 | Default |
| 2 | 7 | HTS221 only |
| 3 | 12.5 | HTS221 only |

### IMU - Accelerometer (LSM6DSO)
**Parameter**: `Accel_ODR` (0-11)

| ODR | Rate (Hz) | Power Mode |
|-----|-----------|------------|
| 0 | 0 | Power-down |
| 1 | 12.5 | Low-power (default) |
| 2 | 26 | Low-power |
| 3 | 52 | Low-power |
| 4 | 104 | Normal |
| 5 | 208 | Normal |
| 6 | 416 | High-performance |
| 7 | 833 | High-performance |
| 8 | 1666 | High-performance |
| 9 | 3333 | High-performance |
| 10 | 6666 | High-performance (max) |

### IMU - Gyroscope (LSM6DSO)
**Parameter**: `Gyro_ODR` (0-11) - Same table as Accelerometer

### Magnetometer (LIS2MDL)
**Parameter**: `Mag_ODR` (0-3)

| ODR | Rate (Hz) | Notes |
|-----|-----------|-------|
| 0 | 10 | Default |
| 1 | 20 | |
| 2 | 50 | |
| 3 | 100 | Max |

---

## BLE Bandwidth Constraints

**Safe Throughput Limit**: 1,500 bytes/sec (sustained)  
**Connection Interval**: 80-100ms typical  
**Effective MTU**: 250 bytes

### Packet Sizes

| Sensor | Packet Size | Fields Included |
|--------|-------------|-----------------|
| GPS | 44 bytes | Time, position, velocity, accuracy |
| Barometer | 11 bytes | Mask + timestamp + pressure + temp |
| Humidity | 9 bytes | Mask + timestamp + humidity + temp |
| Accelerometer | 19 bytes | Mask + timestamp + XYZ + temp |
| Gyroscope | 19 bytes | Mask + timestamp + XYZ + temp |
| Magnetometer | 13 bytes | Mask + timestamp + XYZ + temp |

---

## GPS-Aware Divider Auto-Calculation

### Algorithm Overview

When `BLE_*_Divider = 0` (auto mode), the firmware calculates optimal dividers using GPS bandwidth awareness:

1. **Calculate GPS bandwidth** from `Rate` parameter:
   ```
   GPS_Hz = 1000 / Rate_ms
   GPS_bandwidth = GPS_Hz Ã— 44 bytes
   ```

2. **Calculate remaining sensor budget**:
   ```
   Sensor_budget = 1500 - GPS_bandwidth
   (Minimum 100 bytes/sec reserved)
   ```

3. **Calculate initial dividers** for each enabled sensor:
   - Target 15Hz BLE per sensor (if hardware rate allows)
   - Formula: `divider = ceil(HW_rate_Hz / 15)`

4. **Calculate total sensor bandwidth** at initial dividers

5. **If exceeds sensor budget**, scale all dividers proportionally:
   ```
   Scale_factor = Total_sensor_bandwidth / Sensor_budget
   Final_divider = ceil(Initial_divider Ã— Scale_factor)
   ```

### Example Scenarios

#### Scenario 1: Default Configuration (GPS 5Hz)
**Config**: GPS 200ms (5Hz), IMU 12.5Hz, Baro 10Hz, Mag 10Hz, Hum 1Hz

```
GPS:   5 Hz Ã— 44 = 220 bytes/sec
Remaining budget: 1500 - 220 = 1280 bytes/sec

Sensors (initial dividers, target 15Hz):
  Accel:  12.5 Hz Ã· 1 = 12.5 Hz Ã— 19 = 238 bytes/sec
  Gyro:   12.5 Hz Ã· 1 = 12.5 Hz Ã— 19 = 238 bytes/sec
  Baro:   10 Hz Ã· 1 = 10 Hz Ã— 11 = 110 bytes/sec
  Mag:    10 Hz Ã· 1 = 10 Hz Ã— 13 = 130 bytes/sec
  Hum:    1 Hz Ã· 1 = 1 Hz Ã— 9 = 9 bytes/sec
  Total sensors: 725 bytes/sec

725 < 1280 â†’ No scaling needed
Final: GPS + sensors = 945 bytes/sec (63% of limit) âœ“
```

#### Scenario 2: High GPS Rate (GPS 25Hz)
**Config**: GPS 40ms (25Hz), IMU 416Hz, Baro 100Hz, Mag 100Hz, Hum 12.5Hz

```
GPS:   25 Hz Ã— 44 = 1100 bytes/sec
Remaining budget: 1500 - 1100 = 400 bytes/sec

Sensors (initial dividers, target 15Hz):
  Accel:  416 Hz Ã· 28 = 14.9 Hz Ã— 19 = 283 bytes/sec
  Gyro:   416 Hz Ã· 28 = 14.9 Hz Ã— 19 = 283 bytes/sec
  Baro:   100 Hz Ã· 7 = 14.3 Hz Ã— 11 = 157 bytes/sec
  Mag:    100 Hz Ã· 7 = 14.3 Hz Ã— 13 = 186 bytes/sec
  Hum:    12.5 Hz Ã· 1 = 12.5 Hz Ã— 9 = 113 bytes/sec
  Total sensors: 1022 bytes/sec

1022 > 400 â†’ Scaling required!
Scale factor: 1022 / 400 = 2.56Ã—

Final dividers (scaled):
  Accel:  28 Ã— 2.56 = 72 â†’ 416/72 = 5.8 Hz Ã— 19 = 110 bytes/sec
  Gyro:   28 Ã— 2.56 = 72 â†’ 416/72 = 5.8 Hz Ã— 19 = 110 bytes/sec
  Baro:   7 Ã— 2.56 = 18 â†’ 100/18 = 5.6 Hz Ã— 11 = 62 bytes/sec
  Mag:    7 Ã— 2.56 = 18 â†’ 100/18 = 5.6 Hz Ã— 13 = 73 bytes/sec
  Hum:    1 Ã— 2.56 = 3 â†’ 12.5/3 = 4.2 Hz Ã— 9 = 38 bytes/sec
  Total sensors: 393 bytes/sec

Final: GPS + sensors = 1493 bytes/sec (99.5% of limit) âœ“
```

**Key Insight**: High GPS rates automatically reduce sensor BLE rates to stay within budget.

#### Scenario 3: Ultra-High Sensor ODR (GPS 5Hz, IMU 6666Hz)
**Config**: GPS 200ms (5Hz), IMU 6666Hz, Baro 200Hz, Mag 100Hz, Hum 7Hz

```
GPS:   5 Hz Ã— 44 = 220 bytes/sec
Remaining budget: 1500 - 220 = 1280 bytes/sec

Sensors (initial dividers, target 15Hz):
  Accel:  6666 Hz Ã· 445 = 15 Hz Ã— 19 = 285 bytes/sec
  Gyro:   6666 Hz Ã· 445 = 15 Hz Ã— 19 = 285 bytes/sec
  Baro:   200 Hz Ã· 14 = 14.3 Hz Ã— 11 = 157 bytes/sec
  Mag:    100 Hz Ã· 7 = 14.3 Hz Ã— 13 = 186 bytes/sec
  Hum:    7 Hz Ã· 1 = 7 Hz Ã— 9 = 63 bytes/sec
  Total sensors: 976 bytes/sec

976 < 1280 â†’ No scaling needed
Final: GPS + sensors = 1196 bytes/sec (80% of limit) âœ“
```

**Key Insight**: Even at extreme sensor rates (6666Hz), auto-calculation keeps BLE within safe limits.

---

## Configuration

### config.txt Settings

```ini
; GPS Rate (milliseconds between measurements)
Rate: 200  ; 5Hz (default)

; Sensor ODR Settings
Baro_ODR:  2   ; 10Hz (default)
Hum_ODR:   1   ; 1Hz (default)
Accel_ODR: 1   ; 12.5Hz (default)
Gyro_ODR:  1   ; 12.5Hz (default)
Mag_ODR:   0   ; 10Hz (default)

; BLE Divider Settings
; 0 = Auto-calculate (GPS-aware, recommended)
; 1-65535 = Manual override
BLE_Baro_Divider:  0  ; Auto (recommended)
BLE_Hum_Divider:   0  ; Auto (recommended)
BLE_Accel_Divider: 0  ; Auto (recommended)
BLE_Gyro_Divider:  0  ; Auto (recommended)
BLE_Mag_Divider:   0  ; Auto (recommended)
```

### Validation

Configuration is validated on boot and when BLE connects:

```c
FS_BLE_ValidationResult_t result = FS_BLE_ValidateConfig(config);
if (!result.valid) {
    // Log error: result.error_msg
    // System marked unhealthy but continues (allows debugging)
}
```

**Validation checks**:
- GPS bandwidth + sensor bandwidth < 1500 bytes/sec
- Reports estimated throughput and percent of limit
- Suggests fixes if exceeded

---

## Control Point Commands

Runtime adjustment via SD_Control_Point characteristic:

### SET_BLE_DIVIDER (0x10)
```
Request:  [0x10] [sensor_id] [divider_low] [divider_high]
Response: [0xF0] [0x10] [0x01=SUCCESS]

sensor_id: 0=Baro, 1=Hum, 2=Accel, 3=Gyro, 4=Mag
divider: 1-65535 (little-endian uint16_t)
```

### GET_BLE_DIVIDER (0x11)
```
Request:  [0x11] [sensor_id]
Response: [0xF0] [0x11] [0x01] [sensor_id] [divider_low] [divider_high]
```

**Note**: Runtime changes are temporary (reset on reboot). Modify config.txt for persistent changes.

---

## Implementation Summary

### Files Modified

**Core Algorithm**:
- `FlySight/ble_config.h` - Function declarations
- `FlySight/ble_config.c` - GPS-aware auto-calculation and validation
- `FlySight/config.c` - Config parsing and auto-calc invocation

**Per-Sensor Modules**:
- `FlySight/baro_ble.c` - Barometer decimation
- `FlySight/hum_ble.c` - Humidity decimation
- `FlySight/imu_ble.c` - IMU (accel/gyro) decimation
- `FlySight/mag_ble.c` - Magnetometer decimation

**Integration**:
- `FlySight/active_control.c` - Sensor BLE init and data callbacks
- `FlySight/active_mode.c` - Config validation on mode start
- `FlySight/sensor_data.c` - Control point command handlers

### Data Flow

```
Sensor HW (ODR rate)
    â†“
Data Ready Interrupt
    â†“
Sensor Callback
    â†“
â”œâ”€â†’ Write to log (SENSOR.CSV)
â””â”€â†’ Custom_SENSOR_Update()
        â†“
    Decimation check (++counter % divider)
        â†“
    Build BLE packet
        â†“
    BLE_TX_Queue (8 slots, drop-on-overflow)
        â†“
    BLE Radio (notifications)
```

---

## Testing & Validation

### Bandwidth Validation Examples

Test various configurations to verify auto-calculation:

```bash
# Low GPS, default sensors â†’ ~945 bytes/sec
Rate: 200, Accel_ODR: 1, Gyro_ODR: 1, all dividers: 0

# High GPS, low sensors â†’ ~1493 bytes/sec
Rate: 40, Accel_ODR: 6, Gyro_ODR: 6, all dividers: 0

# Extreme sensors â†’ ~1196 bytes/sec
Rate: 200, Accel_ODR: 10, Gyro_ODR: 10, all dividers: 0
```

### Expected Behavior

âœ… **Divider = 0**: Always safe, never exceeds 1500 bytes/sec  
âœ… **High GPS rate**: Sensors automatically get higher dividers  
âœ… **Low GPS rate**: Sensors use lower dividers (faster BLE)  
âœ… **Manual dividers**: Override auto-calc, but validation still runs  
âŒ **Invalid config**: Validation fails, logs error, marks system unhealthy

---

## References

- **Implementation**: PR #62 - Add BLE sensor streaming and mode control
- **BLE Documentation**: `Docs/ble.md` - Control point commands
- **Config Documentation**: `config.txt` template in `FlySight/config.c`
- **Sensor Datasheets**: LPS22HH, HTS221/SHT4x, LSM6DSO, LIS2MDL
