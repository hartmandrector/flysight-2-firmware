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

## Priority-Based Divider Auto-Calculation

### Algorithm Overview

When `BLE_*_Divider = 0` (auto mode), the firmware uses a **priority-based algorithm** that respects user ODR choices while preventing bandwidth overrun:

**Priority Order**:
1. **GPS rate** (highest priority, no divider)
2. **Manual dividers** (non-zero values, respected exactly)
3. **Auto dividers** (zero values, calculated to fit remaining budget)

**Core Principle**: Try to run sensors at **full ODR** (divider=1), only throttle when bandwidth requires it.

### Algorithm Steps

1. **Calculate GPS bandwidth** from `Rate` parameter:
   ```
   GPS_Hz = 1000 / Rate_ms
   GPS_bandwidth = GPS_Hz × 44 bytes
   ```

2. **Calculate manual divider bandwidth**:
   ```
   For each sensor with non-zero divider:
     Manual_bandwidth += (HW_rate_Hz / divider) × packet_size
   ```

3. **Calculate remaining budget for auto sensors**:
   ```
   Remaining_budget = 1500 - GPS_bandwidth - Manual_bandwidth
   ```

4. **Try auto sensors at full ODR** (divider=1):
   ```
   For each sensor with divider=0:
     Auto_bandwidth += (HW_rate_Hz / 1) × packet_size
   ```

5. **If auto bandwidth exceeds remaining budget**, scale by percentage:
   ```
   Scale_factor = Auto_bandwidth / Remaining_budget
   For each auto sensor:
     Scaled_divider = ceil(1 × Scale_factor)
     Enforce minimum rate: divider ≤ HW_rate_Hz / 0.5
   ```

6. **Otherwise**, use divider=1 (full ODR)


### Example Scenarios

#### Scenario 1: Default Configuration - Full ODR
**Config**: GPS 200ms (5Hz), All sensors default ODR, All dividers=0 (auto)
- Accel/Gyro: 12.5 Hz, Baro: 10 Hz, Mag: 10 Hz, Hum: 1 Hz

```
Step 1: GPS bandwidth
  GPS: 5 Hz  44 = 220 bytes/sec

Step 2: Manual divider bandwidth
  (None - all dividers are 0)
  Manual_bandwidth = 0 bytes/sec

Step 3: Remaining budget
  Remaining = 1500 - 220 - 0 = 1280 bytes/sec

Step 4: Try auto sensors at full ODR (divider=1)
  Accel:  12.5 Hz  19 = 238 bytes/sec
  Gyro:   12.5 Hz  19 = 238 bytes/sec
  Baro:   10 Hz  11 = 110 bytes/sec
  Mag:    10 Hz  13 = 130 bytes/sec
  Hum:    1 Hz  9 = 9 bytes/sec
  Auto_bandwidth = 725 bytes/sec

Step 5: Check if scaling needed
  725 < 1280  No scaling needed!

Final Result:
  All dividers = 1 (full ODR)
  Total: 220 + 725 = 945 bytes/sec (63% of limit)
  
BLE Rates:
  GPS: 5 Hz, Accel: 12.5 Hz, Gyro: 12.5 Hz
  Baro: 10 Hz, Mag: 10 Hz, Hum: 1 Hz
```

**Key Insight**: Default config runs all sensors at full ODR with plenty of headroom!

---

#### Scenario 2: High IMU ODR - Percentage Scaling
**Config**: GPS 200ms (5Hz), Accel/Gyro: 416 Hz (ODR=6), Other sensors default, All dividers=0

```
Step 1: GPS bandwidth
  GPS: 5 Hz  44 = 220 bytes/sec

Step 2: Manual divider bandwidth
  Manual_bandwidth = 0 bytes/sec

Step 3: Remaining budget
  Remaining = 1500 - 220 - 0 = 1280 bytes/sec

Step 4: Try auto sensors at full ODR
  Accel:  416 Hz  19 = 7,904 bytes/sec
  Gyro:   416 Hz  19 = 7,904 bytes/sec
  Baro:   10 Hz  11 = 110 bytes/sec
  Mag:    10 Hz  13 = 130 bytes/sec
  Hum:    1 Hz  9 = 9 bytes/sec
  Auto_bandwidth = 16,057 bytes/sec

Step 5: Scaling required!
  Scale_factor = 16,057 / 1,280 = 12.54
  
  Accel:  divider = ceil(1  12.54) = 13  416/13 = 32.0 Hz  19 = 608 bytes/sec
  Gyro:   divider = ceil(1  12.54) = 13  416/13 = 32.0 Hz  19 = 608 bytes/sec
  Baro/Mag/Hum similarly scaled
  
  Auto_bandwidth after scaling = 1,235 bytes/sec

Final Result:
  All dividers = 13
  Total: 220 + 1,235 = 1,455 bytes/sec (97% of limit)
  
BLE Rates:
  GPS: 5 Hz, Accel: 32 Hz, Gyro: 32 Hz
  Baro: 0.77 Hz, Mag: 0.77 Hz, Hum: 0.08 Hz
```

**Key Insight**: High-rate sensors maintain proportionally higher BLE rates (32 Hz vs 0.77 Hz).

---

#### Scenario 3: Manual Divider Priority
**Config**: GPS 200ms (5Hz), Accel=416 Hz with manual divider=5, All others auto

```
Step 1: GPS bandwidth = 220 bytes/sec

Step 2: Manual divider bandwidth
  Accel: 416 / 5 = 83.2 Hz  19 = 1,581 bytes/sec

Step 3: Remaining budget = 1500 - 220 - 1,581 = -301 (negative!)

Step 5: Auto sensors set to minimum rate (0.5 Hz)

Final Result:
  Accel: 83.2 Hz (manual, honored)
  Other sensors: 0.5 Hz (minimum)
  Total: 1,829 bytes/sec (122% over!)
   Validation fails, system unhealthy
```

**Key Insight**: Manual dividers have absolute priority, even when exceeding budget.

---

#### Scenario 4: Mixed Manual/Auto
**Config**: GPS 200ms (5Hz), Accel manual div=20, Baro manual div=2, Others auto

```
Step 1: GPS = 220 bytes/sec
Step 2: Manual = 450 bytes/sec (accel + baro)
Step 3: Remaining = 830 bytes/sec
Step 4: Auto sensors try full ODR
Step 5: Scale factor = 9.69

Final Result:
  Accel: 20.8 Hz (manual), Baro: 5.0 Hz (manual)
  Gyro: 41.6 Hz (auto, scaled)
  Total: 1,474 bytes/sec (98%)
```

**Key Insight**: Manual dividers respected, auto scales to remaining budget.

---

#### Scenario 5: High GPS Rate
**Config**: GPS 40ms (25Hz), Accel/Gyro: 416 Hz, Baro: 100 Hz, All auto

```
Step 1: GPS = 1,100 bytes/sec (high!)
Step 3: Remaining = 400 bytes/sec (limited!)
Step 5: Scale factor = 45.54 (extreme!)

Final Result:
  GPS: 25 Hz
  Accel/Gyro: 9.0 Hz (heavily throttled)
  Baro/Mag: 2.2 Hz
  Total: 1,495 bytes/sec (99.7%)
```

**Key Insight**: High GPS rate forces aggressive sensor throttling, but priorities maintained.

---

### Algorithm Comparison

| Scenario | Old (15 Hz target) | New (Full ODR) | Improvement |
|----------|-------------------|----------------|-------------|
| Default config | ~15 Hz | Full ODR (12.5/10/1) |  Respects user choice |
| High IMU (416 Hz) | 15 Hz (div=28) | 32 Hz (div=13) |  2 higher rate |
| Low sensors | Throttled to 15 Hz | Full ODR |  No waste |
| Manual dividers | Not prioritized | Absolute priority |  User control |
| GPS + sensors | Fixed 15 Hz | Percentage scaled |  Proportional |

---


**Minimum Rate**: All sensors maintain ≥ 0.5 Hz even under extreme bandwidth pressure
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
