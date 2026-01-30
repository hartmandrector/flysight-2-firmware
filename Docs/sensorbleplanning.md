# Sensor BLE Streaming Implementation Plan

**Status**: ✅ Implementation Complete (Phases 1-5)  
**Date**: 2026-01-28  
**Pull Request**: #62 - Add BLE sensor streaming and mode control

## Executive Summary

Implement proper BLE sensor data streaming that respects user-configured hardware sensor rates (ODR settings) while preventing BLE radio overrun through intelligent per-sensor decimation.

**PR Scope Note**: This PR contains two independent features:
1. **Sensor Streaming** (primary): BLE transmission of sensor data with configurable dividers
2. **Mode Control** (secondary): External control to override sensor settings

Mode control should override sensor streaming settings per the algorithm, but sensor streaming must work independently.

---

## Problem Statement

**Current Issue**: BLE sensor transmission ignores user-configured ODR settings and has hardcoded `divider = 1`, causing:
- Mismatch between logged data rates and BLE transmission rates
- Potential BLE radio buffer overrun at high ODR settings
- No relationship between config.txt sensor rates and BLE output

**Goal**: Mirror the logging system's sensor rate handling in BLE, with per-sensor configurable dividers that prevent BLE overrun while maximizing data fidelity.

---

## 1. Sensor Hardware Rate Configuration (ODR)

### Current Config Structure

All sensors have user-configurable Output Data Rates (ODR) in `config.txt`:

```c
// config.h
typedef struct {
    uint16_t rate;       // GPS measurement rate (milliseconds, e.g., 50ms = 20Hz)
    uint8_t  baro_odr;   // Barometric pressure sensor ODR
    uint8_t  hum_odr;    // Humidity sensor ODR  
    uint8_t  mag_odr;    // Magnetometer ODR
    uint8_t  accel_odr;  // Accelerometer ODR
    uint8_t  accel_fs;   // Accelerometer full scale
    uint8_t  gyro_odr;   // Gyroscope ODR
    uint8_t  gyro_fs;    // Gyroscope full scale
} FS_Config_Data_t;
```

**Note**: GPS rate is configured in milliseconds (e.g., `rate: 50` = 20Hz, `rate: 40` = 25Hz). GPS has been thoroughly tested and produces accurate, error-free data at all settings up to 25Hz. GPS data is currently transmitted over BLE faithfully at the configured rate.

### ODR to Hz Mapping Tables

#### GPS/GNSS (u-blox)
| Rate Value (ms) | Rate (Hz) | Notes |
|-----------------|-----------|-------|
| 1000 | 1 | Default |
| 500 | 2 | |
| 200 | 5 | |
| 100 | 10 | |
| 50 | 20 | Tested - accurate |
| 40 | 25 | Tested - accurate, max recommended |

**GPS BLE Transmission**: GPS data is already transmitted over BLE at the configured rate. No divider needed - works correctly as-is.

#### Sensors Not Requiring BLE Transmission
- **TIME**: Time sync data (sensor time, time of week) is handled internally for correlation between sensors and GPS. Not needed in BLE stream.
- **VBAT**: Battery voltage is already available through Device_State service. Not duplicated in sensor stream.

**Focus for this implementation**: Add HUM (humidity) BLE support. Verify TIME/VBAT exclusion.

#### Barometer (LPS22HH)
| ODR Value | Rate (Hz) | Power Mode | Notes |
|-----------|-----------|------------|-------|
| 0 | 0 | Power-down / One-shot | |
| 1 | 1 | | |
| 2 | 10 | | Default |
| 3 | 25 | | |
| 4 | 50 | | |
| 5 | 75 | | |
| 6 | 100 | | Max |
| 7 | 200 | Low-noise | High current |

#### Humidity (HTS221 or SHT4x)
| ODR Value | Rate (Hz) | Sensor | Notes |
|-----------|-----------|--------|-------|
| 0 | 0 | Both | Power-down / One-shot |
| 1 | 1 | Both | Default |
| 2 | 7 | HTS221 | |
| 3 | 12.5 | HTS221 | Max |

#### Magnetometer (LIS2MDL)
| ODR Value | Rate (Hz) | Power Mode | Notes |
|-----------|-----------|------------|-------|
| 0 | 10 | Continuous | Default |
| 1 | 20 | Continuous | |
| 2 | 50 | Continuous | |
| 3 | 100 | Continuous | Max |

#### Accelerometer (LSM6DSO)
| ODR Value | Rate (Hz) | Power Mode | Notes |
|-----------|-----------|------------|-------|
| 0 | 0 | Power-down | |
| 1 | 12.5 | Low-power | Default |
| 2 | 26 | Low-power | |
| 3 | 52 | Low-power | |
| 4 | 104 | Normal | |
| 5 | 208 | Normal | |
| 6 | 416 | High-performance | |
| 7 | 833 | High-performance | |
| 8 | 1666 | High-performance | |
| 9 | 3333 | High-performance | |
| 10 | 6666 | High-performance | Max |
| 11 | 1.6 | Ultra-low-power | |

#### Gyroscope (LSM6DSO)
| ODR Value | Rate (Hz) | Power Mode | Notes |
|-----------|-----------|------------|-------|
| 0 | 0 | Power-down | |
| 1 | 12.5 | Low-power | Default |
| 2 | 26 | Low-power | |
| 3 | 52 | Low-power | |
| 4 | 104 | Normal | |
| 5 | 208 | Normal | |
| 6 | 416 | High-performance | |
| 7 | 833 | High-performance | |
| 8 | 1666 | High-performance | |
| 9 | 3333 | High-performance | |
| 10 | 6666 | High-performance | Max |
| 11 | 1.6 | Ultra-low-power | |

---

## 2. Current Data Flow Analysis

### Logging System (WORKING CORRECTLY)
```
Sensor HW → Data Ready Interrupt → Callback → Write to circular buffer
                                                       ↓
                                            Log system reads buffer
                                                       ↓
                                            Timestamp ordering
                                                       ↓
                                            Write to SENSOR.CSV at ODR rate
```

**Key insight**: Logging respects ODR because hardware only generates interrupts at the configured rate.

### BLE System (IMPLEMENTED)
```
Sensor HW → Data Ready Interrupt → Callback → Custom_SENSOR_Update()
                                                       ↓
                                                Check divider (config-based)
                                                       ↓
                                                Decimation counter
                                                       ↓
                                                Send via BLE (if counter==0)
                                                       ↓
                                            BLE TX Queue (shared, 8 slots)
```

**Features**:
1. Config-based dividers (auto-calculated or manual override)
2. Respects user ODR settings with intelligent decimation
3. Validation prevents BLE overrun before streaming starts
4. Runtime adjustment via control point commands

---

## 3. BLE Radio Constraints

### Theoretical Limits
- **Max ATT MTU**: 250 bytes
- **Connection Interval**: 80-100ms (configurable: 1s-2.5s low-power)
- **Max packets per connection event**: ~6 (typical)
- **Effective throughput**: ~1500 bytes/sec sustained

### Per-Sensor Packet Sizes
| Sensor | Max Size | Typical Size (all fields) | Status |
|--------|----------|---------------------------|--------|
| GNSS | 44 bytes | 44 bytes | Already implemented |
| Baro | 12 bytes | 11 bytes (mask + time 4 + pressure 4 + temp 2) | Implemented (PR #62) |
| Hum | 12 bytes | 9 bytes (mask + time 4 + humidity 2 + temp 2) | **Implemented (PR #62)** |
| Accel | 20 bytes | 19 bytes (mask + time 4 + xyz 12 + temp 2) | Implemented (PR #62) |
| Gyro | 28 bytes | 27 bytes (mask + time 4 + xyz 12 + temp 2 + quaternion 8) | Implemented (PR #62) |
| Mag | 14 bytes | 13 bytes (mask + time 4 + xyz 6 + temp 2) | Implemented (PR #62) |

### Safe Transmission Budget

**Design Target**: Keep total BLE throughput < 1500 bytes/sec (safe margin below theoretical max)

#### Realistic Scenario (Default Configuration)
**Configuration**: GPS 20Hz, all sensors at default ODR settings  
**Use Case**: Typical skydiving/BASE jumping with high-quality GPS tracking

- GNSS: 44 bytes × 20Hz = **880 bytes/sec** (Rate: 50ms)
- Baro: 11 bytes × 10Hz = **110 bytes/sec** (ODR=2, no divider needed)
- Hum: 9 bytes × 1Hz = **9 bytes/sec** (ODR=1, no divider needed)
- Accel: 19 bytes × 12.5Hz = **238 bytes/sec** (ODR=1, no divider needed)
- Gyro: 27 bytes × 12.5Hz = **338 bytes/sec** (ODR=1, no divider needed)
- Mag: 13 bytes × 10Hz = **130 bytes/sec** (ODR=0, no divider needed)
- **Total**: **~1705 bytes/sec** ⚠️ **Over budget - needs divider adjustment**

**Analysis**: This matches real-world user data (TRACK.CSV example). GPS dominates at 58% of total bandwidth. All sensors transmit at their configured hardware rates with no dividers required.

#### Conservative Scenario (Minimum Power Configuration)
**Configuration**: GPS 5Hz, all sensors at minimum practical rates  
**Use Case**: Long-duration flights, battery conservation, lower data rate applications

- GNSS: 44 bytes × 5Hz = **220 bytes/sec** (Rate: 200ms)
- Baro: 11 bytes × 1Hz = **11 bytes/sec** (ODR=1, no divider needed)
- Hum: 9 bytes × 1Hz = **9 bytes/sec** (ODR=1, no divider needed)
- Accel: 19 bytes × 12.5Hz = **238 bytes/sec** (ODR=1, no divider needed)
- Gyro: 27 bytes × 12.5Hz = **338 bytes/sec** (ODR=1, no divider needed)
- Mag: 13 bytes × 10Hz = **130 bytes/sec** (ODR=0, no divider needed)
- **Total**: **~946 bytes/sec** ✓ **Good margin - 63% of budget**

**Analysis**: Even with GPS at minimum practical rate, sensor bandwidth usage remains comfortable. This configuration leaves significant headroom for future features or higher sensor rates if needed.

#### Maximum Rate Scenario (Demonstrates Overflow Without Dividers)
**Configuration**: GPS 25Hz, all sensors at maximum tested rates  
**Use Case**: Absolute maximum data collection - **requires dividers to prevent BLE overrun**

**Without dividers** (divider=1 for all sensors):
- GNSS: 44 bytes × 25Hz = **1100 bytes/sec** (Rate: 40ms) ⚠️
- Baro: 11 bytes × 100Hz = **1100 bytes/sec** (ODR=6, divider=1) ⚠️
- Hum: 9 bytes × 12.5Hz = **113 bytes/sec** (ODR=3, divider=1)
- Accel: 19 bytes × 416Hz = **7904 bytes/sec** (ODR=6, divider=1) ❌
- Gyro: 27 bytes × 416Hz = **11,232 bytes/sec** (ODR=6, divider=1) ❌
- Mag: 13 bytes × 100Hz = **1300 bytes/sec** (ODR=3, divider=1) ⚠️
- **Total**: **~22,749 bytes/sec** ❌ **MASSIVE OVERRUN - BLE will fail**

**With auto-calculated dividers** (targets 15Hz BLE per sensor):
- GNSS: 44 bytes × 25Hz = **1100 bytes/sec** (Rate: 40ms, no divider) ⚠️
- Baro: 11 bytes × 14.3Hz = **157 bytes/sec** (ODR=6: 100Hz, **divider=7**)
- Hum: 9 bytes × 12.5Hz = **113 bytes/sec** (ODR=3: 12.5Hz, **divider=1**)
- Accel: 19 bytes × 14.9Hz = **283 bytes/sec** (ODR=6: 416Hz, **divider=28**)
- Gyro: 27 bytes × 14.9Hz = **402 bytes/sec** (ODR=6: 416Hz, **divider=28**)
- Mag: 13 bytes × 14.3Hz = **186 bytes/sec** (ODR=3: 100Hz, **divider=7**)
- **Total**: **~2241 bytes/sec** ⚠️ **Still exceeds 1500 budget**

**Analysis**: Even with auto-calculated dividers, GPS at 25Hz (1100 bytes/sec) leaves insufficient headroom for all sensors at high ODR. **Validation must fail** and suggest either:
1. Reduce GPS rate to 20Hz or lower, OR
2. Reduce sensor ODR settings, OR
3. Increase manual dividers beyond auto-calculated values

This scenario demonstrates why configuration validation is critical.

#### Key Insights

1. **Default configuration needs adjustment**: With corrected packet sizes (19 bytes for IMU including timestamps), the default 20Hz GPS + 12.5Hz IMU configuration slightly exceeds the safe 1500 bytes/sec budget. **Recommend GPS 18Hz OR IMU divider=2**.
2. **GPS dominates bandwidth**: 44-byte packets make GPS the primary consumer at high rates
3. **TIME field impact**: 4-byte timestamps in every sensor packet add ~13-21% overhead but are essential for data synchronization
4. **Dividers enable flexibility**: Users can increase sensor ODR for logging without overwhelming BLE
5. **Conservative GPS rates leave headroom**: 5Hz GPS leaves 1200+ bytes/sec for sensors

#### Implementation Strategy

- **GPS**: No divider (user controls via `Rate` setting, tested up to 25Hz)
- **Other sensors**: 
  - Default divider = 1 (no decimation) for ODR ≤ 15Hz
  - Auto-calculate divider for ODR > 15Hz to target ~15Hz BLE rate
  - Manual override available via config for advanced users

---

## 4. Divider Calculation Algorithm

### Default Divider Strategy

**Goal**: Calculate per-sensor divider to ensure BLE output rate ≤ 15Hz (leaves headroom for GPS)

**Why 15Hz instead of 25Hz?**  
GPS can use up to 1100 bytes/sec at 25Hz, leaving only ~400 bytes/sec for all other sensors. Targeting 15Hz per sensor provides comfortable margin.

```c
uint16_t calculate_ble_divider(uint8_t odr_setting, const uint16_t *odr_table, uint8_t table_size)
{
    // Get hardware rate from ODR setting
    uint16_t hw_rate_hz = odr_table[odr_setting];
    
    // If hardware rate is 0 or already ≤ 15Hz, no division needed
    if (hw_rate_hz == 0 || hw_rate_hz <= 15) {
        return 1;
    }
    
    // Calculate minimum divider to bring rate to ≤ 15Hz
    // Round up to ensure we don't exceed 15Hz
    uint16_t divider = (hw_rate_hz + 14) / 15;  // Ceiling division
    
    // Note: No clamping - uint16_t supports up to 65535
    // Maximum practical: 6666Hz / 15Hz = 445
    
    return divider;
}
```

### Example Calculations

| Sensor | ODR Setting | HW Rate | Calculated Divider | BLE Rate |
|--------|-------------|---------|-------------------|----------|
| Accel | 1 | 12.5Hz | 1 | 12.5Hz |
| Accel | 4 | 104Hz | 7 | 14.9Hz |
| Accel | 6 | 416Hz | 28 | 14.9Hz |
| Accel | 9 | 3333Hz | 223 | 14.9Hz |
| Gyro | 6 | 416Hz | 28 | 14.9Hz |
| Mag | 3 | 100Hz | 7 | 14.3Hz |
| Baro | 6 | 100Hz | 7 | 14.3Hz |
| Hum | 3 | 12.5Hz | 1 | 12.5Hz |

---

## 5. Implementation Architecture

### 5.1 Configuration Extensions

#### Add to `config.h`:
```c
typedef struct {
    // ... existing fields ...
    
    // BLE-specific divider overrides (0 = use auto-calculated)
    uint16_t ble_baro_divider;   // Manual override for baro BLE rate
    uint16_t ble_hum_divider;    // Manual override for humidity BLE rate
    uint16_t ble_accel_divider;  // Manual override for accel BLE rate
    uint16_t ble_gyro_divider;   // Manual override for gyro BLE rate
    uint16_t ble_mag_divider;    // Manual override for mag BLE rate
} FS_Config_Data_t;
```

#### Add to `config.txt` (defaultConfig[]):
```
; Sensor BLE Settings
;
; BLE transmission dividers control how often sensor data is sent over Bluetooth.
; Default (0) = Auto-calculate to keep each sensor ≤ 15Hz (leaves headroom for GPS)
; Manual override = Divide hardware rate by this value
;
; Note: GPS rate is controlled by 'Rate' setting (no BLE divider)
; Example: Accel_ODR=6 (416Hz) with BLE_Accel_Divider=28 → 14.9Hz BLE rate
;
BLE_Baro_Divider:  0  ; 0 = Auto
BLE_Hum_Divider:   0  ; 0 = Auto (NEW)
BLE_Accel_Divider: 0  ; 0 = Auto
BLE_Gyro_Divider:  0  ; 0 = Auto
BLE_Mag_Divider:   0  ; 0 = Auto
```

### 5.2 BLE Module Updates

#### Each `sensor_ble.c` module needs:

```c
// Static state
static uint8_t s_mask = SENSOR_BLE_DEFAULT_MASK;
static uint16_t s_divider = 1;         // Note: uint16_t to support up to 65535
static uint16_t s_sample_counter = 0;  // For decimation (must match divider type)

// Initialization with divider calculation
void SENSOR_BLE_Init(const FS_Config_Data_t *config)
{
    s_mask = SENSOR_BLE_DEFAULT_MASK;
    
    // Calculate or use manual divider
    if (config->ble_sensor_divider == 0) {
        // Auto-calculate
        s_divider = calculate_ble_divider(config->sensor_odr, 
                                           sensor_odr_table, 
                                           SENSOR_ODR_TABLE_SIZE);
    } else {
        // Use manual override
        s_divider = config->ble_sensor_divider;
    }
    
    s_sample_counter = 0;  // Reset counter
}

// Decimation check in update function
void Custom_SENSOR_Update(const FS_Sensor_Data_t *current)
{
    // Decimation: only send every Nth sample
    if (++s_sample_counter < s_divider) {
        return;  // Skip this sample
    }
    s_sample_counter = 0;  // Reset counter
    
    // Build and send packet
    uint8_t length = SENSOR_BLE_Build(current, packet);
    
    if (Custom_App_Context.Sd_sensor_measurement_Notification_Status) {
        BLE_TX_Queue_SendTxPacket(CUSTOM_STM_SD_SENSOR_MEASUREMENT,
                                   packet, length, &SizeSd_Sensor_Measurement, 0);
    }
}
```

### 5.3 Initialization Sequence

```c
// In active_control.c or similar
void FS_ActiveMode_Init(void)
{
    const FS_Config_Data_t *config = FS_Config_Get();
    
    // Initialize BLE modules with config-aware dividers
    BARO_BLE_Init(config);
    HUM_BLE_Init(config);   // NEW: Add humidity support
    ACCEL_BLE_Init(config);
    GYRO_BLE_Init(config);
    MAG_BLE_Init(config);
    
    // ... rest of initialization ...
}
```

---

## 6. Data Flow (Proposed)

### Per-Sensor Flow

```
Sensor HW (ODR rate)
    ↓
Data Ready Interrupt
    ↓
Sensor Callback (e.g., FS_IMU_DataReady_Callback)
    ↓
┌─────────────────────────────────┐
│ 1. Write to log buffer          │ ← Existing (keep)
│ 2. Call Custom_SENSOR_Update()  │ ← Existing (keep)
└─────────────────────────────────┘
    ↓
Custom_SENSOR_Update()
    ↓
┌─────────────────────────────────┐
│ Check sample counter            │ ← NEW: Decimation logic
│   if (++counter < divider)      │
│       return; // Skip            │
│   counter = 0;                   │
└─────────────────────────────────┘
    ↓
Build BLE packet with current mask
    ↓
Check if notifications enabled
    ↓
BLE_TX_Queue_SendTxPacket()
    ↓
┌─────────────────────────────────┐
│ Shared BLE TX Queue             │
│ - Handles flow control          │
│ - Prioritizes by timestamp?     │
│ - Drops if queue full (ERROR)   │
└─────────────────────────────────┘
    ↓
BLE Radio Transmission
```

---

## 7. Queue Management Strategy

### Current Queue System
- Single shared queue: `BLE_TX_Queue`
- FIFO behavior
- No per-sensor queues

### Problem: Queue Overflow

**Scenario**: User configures all sensors at max rates, dividers set too low
- GPS at 40ms (25Hz) = 1100 bytes/sec (no divider)
- Accel at 3333Hz ÷ 10 = 333Hz × 15 bytes = 5000 bytes/sec
- Gyro at 3333Hz ÷ 10 = 333Hz × 15 bytes = 5000 bytes/sec
- Mag at 100Hz ÷ 1 = 100Hz × 13 bytes = 1300 bytes/sec
- Baro at 100Hz ÷ 1 = 100Hz × 11 bytes = 1100 bytes/sec
- Hum at 12.5Hz ÷ 1 = 12.5Hz × 11 bytes = 138 bytes/sec
- **Total**: ~13.6KB/sec → **MASSIVE BLE OVERRUN**

### Implemented Overflow Handling Strategy

**Approach**: Drop-on-overflow with validation (prevents regular overflow)

1. **Config Validation** (`FS_BLE_ValidateConfig()`):
   - Calculates total BLE bandwidth from GPS rate + all sensor ODR/divider settings
   - Checks against 1500 bytes/sec safe threshold
   - Called during active mode init, logs error if validation fails
   - Marks system unhealthy but doesn't prevent boot (allows debugging)

2. **Queue Drop** (`BLE_TX_Queue_SendTxPacket()`):
   - Returns NULL when 8-slot queue is full
   - Packets silently dropped (old data less valuable than new)
   - Debug logging added: "BLE_TX_Queue_SendTxPacket: buffer overflow (dropped)"
   - FIFO ordering preserved, no corruption

3. **Why This Works**:
   - Validation prevents misconfiguration at the source
   - Drop-on-overflow handles unexpected transient spikes
   - Simple, deterministic, no performance overhead
   - Per-sensor drop counters deemed unnecessary (validation ensures proper config)

---

## 8. Control Point Commands

### SD_Control_Point Extensions

Add new commands for runtime adjustment:

```c
#define SD_CMD_SET_BLE_DIVIDER    0x10  // Set divider for a sensor
#define SD_CMD_GET_BLE_DIVIDER    0x11  // Get current divider
#define SD_CMD_SET_BLE_MASK       0x12  // Set field mask (existing)
#define SD_CMD_GET_BLE_MASK       0x13  // Get field mask (existing)

// SET_BLE_DIVIDER payload:
// [sensor_id (uint8)] [divider_low (uint8)] [divider_high (uint8)]
//   sensor_id: 0=Baro, 1=Hum, 2=Accel, 3=Gyro, 4=Mag
//   divider: 1-65535 (0 reserved for auto, little-endian uint16_t)
//   Note: GPS has no divider (controlled by Rate setting)

// GET_BLE_DIVIDER payload:
// [sensor_id (uint8)]

// Response:
// [0xF0] [0x11] [CP_STATUS_SUCCESS] [sensor_id] [divider_low] [divider_high]
```

### Usage Example
```python
# Python client adjusting IMU rate
def set_ble_divider(sensor, divider):
    cmd = bytes([0x10, sensor, divider])
    char.write(cmd)
    response = await char.read()  # Wait for indication
    # response: [0xF0, 0x10, 0x01, sensor, divider]
```

---

## 9. Configuration Validation

### Validation Function

```c
typedef struct {
    bool valid;
    uint32_t estimated_bytes_per_sec;
    char error_msg[64];
} FS_BLE_ValidationResult_t;

FS_BLE_ValidationResult_t FS_BLE_ValidateConfig(const FS_Config_Data_t *config)
{
    FS_BLE_ValidationResult_t result = {0};
    uint32_t total = 0;
    
    // GPS rate (no divider - user controlled)
    if (config->rate > 0) {
        uint16_t gps_hz = 1000 / config->rate;  // Convert ms to Hz
        total += (gps_hz * 44);  // GPS packet is 44 bytes
    }
    
    // Helper macro for each sensor
    #define CHECK_SENSOR(name, odr_field, div_field, odr_table, packet_size) \
        do { \
            uint16_t hz = odr_table[config->odr_field]; \
            uint16_t div = (config->div_field == 0) ? \
                calculate_ble_divider(config->odr_field, odr_table, sizeof(odr_table)/sizeof(odr_table[0])) : \
                config->div_field; \
            if (hz > 0 && div > 0) total += ((hz / div) * packet_size); \
        } while(0)
    
    CHECK_SENSOR("Baro", baro_odr, ble_baro_divider, baro_odr_table, 11);
    CHECK_SENSOR("Hum", hum_odr, ble_hum_divider, hum_odr_table, 9);
    CHECK_SENSOR("Accel", accel_odr, ble_accel_divider, accel_odr_table, 19);
    CHECK_SENSOR("Gyro", gyro_odr, ble_gyro_divider, gyro_odr_table, 19);
    CHECK_SENSOR("Mag", mag_odr, ble_mag_divider, mag_odr_table, 13);
    
    result.estimated_bytes_per_sec = total;
    
    if (total > 1500) {
        result.valid = false;
        snprintf(result.error_msg, sizeof(result.error_msg),
                 "BLE overload: %lu bytes/sec (max 1500)", total);
    } else {
        result.valid = true;
    }
    
    return result;
}
```

### Integration Points

1. **Config file load**: Validate and warn
2. **BLE connection**: Validate before enabling notifications
3. **Control point**: Validate before applying new dividers

---

## 10. Testing Strategy

### Unit Tests
- [ ] ODR table accuracy (verify against datasheets)
- [ ] Divider calculation (test all ODR values)
- [ ] Sample counter decimation (verify every Nth sample sent)
- [ ] Configuration validation (test boundary cases)

### Integration Tests
- [ ] Single sensor streaming at various ODRs
- [ ] All sensors streaming simultaneously at default dividers
- [ ] Queue overflow behavior (intentionally overload)
- [ ] Control point commands (set/get dividers and masks)
- [ ] Config file parsing (valid and invalid dividers)

### Real-World Tests
- [ ] Battery life impact (compare to current)
- [ ] BLE connection stability over time
- [ ] Data accuracy (compare BLE to logged CSV)
- [ ] Mobile app compatibility (iOS/Android)

---

## 11. Implementation Phases

### Phase 1: Documentation & Infrastructure
- [x] Create planning document
- [x] Update planning document with GPS budget and humidity sensor
- [x] Add ODR tables to config.c comments (including GPS rate table)
- [x] Verify TIME and VBAT are not needed in BLE stream
- [x] Add BLE divider config parameters (exclude GPS - no divider needed)
- [x] Create ODR->Hz lookup tables in code
- [x] Create hum_ble.c module (mirror baro_ble.c structure)

**Status**: ✅ COMPLETE

### Phase 2: Core Divider Logic
- [x] Implement divider calculation algorithm (target 15Hz, not 25Hz)
- [x] Update BARO_BLE_Init() with config-based divider
- [x] Implement HUM_BLE_Init() with config-based divider (NEW)
- [x] Update ACCEL_BLE_Init() with config-based divider
- [x] Update GYRO_BLE_Init() with config-based divider
- [x] Update MAG_BLE_Init() with config-based divider
- [x] Update Custom_HUM_Update() with decimation (NEW)
- [x] Add validation function (must include GPS in budget calculation)

**Status**: ✅ COMPLETE

### Phase 3: Configuration Integration ✅ COMPLETE
- [x] Add config.txt parsing for BLE dividers (baro, hum, accel, gyro, mag)
- [x] Integrate validation into active mode init (FS_ActiveMode_Init)
- [x] Add sensor BLE Init() calls to active_control (FS_ActiveControl_Init)
- [x] Add Custom_HUM_Update() call to humidity data callback
- [x] Verify mode control independence

**Implementation**: 
- Added includes for all `*_ble.h` headers and `ble_config.h` to [active_control.c](../FlySight/active_control.c)
- Added sensor BLE initialization in `FS_ActiveControl_Init()`:
  - Gets config via `FS_Config_Get()`
  - Calls `BARO_BLE_Init(config)`, `HUM_BLE_Init(config)`, `ACCEL_BLE_Init(config)`, `GYRO_BLE_Init(config)`, `MAG_BLE_Init(config)`
  - Each Init() function auto-calculates divider if config value is 0, or uses manual override
- Added BLE config validation in [active_mode.c](../FlySight/active_mode.c):
  - Added `ble_config.h` include
  - Calls `FS_BLE_ValidateConfig()` after config is loaded
  - Logs error and marks system unhealthy if validation fails (bandwidth > 1500 bytes/sec)
- Added `Custom_HUM_Update()` call in `FS_Hum_DataReady_Callback()` in [active_control.c](../FlySight/active_control.c)

**Mode Control Independence**: 
- Mode control operates independently of sensor streaming
- Sensor BLE Init() is called once during active mode initialization with current config
- Mode control can override sensor settings via future control point commands (Phase 4)
- BLE transmission happens in sensor data callbacks (`Custom_*_Update()` functions) with decimation
- No dependencies between sensor streaming dividers and mode control state

### Phase 4: Control Point Commands ✅ COMPLETE
- [x] Add SET_BLE_DIVIDER command (opcode 0x10)
- [x] Add GET_BLE_DIVIDER command (opcode 0x11)
- [x] Update documentation ([ble.md](../Docs/ble.md))
- [x] Integration with existing sensor BLE modules

**Implementation**:
- Added `SD_CMD_SET_BLE_DIVIDER` (0x10) and `SD_CMD_GET_BLE_DIVIDER` (0x11) to [sensor_data.h](../FlySight/sensor_data.h)
- Implemented command handlers in [sensor_data.c](../FlySight/sensor_data.c):
  - `SD_CMD_SET_BLE_DIVIDER`: Accepts sensor_id (0=Baro, 1=Hum, 2=Accel, 3=Gyro, 4=Mag) and uint16_t divider (little-endian)
  - `SD_CMD_GET_BLE_DIVIDER`: Returns current divider for specified sensor
  - Validates sensor_id and rejects divider=0 (reserved for auto-calc at boot)
  - Calls existing `*_BLE_SetDivider()` and `*_BLE_GetDivider()` functions in each sensor module
- Updated [ble.md](../Docs/ble.md) SD_Control_Point documentation with command formats and response examples
- Runtime changes are temporary (reset on reboot) - config.txt changes persist

**Control Point Protocol**:
```
SET: [0x10] [sensor_id] [divider_low] [divider_high]
     Response: [0xF0] [0x10] [0x01=SUCCESS or 0x03=INVALID_PARAM]
     
GET: [0x11] [sensor_id]
     Response: [0xF0] [0x11] [0x01] [sensor_id] [divider_low] [divider_high]
```

### Phase 5: Queue Management ✅ COMPLETE
- [x] Analyze current queue behavior
- [x] Document overflow handling strategy
- [x] Add minimal diagnostic logging

**Analysis**:
- BLE TX queue uses 8-slot ring buffer (`FS_CRS_WINDOW_LENGTH = 8`)
- FIFO ordering with flow control (pauses when BLE stack reports insufficient resources)
- **Drop-on-overflow already implemented** in `BLE_TX_Queue_SendTxPacket()` - returns NULL when full
- All `Custom_*_Update()` functions use `BLE_TX_Queue_SendTxPacket()`, so drops are automatic

**Overflow Handling Strategy**:
- **Simple drop strategy**: When queue is full, silently drop the packet
- No packet corruption, no queue corruption, FIFO ordering preserved
- Appropriate for real-time sensor streaming (old data less valuable than new data)
- Dividers should be tuned via validation to prevent regular overflow

**Implementation**:
- Added debug logging to [ble_tx_queue.c](../STM32_WPAN/App/ble_tx_queue.c) `BLE_TX_Queue_SendTxPacket()`
- Logs `"BLE_TX_Queue_SendTxPacket: buffer overflow (dropped)"` when packet dropped
- Consistent with existing overflow logging in `BLE_TX_Queue_SendNextTxPacket()`
- Provides visibility for debugging without adding complexity or performance overhead

**Decision**: Per-sensor drop counters deemed unnecessary - validation function ensures dividers prevent regular overflow. Debug logging sufficient for diagnosing configuration issues.

### Phase 6: Testing & Validation
- [ ] Unit tests
- [ ] Integration tests
- [ ] Real-world testing with various configs
- [ ] Performance profiling

### Phase 7: Documentation 🔄 IN PROGRESS
- [x] Update ble.md with divider info (control point commands documented)
- [x] Update config.txt comments (BLE divider section complete)
- [x] Create comprehensive planning document (this document)
- [ ] Add user guide section (future work)
- [ ] Add troubleshooting guide (future work)

**Status**: Core technical documentation complete. User-facing guides deferred to later releases.

---

## 12. Edge Cases & Error Handling

### Case 1: Sensor Disabled (ODR = 0)
**Behavior**: Divider = 1, no BLE transmission (sensor not running)  
**Action**: None needed - handled by existing enable flags

### Case 2: Ultra-High ODR (e.g., 6666Hz)
**Behavior**: Auto-calculate divider = 445, BLE rate = 15Hz  
**Action**: Works correctly with default algorithm

### Case 3: Manual Divider Too Large (e.g., 1000 at 100Hz)
**Behavior**: BLE rate = 0.1Hz (extremely slow, impractical)  
**Action**: Allow - no upper limit on divider, but warn if BLE rate < 1Hz

### Case 4: Manual Divider = 1 at 6666Hz
**Behavior**: BLE overrun, packet loss  
**Action**: 
- Validation fails on config load
- Fall back to auto-calculated divider
- Log warning to event system

### Case 5: GPS at 25Hz (1100 bytes/sec)
**Behavior**: GPS alone uses 73% of safe budget  
**Action**: 
- No divider for GPS (user controls via Rate setting)
- Validation must account for GPS when checking other sensors
- Warning if GPS + sensors > 1500 bytes/sec

### Case 6: GPS at 25Hz + All Sensors at High ODR
**Behavior**: Total > 2500 bytes/sec, guaranteed overrun  
**Action**: 
- Validation fails on config load
- Suggest: Reduce GPS rate OR increase sensor dividers
- Display breakdown: "GPS: 1100 bps, Sensors: 1400 bps, Total: 2500 > 1500 limit"
- Refuse to enable BLE notifications until fixed

### Case 7: All Sensors Max ODR, Divider = 1
**Behavior**: Catastrophic overrun (~13KB/sec)  
**Action**:
- Validation MUST fail
- Refuse to enable BLE notifications
- Require valid config before streaming

### Case 8: Connection Lost During Streaming
**Behavior**: Queue fills, packets dropped  
**Action**:
- BLE_TX_Queue already handles this
- Existing connection callbacks handle cleanup
- No new code needed

### Case 9: Mid-Flight Divider Change
**Behavior**: Sample counter might skip/duplicate one sample  
**Action**:
- Reset sample counter when divider changes
- Document that brief glitch is possible
- Not a safety issue for this use case

---

## 13. Design Decisions (All Resolved)

1. ~~**GPS Rate**: Does GPS respect config.txt Rate parameter or is it fixed?~~
   - **RESOLVED**: GPS rate is user-configurable via `Rate` parameter (milliseconds). Tested and confirmed accurate up to 25Hz.

2. ~~**TIME and VBAT in BLE Stream**: Are these needed over BLE?~~
   - **RESOLVED - NOT NEEDED**:
     - **TIME**: Sensor time (system clock) is handled internally for correlation between GPS timestamps and sensor data. Each sensor packet includes its own timestamp via the mask bit (see #4), eliminating need for separate TIME data stream.
     - **VBAT**: Battery voltage already available via Device_State service characteristic. No need to duplicate in sensor stream.
   - **Action**: Verify timestamp reconstruction works correctly on receiver end (see Success Criteria)

3. ~~**Queue Priority**: Should sensors have different priorities?~~
   - **RESOLVED - NOT NEEDED**:
     - Keep FIFO (First-In-First-Out) with no priority
     - GPS already low-rate (≤25Hz) so conflicts minimal
     - Adding priority increases complexity without clear benefit
     - Can be added later if field testing shows need

4. ~~**Timestamp Sync**: Should BLE packets include timestamps?~~
   - **RESOLVED - USE MASK BIT**:
     - All sensor packets include timestamps when mask bit enabled (already implemented)
     - Receiver can reconstruct exact sequence and timing of both GPS and sensor data
     - No separate TIME data needed (see #2)
     - Current implementation is sufficient

5. ~~**Notification vs Indication**: Should we use indications for reliability?~~
   - **RESOLVED - USE NOTIFICATIONS**:
     - Keep notifications (best-effort, faster)
     - Accept occasional packet loss (typical for sensor streaming)
     - Indications add ~2x latency overhead for minimal benefit
     - High-rate sensors (100+ Hz) make indications impractical

6. ~~**Divider Persistence**: Should dividers persist across power cycles?~~
   - **RESOLVED - RECALCULATE ON BOOT**:
     - Always recalculate from config.txt on boot (Option A)
     - Simpler implementation, more predictable behavior
     - Users can modify config.txt to change dividers permanently
     - Runtime adjustments via control point are temporary (until reboot)

---

## 14. Success Criteria

**Phases 1-5 Complete**:

- [x] All sensors respect configured ODR rates via divider calculation
- [x] Default dividers target 15Hz BLE per sensor (leaves GPS headroom)
- [x] Configuration validation implemented (`FS_BLE_ValidateConfig()`) checks total bandwidth
- [x] Control point commands implemented (SET/GET_BLE_DIVIDER)
- [x] **TIME and VBAT exclusion verified**: TIME included in sensor packets (4-byte timestamp via mask), VBAT in Device_State service
- [x] Queue overflow handling implemented (drop-on-overflow with debug logging)
- [x] Core technical documentation complete (planning doc, ble.md updated)

**Pending Verification** (requires field testing):

- [ ] BLE data matches logged CSV data (accounting for divider)
- [ ] Timestamp reconstruction verified on receiver end
- [ ] No BLE disconnections due to buffer overrun in real-world conditions
- [ ] Integration tests with mobile apps (iOS/Android)
- [ ] Battery life impact assessment

---

## 15. References

- **STM32WB BLE Stack**: PM0271 - STM32WB Series BLE Stack Programming Guidelines
- **Sensor Datasheets**:
  - LPS22HH (Barometer)
  - HTS221 / SHT4x (Humidity)
  - LIS2MDL (Magnetometer)
  - LSM6DSO (IMU - Accel/Gyro)
- **Existing Code**:
  - `FlySight/log.c` - Sensor data logging (reference implementation)
  - `FlySight/sensor.c` - Sensor I2C communication
  - `FlySight/imu.c`, `baro.c`, `mag.c` - Sensor drivers
  - `STM32_WPAN/App/custom_app.c` - BLE application layer
  - `FlySight/*_ble.c` - BLE packet builders

---

## Appendix A: Config.txt Example (Full BLE Section)

```
; Sensor Hardware Rates (Output Data Rate)
;
; These configure the rate at which the sensor hardware sends data to the MCU.
; This determines the rate of data in SENSOR.CSV log files.
;
; Barometer (LPS22HH):
;   0 = Power-down, 1 = 1Hz, 2 = 10Hz, 3 = 25Hz, 4 = 50Hz, 
;   5 = 75Hz, 6 = 100Hz, 7 = 200Hz (high power)
; Humidity (HTS221/SHT4x):
;   0 = Power-down, 1 = 1Hz, 2 = 7Hz, 3 = 12.5Hz
; Magnetometer (LIS2MDL):
;   0 = 10Hz, 1 = 20Hz, 2 = 50Hz, 3 = 100Hz
; Accelerometer (LSM6DSO):
;   0 = Power-down, 1 = 12.5Hz, 2 = 26Hz, 3 = 52Hz, 4 = 104Hz,
;   5 = 208Hz, 6 = 416Hz, 7 = 833Hz, 8 = 1.666kHz, 9 = 3.333kHz,
;   10 = 6.666kHz, 11 = 1.6Hz (ultra-low-power)
; Gyroscope (LSM6DSO): Same as Accelerometer
;
Baro_ODR:       2  ; 10Hz
Hum_ODR:        1  ; 1Hz  
Mag_ODR:        0  ; 10Hz
Accel_ODR:      1  ; 12.5Hz
Gyro_ODR:       1  ; 12.5Hz

; Sensor BLE Transmission Dividers
;
; Controls how often sensor data is transmitted over Bluetooth.
; BLE_Rate = Hardware_Rate / Divider
;
; 0 = Auto-calculate (keeps all sensors ≤ 25Hz, recommended)
; 1-255 = Manual divider value
;
; Examples:
;   Accel_ODR=6 (416Hz), BLE_Accel_Divider=0 (auto) → 17 → 24.5Hz BLE
;   Accel_ODR=6 (416Hz), BLE_Accel_Divider=20       → 20.8Hz BLE
;   Accel_ODR=1 (12.5Hz), BLE_Accel_Divider=0       → 1 → 12.5Hz BLE
;
; WARNING: Setting dividers too low may cause BLE buffer overrun and
;          connection loss. Default (0) is safe for all configurations.
;
BLE_Baro_Divider:   0  ; Auto (recommended)
BLE_Accel_Divider:  0  ; Auto (recommended)
BLE_Gyro_Divider:   0  ; Auto (recommended)
BLE_Mag_Divider:    0  ; Auto (recommended)
```

---

**END OF DOCUMENT**
