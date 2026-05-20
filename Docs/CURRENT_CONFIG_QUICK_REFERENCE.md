# Current Config — BLE Quick Reference

How to query FlySight 2 runtime configuration over BLE.

---

## 1. Prerequisites

The device must be in **Active Mode or Start Mode** (green or orange LED). The `SD_Control_Point` characteristic requires an **encrypted, bonded** connection, and the client must have **indications enabled** on that characteristic before sending any command.

**Characteristic:** `SD_Control_Point`  
**UUID:** `00000006-8e22-4541-9d4c-21edae82ed19`  
**Service UUID:** `00000001-cc7a-482a-984a-7f2ed5b3e58f`  
**Properties:** Write (command), Indicate (response)

---

## 2. Commands Overview

Three single-packet commands replace the old multi-packet `0x30` command. Each fits in one 20-byte ATT indication — no reassembly needed.

| Opcode | Command | Response size | What it returns |
|--------|---------|---------------|-----------------|
| `0x31` | `SD_CMD_GET_SENSOR_ODRS` | 13 bytes | ODR index + source for all 5 sensors |
| `0x32` | `SD_CMD_GET_RATES` | 14 bytes | GNSS rate, AL rate, AL enabled flag |
| `0x33` | `SD_CMD_GET_BLE_BUDGET` | 20 bytes | Bandwidth totals, budget OK flag, warning flags |

For per-sensor BLE dividers use the existing `SD_CMD_GET_BLE_DIVIDER (0x11)` (returns effective value, one sensor per call).

All responses follow the standard control-point format:  
`[0xF0][cmd_echo][status][data…]`  
If `status ≠ 0x01` the command failed — see `CP_STATUS_*` in `control_point_protocol.h`.

---

## 3. `0x31` — GET_SENSOR_ODRS

### Request

Write a single byte: `[0x31]`

### Response — 13 bytes total

```
[0xF0][0x31][0x01][baro_idx][baro_src]
[hum_idx][hum_src][accel_idx][accel_src]
[gyro_idx][gyro_src][mag_idx][mag_src]
```

| Byte | Field | Notes |
|------|-------|-------|
| 0 | `0xF0` | Response ID |
| 1 | `0x31` | Command echo |
| 2 | `0x01` | SUCCESS |
| 3 | `baro_odr` index | See ODR tables §8 |
| 4 | `baro_odr` source | See §5 |
| 5 | `hum_odr` index | |
| 6 | `hum_odr` source | |
| 7 | `accel_odr` index | |
| 8 | `accel_odr` source | |
| 9 | `gyro_odr` index | |
| 10 | `gyro_odr` source | |
| 11 | `mag_odr` index | |
| 12 | `mag_odr` source | |

ODR `requested == effective` always — ODRs are not auto-scaled, so only the effective index is returned.

---

## 4. `0x32` — GET_RATES

### Request

Write a single byte: `[0x32]`

### Response — 14 bytes total

```
[0xF0][0x32][0x01]
[gnss_req_lo][gnss_req_hi][gnss_eff_lo][gnss_eff_hi][gnss_src]
[al_req_lo][al_req_hi][al_eff_lo][al_eff_hi][al_src]
[al_enabled]
```

| Bytes | Field | Type | Notes |
|-------|-------|------|-------|
| 0 | `0xF0` | — | Response ID |
| 1 | `0x32` | — | Command echo |
| 2 | `0x01` | — | SUCCESS |
| 3–4 | `gnss_rate_ms.requested` | uint16 LE | ms — last value written by config.txt or control point |
| 5–6 | `gnss_rate_ms.effective` | uint16 LE | ms — what the device is actually using (may be clamped to 40–1000) |
| 7 | `gnss_rate_ms.source` | uint8 | See §5 |
| 8–9 | `al_rate_ms.requested` | uint16 LE | ms |
| 10–11 | `al_rate_ms.effective` | uint16 LE | ms |
| 12 | `al_rate_ms.source` | uint8 | |
| 13 | `al_enabled` | uint8 | 1 = ActiveLook subsystem active |

---

## 5. `0x33` — GET_BLE_BUDGET

### Request

Write a single byte: `[0x33]`

### Response — 20 bytes total

```
[0xF0][0x33][0x01]
[est_bps × 4][sensor_bps × 4][al_bps × 4]
[budget_ok][warning_flags × 4]
```

| Bytes | Field | Type | Notes |
|-------|-------|------|-------|
| 0 | `0xF0` | — | Response ID |
| 1 | `0x33` | — | Command echo |
| 2 | `0x01` | — | SUCCESS |
| 3–6 | `ble_estimated_bps` | uint32 LE | B/s — GPS + all sensors + AL |
| 7–10 | `ble_sensor_bps` | uint32 LE | B/s — sensors only |
| 11–14 | `ble_activelook_bps` | uint32 LE | B/s — ActiveLook only |
| 15 | `ble_budget_ok` | uint8 | 1 = within 1500 B/s safe limit |
| 16–19 | `warning_flags` | uint32 LE | Bitmask — see §6 |

> **GPS bandwidth** is not a separate field. Compute it independently as:
> `gps_bps = (1000 / gnss_rate_ms.effective) × 44`

---

## 6. Warning Flags (bitmask in `warning_flags`)

| Bit | Mask | Meaning |
|-----|------|---------|
| 0 | `0x01` | `BLE_OVER_BUDGET` — total bandwidth exceeds 1500 B/s safe limit |
| 1 | `0x02` | `GNSS_RATE_CLAMPED` — requested GNSS rate was out of 40–1000 ms range and clamped |
| 2 | `0x04` | `DIVIDER_AT_MIN_RATE` — at least one sensor is floored at 0.5 Hz minimum |
| 3 | `0x08` | `AL_RATE_TOO_FAST` — ActiveLook rate < 100 ms was rejected |
| 4 | `0x10` | `AL_HIGH_BW` — ActiveLook consuming > 30 % of total BLE budget (> 450 B/s) |

---

## 7. ODR Index Lookup Tables

### Barometer (LPS22HH) — `baro_odr` indices 0–7

| Index | Rate |
|-------|------|
| 0 | Power-down |
| 1 | 1 Hz |
| 2 | 10 Hz |
| 3 | 25 Hz |
| 4 | 50 Hz |
| 5 | 75 Hz |
| 6 | 100 Hz |
| 7 | 200 Hz (low-noise) |

### Humidity (HTS221 / SHT4x) — `hum_odr` indices 0–3

| Index | Rate |
|-------|------|
| 0 | Power-down |
| 1 | 1 Hz |
| 2 | 7 Hz |
| 3 | 12 Hz |

### Magnetometer (LIS2MDL) — `mag_odr` indices 0–3

| Index | Rate |
|-------|------|
| 0 | 10 Hz |
| 1 | 20 Hz |
| 2 | 50 Hz |
| 3 | 100 Hz |

### Accelerometer (LSM6DSO) — `accel_odr` indices 0–11

| Index | Rate |
|-------|------|
| 0 | Power-down |
| 1 | 12 Hz |
| 2 | 26 Hz |
| 3 | 52 Hz |
| 4 | 104 Hz |
| 5 | 208 Hz |
| 6 | 416 Hz |
| 7 | 833 Hz |
| 8 | 1666 Hz |
| 9 | 3333 Hz |
| 10 | 6666 Hz |
| 11 | 1.6 Hz (ultra-low-power) |

### Gyroscope (LSM6DSO) — `gyro_odr` indices 0–10

Same as accelerometer indices 0–10 (gyro does not support index 11).

---

## 8. Source Values

| Value | Meaning |
|-------|---------|
| `0` | Compiled-in default |
| `1` | Set by `config.txt` at boot |
| `2` | Changed by BLE control point at runtime |
| `3` | Computed by BLE budget algorithm (auto-calculated) |

A divider with `requested = 0` and `source = 3` (returned by `0x11`) means the device is auto-managing that sensor's BLE rate.

---

## 9. Packet Sizes Used in Budget Calculations

These are the byte counts the firmware uses when computing `ble_*_bytes_per_sec`.

| Stream | Bytes / packet | Notes |
|--------|---------------|-------|
| GPS (GNSS PVT) | 44 | Always included when GNSS enabled |
| Barometer | 11 | |
| Humidity | 9 | |
| Accelerometer | 19 | |
| Gyroscope | 27 | Includes quaternion (default BLE mask 0xF0) |
| Magnetometer | 13 | |
| ActiveLook | 60 | Conservative estimate per pageClearAndDisplay packet |

**Budget formula per sensor stream:** `bytes_per_sec = (odr_hz / divider) × packet_size`

ActiveLook is the exception: `al_bps = (1000 / al_rate_ms.effective) × 60`

---

## 10. Computing the Effective BLE Rate for a Sensor

```
odr_hz    = odr_table[sensor_odr_index]      # from 0x31
divider   = get_ble_divider(sensor_id)        # from 0x11
ble_rate  = odr_hz / divider                  # Hz
bps       = ble_rate × packet_size            # B/s
```

If divider == 0, treat as 1 (should not occur in a running device).

---

## 11. Python Decode Snippet

```python
import struct

def decode_sensor_odrs(indication: bytes) -> dict:
    """Parse a 0x31 GET_SENSOR_ODRS indication (13 bytes)."""
    assert indication[0] == 0xF0 and indication[1] == 0x31 and indication[2] == 0x01
    data = indication[3:]
    sensors = ['baro', 'hum', 'accel', 'gyro', 'mag']
    return {s: {'index': data[i*2], 'source': data[i*2+1]} for i, s in enumerate(sensors)}

def decode_rates(indication: bytes) -> dict:
    """Parse a 0x32 GET_RATES indication (14 bytes)."""
    assert indication[0] == 0xF0 and indication[1] == 0x32 and indication[2] == 0x01
    d = indication[3:]
    return {
        'gnss_rate_ms': {
            'requested': struct.unpack_from('<H', d, 0)[0],
            'effective': struct.unpack_from('<H', d, 2)[0],
            'source':    d[4],
        },
        'al_rate_ms': {
            'requested': struct.unpack_from('<H', d, 5)[0],
            'effective': struct.unpack_from('<H', d, 7)[0],
            'source':    d[9],
        },
        'al_enabled': bool(d[10]),
    }

def decode_ble_budget(indication: bytes) -> dict:
    """Parse a 0x33 GET_BLE_BUDGET indication (20 bytes)."""
    assert indication[0] == 0xF0 and indication[1] == 0x33 and indication[2] == 0x01
    d = indication[3:]
    est, sensor, al = struct.unpack_from('<III', d, 0)
    return {
        'ble_estimated_bps':   est,
        'ble_sensor_bps':      sensor,
        'ble_activelook_bps':  al,
        'ble_budget_ok':       bool(d[12]),
        'warning_flags':       struct.unpack_from('<I', d, 13)[0],
    }

SOURCE_NAMES = {0: 'DEFAULT', 1: 'FILE', 2: 'CONTROL_POINT', 3: 'AUTO_CALC'}

WARN_BITS = [
    (0x01, 'BLE_OVER_BUDGET'),
    (0x02, 'GNSS_RATE_CLAMPED'),
    (0x04, 'DIVIDER_AT_MIN_RATE'),
    (0x08, 'AL_RATE_TOO_FAST'),
    (0x10, 'AL_HIGH_BW'),
]
```
