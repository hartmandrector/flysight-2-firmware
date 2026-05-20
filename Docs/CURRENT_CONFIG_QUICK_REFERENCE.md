# Current Config — BLE Quick Reference

How to pull a complete runtime configuration snapshot from a FlySight 2 device over BLE and decode every field.

---

## 1. Prerequisite

The device must be in **Active Mode or Start Mode** (green or orange LED). The `SD_Control_Point` characteristic requires an **encrypted, bonded** connection, and the client must have **indications enabled** on that characteristic before sending the command.

**Characteristic:** `SD_Control_Point`  
**UUID:** `00000006-8e22-4541-9d4c-21edae82ed19`  
**Service UUID:** `00000001-cc7a-482a-984a-7f2ed5b3e58f`  
**Properties:** Write (command), Indicate (response)

---

## 2. Sending the Command

Write a single byte to `SD_Control_Point`:

```
[0x30]
```

No additional payload. Total write length: **1 byte**.

---

## 3. Receiving and Reassembling the Response

The snapshot is too large for one 20-byte ATT packet, so it arrives as **5 consecutive indications** on the same `SD_Control_Point` characteristic.

### Packet framing

| Packet | Byte 0 | Byte 1 | Byte 2 | Byte 3 | Bytes 4–19 |
|--------|--------|--------|--------|--------|------------|
| **First** | `0xF0` (response ID) | `0x30` (echo) | `0x01` (SUCCESS) | `82` (total payload bytes) | Payload bytes 0–15 (16 bytes) |
| **Each continuation** | `0xF1` (continuation ID) | `seq` (1, 2, 3, …) | Payload bytes (up to 18) | … | … |

If byte 0 of an indication is `0xF0` and byte 2 is not `0x01`, the command failed — byte 2 is the status code (see `CP_STATUS_*` in `control_point_protocol.h`).

### Reassembly algorithm

```
buffer = []

on_indication(pkt):
    if pkt[0] == 0xF0 and pkt[1] == 0x30:
        if pkt[2] != 0x01:
            handle_error(pkt[2])
            return
        total_len = pkt[3]          # always 82
        buffer = pkt[4:]            # first 16 payload bytes
    elif pkt[0] == 0xF1:
        buffer += pkt[2:]           # append, skip 0xF1 + seq

    if len(buffer) >= total_len:
        decode_snapshot(buffer[:total_len])
```

---

## 4. Payload Layout — 82 bytes, all little-endian

### 4.1 Header

| Offset | Size | Field | Notes |
|--------|------|-------|-------|
| 0 | 4 | `revision` (uint32) | Increments on every CC mutation. Use for cache invalidation. |

### 4.2 Rate and ODR fields (12 fields × 5 bytes each = 60 bytes)

Each field uses the same 5-byte sub-structure:

```
[requested (uint16 LE)] [effective (uint16 LE)] [source (uint8)]
```

| Offset | Field | Valid range | Notes |
|--------|-------|-------------|-------|
| 4  | `gnss_rate_ms` | 40–1000 ms | GNSS measurement interval |
| 9  | `baro_odr` | 0–7 | See ODR table below |
| 14 | `hum_odr` | 0–3 | See ODR table below |
| 19 | `accel_odr` | 0–11 | See ODR table below |
| 24 | `gyro_odr` | 0–10 | See ODR table below |
| 29 | `mag_odr` | 0–3 | See ODR table below |
| 34 | `ble_baro_divider` | 0 = auto | 1 = every sample, N = every Nth sample |
| 39 | `ble_hum_divider` | 0 = auto | |
| 44 | `ble_accel_divider` | 0 = auto | |
| 49 | `ble_gyro_divider` | 0 = auto | |
| 54 | `ble_mag_divider` | 0 = auto | |
| 59 | `al_rate_ms` | ≥ 100 ms | ActiveLook refresh interval |

**`requested`** — the last value written via `config.txt` or BLE control point. For **divider fields**, `0` means auto-managed. For **ODR fields**, `requested` and `effective` are always the same index — ODR is not auto-scaled.  
**`effective`** — the value the device is actually using right now.  
**`source`** — see §5.

### 4.3 Summary fields

| Offset | Size | Field | Notes |
|--------|------|-------|-------|
| 64 | 1 | `al_enabled` (uint8) | 1 = ActiveLook subsystem active |
| 65 | 4 | `ble_estimated_bytes_per_sec` (uint32) | Total: GPS + AL + all sensors |
| 69 | 4 | `ble_sensor_bytes_per_sec` (uint32) | Sensors only (excludes GPS and AL) |
| 73 | 4 | `ble_activelook_bytes_per_sec` (uint32) | AL traffic only |
| 77 | 1 | `ble_budget_ok` (uint8) | 1 = within 1500 B/s limit |
| 78 | 4 | `warning_flags` (uint32) | See §6 |

> **Note:** GPS bandwidth is not broken out as a separate summary field. To compute it independently: `gps_bps = (1000 / gnss_rate_ms.effective) × 44`. The total is: `ble_estimated_bps = gps_bps + ble_sensor_bps + ble_activelook_bps`.

---

## 5. Source Values

| Value | Meaning |
|-------|---------|
| `0` | Compiled-in default |
| `1` | Set by `config.txt` at boot |
| `2` | Changed by BLE control point at runtime |
| `3` | Computed by BLE budget algorithm (auto-calculated) |

A divider with `requested = 0` and `source = 3` means the device is auto-managing that sensor's BLE rate.

---

## 6. Warning Flags (bitmask in `warning_flags`)

| Bit | Mask | Meaning |
|-----|------|---------|
| 0 | `0x01` | `BLE_OVER_BUDGET` — total bandwidth exceeds 1500 B/s safe limit |
| 1 | `0x02` | `GNSS_RATE_CLAMPED` — requested GNSS rate was out of 40–1000 ms range and clamped |
| 2 | `0x04` | `DIVIDER_AT_MIN_RATE` — at least one sensor is floored at 0.5 Hz minimum |
| 3 | `0x08` | `AL_RATE_TOO_FAST` — ActiveLook rate < 100 ms was rejected |
| 4 | `0x10` | `AL_HIGH_BW` — ActiveLook consuming > 30 % of total BLE budget |

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

## 8. Worked Example

Decoded bytes 34–38 (first `ble_baro_divider` field):

```
34: 0x00 0x00   → requested = 0   (auto)
36: 0x1C 0x00   → effective = 28
38: 0x03        → source    = 3   (AUTO_CALC)
```

Interpretation: the barometer divider is auto-managed. With baro ODR = 25 Hz (index 3) and effective divider 28, the BLE baro stream runs at 25/28 ≈ 0.9 Hz.

---

## 9. Computing the Effective BLE Rate for a Sensor

```
odr_hz    = odr_table[sensor_odr.effective]
ble_rate  = odr_hz / ble_sensor_divider.effective   # Hz
```

If `effective == 0`, treat as 1 (should not occur in a running device).

---

## 10. Full Python Decode Snippet

```python
import struct

def decode_cc_snapshot(buf: bytes) -> dict:
    assert len(buf) == 82

    def field(off):
        req, eff, src = struct.unpack_from('<HHB', buf, off)
        return {'requested': req, 'effective': eff, 'source': src}

    r = {}
    r['revision']               = struct.unpack_from('<I', buf, 0)[0]
    r['gnss_rate_ms']           = field(4)
    r['baro_odr']               = field(9)
    r['hum_odr']                = field(14)
    r['accel_odr']              = field(19)
    r['gyro_odr']               = field(24)
    r['mag_odr']                = field(29)
    r['ble_baro_divider']       = field(34)
    r['ble_hum_divider']        = field(39)
    r['ble_accel_divider']      = field(44)
    r['ble_gyro_divider']       = field(49)
    r['ble_mag_divider']        = field(54)
    r['al_rate_ms']             = field(59)
    r['al_enabled']             = buf[64]
    r['ble_estimated_bps'],     \
    r['ble_sensor_bps'],        \
    r['ble_activelook_bps']     = struct.unpack_from('<III', buf, 65)
    r['ble_budget_ok']          = buf[77]
    r['warning_flags']          = struct.unpack_from('<I', buf, 78)[0]
    return r

SOURCE_NAMES = {0: 'DEFAULT', 1: 'FILE', 2: 'CONTROL_POINT', 3: 'AUTO_CALC'}

WARN_BITS = [
    (0x01, 'BLE_OVER_BUDGET'),
    (0x02, 'GNSS_RATE_CLAMPED'),
    (0x04, 'DIVIDER_AT_MIN_RATE'),
    (0x08, 'AL_RATE_TOO_FAST'),
    (0x10, 'AL_HIGH_BW'),
]
```

---

## 11. Packet Sizes Used in Budget Calculations

These are the byte counts the firmware uses when computing `ble_*_bytes_per_sec`. Use them to independently verify bandwidth figures or spot-check the budget math.

| Stream | Bytes / packet | Notes |
|--------|---------------|-------|
| GPS (GNSS PVT) | 44 | Always included when GNSS enabled |
| Barometer | 11 | |
| Humidity | 9 | |
| Accelerometer | 19 | |
| Gyroscope | 27 | Includes quaternion (default BLE mask 0xF0) |
| Magnetometer | 13 | |
| ActiveLook | 60 | Conservative estimate per pageClearAndDisplay packet |

**Budget formula per stream:** `bytes_per_sec = (odr_hz / divider) × packet_size`

ActiveLook is the exception: `al_bps = (1000 / al_rate_ms.effective) × 60`

**AL high-bandwidth warning** (`CC_WARN_AL_HIGH_BW`) triggers when ActiveLook alone exceeds **30 %** of the 1500 B/s limit (i.e., > 450 B/s, equivalent to a refresh rate faster than ~25 ms at the 60-byte estimate).
