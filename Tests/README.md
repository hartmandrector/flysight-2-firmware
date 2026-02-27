# FlySight 2 Tests & Tools

This directory contains unit tests and standalone tools for FlySight 2 firmware development.

## Tools

### `calc_ble_dividers.py` - BLE Divider Calculator

Standalone Python tool for calculating BLE divider values and bandwidth usage. Implements the same priority-based algorithm from `ble_config.c`.

**Use cases:**
- Plan BLE configurations before testing on hardware
- Verify bandwidth calculations
- Test different sensor ODR combinations
- Understand divider behavior (auto vs manual)

**Requirements:**
- Python 3.7+

**Usage:**
```bash
cd Tests

# Default configuration (GPS 5Hz, all sensors default ODR, auto dividers)
py calc_ble_dividers.py

# High IMU rate (416 Hz accel/gyro)
py calc_ble_dividers.py --accel-odr 6 --gyro-odr 6

# Manual divider (force accel to ~20.8 Hz = 416/20)
py calc_ble_dividers.py --accel-odr 6 --accel-div 20

# High GPS rate (25 Hz) with high IMU
py calc_ble_dividers.py --gps-ms 40 --accel-odr 6 --gyro-odr 6

# Mixed manual/auto dividers
py calc_ble_dividers.py --accel-odr 6 --accel-div 20 --baro-div 2

# Parse settings from a CONFIG.TXT file
py calc_ble_dividers.py --config-file ../Docs/CONFIG.TXT

# Show all options
py calc_ble_dividers.py --help
```

**Using CONFIG.TXT Files:**

Instead of specifying settings via command-line arguments, you can parse them directly from a FlySight CONFIG.TXT file:

```bash
# Use a config file
py calc_ble_dividers.py --config-file path/to/CONFIG.TXT

# Example with the default config
py calc_ble_dividers.py --config-file ../Docs/CONFIGcopydefault.TXT
```

The parser extracts these keys from CONFIG.TXT:
- `Rate` → GPS rate (ms)
- `Baro_ODR` → Barometer ODR index
- `Hum_ODR` → Humidity ODR index
- `Accel_ODR` → Accelerometer ODR index
- `Gyro_ODR` → Gyroscope ODR index
- `Mag_ODR` → Magnetometer ODR index
- `BLE_Baro_Divider`, `BLE_Hum_Divider`, etc. → Manual dividers

Lines with `;` comments are automatically stripped. Settings not in the file use firmware defaults (GPS 200ms, Baro ODR=2, etc.).

**All Options:**
```
--config-file <path> Load settings from a CONFIG.TXT file (overrides other arguments)
--gps-ms <ms>        GPS rate in milliseconds (default: 200 = 5Hz)
--baro-odr <idx>     Baro ODR: 0=0Hz, 1=1Hz, 2=10Hz, 3=25Hz, 4=50Hz,
                     5=75Hz, 6=100Hz, 7=200Hz (default: 2)
--hum-odr <idx>      Hum ODR: always 1Hz (default: 1)
--accel-odr <idx>    Accel ODR: 0=0Hz, 1=12.5Hz, 2=26Hz, 3=52Hz, 4=104Hz,
                     5=208Hz, 6=416Hz, 7=833Hz, 8=1666Hz, 9=3333Hz,
                     10=6666Hz, 11=1.6Hz (default: 1)
--gyro-odr <idx>     Gyro ODR: 0=0Hz, 1=12.5Hz, 2=26Hz, 3=52Hz, 4=104Hz,
                     5=208Hz, 6=416Hz, 7=833Hz, 8=1666Hz, 9=3333Hz,
                     10=6666Hz (default: 1)
--mag-odr <idx>      Mag ODR: 0=10Hz, 1=20Hz, 2=50Hz, 3=100Hz (default: 0)
--baro-div <n>       Baro divider: 0=auto, >0=manual (default: 0)
--hum-div <n>        Hum divider: 0=auto, >0=manual (default: 0)
--accel-div <n>      Accel divider: 0=auto, >0=manual (default: 0)
--gyro-div <n>       Gyro divider: 0=auto, >0=manual (default: 0)
--mag-div <n>        Mag divider: 0=auto, >0=manual (default: 0)
-h, --help           Show help message
```

**Output Example:**
```
===============================================================
                      INPUT CONFIGURATION
===============================================================
GPS Rate:      200 ms (5.00 Hz)

Sensor ODRs:
  Baro:        10.0 Hz (ODR=0)
  Humidity:    1.0 Hz (ODR=0)
  Accel:       416.0 Hz (ODR=6)
  Gyro:        416.0 Hz (ODR=6)
  Mag:         12.5 Hz (ODR=1)

Input Dividers:
  Baro         0 (AUTO)
  Humidity     0 (AUTO)
  Accel        0 (AUTO)
  Gyro         0 (AUTO)
  Mag          0 (AUTO)

===============================================================
                      CALCULATED DIVIDERS
===============================================================
  Baro:        13
  Humidity:    2
  Accel:       13
  Gyro:        13
  Mag:         13

===============================================================
                           BLE RATES
===============================================================
  GPS:         5.00 Hz (44 bytes/packet)
  Baro:        0.77 Hz (11 bytes/packet)
  Humidity:    0.50 Hz (9 bytes/packet)
  Accel:       32.00 Hz (19 bytes/packet)
  Gyro:        32.00 Hz (19 bytes/packet)
  Mag:         0.96 Hz (13 bytes/packet)

===============================================================
                        BANDWIDTH USAGE
===============================================================
  GPS          220.0 bytes/sec (14.7%)
  Baro         8.5 bytes/sec (0.6%)
  Humidity     4.5 bytes/sec (0.3%)
  Accel        608.0 bytes/sec (40.5%)
  Gyro         608.0 bytes/sec (40.5%)
  Mag          12.5 bytes/sec (0.8%)
  -----------------------------------------------------------
  TOTAL:       1461.5 bytes/sec (97.4% of 1500 limit)

===============================================================
                       VALIDATION RESULT
===============================================================
  [OK] VALID - Configuration within bandwidth limit
===============================================================
```

**Exit Codes:**
- `0` = Valid configuration (within bandwidth)
- `1` = Invalid configuration (exceeds bandwidth)

---

## Tests

### `test_time` - GNSS Time Normalization

Unit tests for GNSS time handling (nanosecond overflow, date/time conversions).

**Build & Run:**
```bash
cd Tests
make test_time    # Linux/WSL
./test_time

# Or on Windows with WSL:
wsl make test_time
wsl ./test_time
```

---

## Building Tests (Linux/WSL Only)

The `test_time` unit test requires gcc. On Windows, use WSL:

```bash
cd Tests
wsl make test_time
```

The Python calculator (`calc_ble_dividers.py`) requires no compilation - just run it with Python 3.7+.

---

## ODR Reference

### Baro (MS5611)
| Index | Rate |
|-------|------|
| 0 | 10 Hz |
| 1 | 20 Hz |
| 2 | 50 Hz |
| 3 | 100 Hz |

### Accel/Gyro (LSM6DSO32)
| Index | Rate |
|-------|------|
| 0 | 1.6 Hz |
| 1 | 12.5 Hz |
| 2 | 26 Hz |
| 3 | 52 Hz |
| 4 | 104 Hz |
| 5 | 208 Hz |
| 6 | 416 Hz |

### Mag (LIS2MDL)
Uses same index mapping as accel/gyro (typically set to index 1 = 10 Hz).

### Humidity (HTS221/SHT4x)
Fixed 1 Hz output rate (ODR index ignored).

---

## Algorithm Reference

The BLE divider calculator uses the **priority-based algorithm**:

1. **GPS bandwidth** calculated first (always enabled)
2. **Manual dividers** honored with exact bandwidth
3. **Auto dividers** get remaining budget:
   - Try full ODR (divider=1) first
   - If exceeds budget, scale all proportionally
   - Enforce 0.5 Hz minimum rate

See `Docs/sensorbleplanning_v2.md` for detailed algorithm documentation.
