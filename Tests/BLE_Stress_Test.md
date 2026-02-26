# BLE Bandwidth Stress Test

**Date:** February 25, 2026  
**Device Config:** [CONFIG.TXT](../Docs/CONFIG.TXT)  
**Test Goal:** Determine BLE bandwidth limits by reducing dividers via Bluetooth control point until data loss occurs

---

## Baseline Configuration

Configuration loaded from [CONFIG.TXT](../Docs/CONFIG.TXT) and verified with `calc_ble_dividers.py`:

### Input Settings
- **GPS Rate:** 50 ms (20.00 Hz)
- **Baro ODR:** 3 (25 Hz)
- **Humidity ODR:** 1 (1 Hz)
- **Accel ODR:** 6 (416 Hz)
- **Gyro ODR:** 6 (416 Hz)
- **Mag ODR:** 3 (100 Hz)
- **All Dividers:** 0 (AUTO)

### Calculated Dividers (Auto Mode)
- Baro: 34
- Humidity: 2
- Accel: 34
- Gyro: 34
- Mag: 34

### Resulting BLE Rates
- **GPS:** 20.00 Hz (880.0 bytes/sec)
- **Baro:** 0.74 Hz (8.1 bytes/sec)
- **Humidity:** 0.50 Hz (4.5 bytes/sec)
- **Accel:** 12.24 Hz (232.5 bytes/sec)
- **Gyro:** 12.24 Hz (330.4 bytes/sec)
- **Mag:** 2.94 Hz (38.2 bytes/sec)
observed gps:20, gyro 12, accel 12, mag 18 baro 10 hum:broken 0
upstream fw: gps: 8-10 with queue problems, gyro/accel: 250, mag 50
Based on packet sizes (including quaternion in gyro data):
observed:
GPS @ 10 Hz: 10 × 44 bytes = 440 bytes/sec
Gyro @ 250 Hz (includes quaternion): 250 × 27 bytes = 6,750 bytes/sec
Accel @ 250 Hz: 250 × 19 bytes = 4,750 bytes/sec
Mag @ 50 Hz: 50 × 13 bytes = 650 bytes/sec
Baro @ 10 Hz: 10 × 11 bytes = 110 bytes/sec
Hum @ 1 Hz: 1 × 9 bytes = 9 bytes/sec
Total: 12,709 bytes/sec (847% of 1500 byte/sec BLE limit)
### Baseline Bandwidth
**Total:** 1493.6 bytes/sec (**99.6%** of 1500 limit)  
**Status:** ✅ VALID - Within bandwidth limit

---

## Test Procedure

1. **Connect to device** with baseline configuration (CONFIG.TXT already loaded)
2. **Verify baseline** - Confirm no data loss at 99.6% bandwidth
3. **Adjust dividers** via BLE control point:
   - Reduce dividers incrementally to increase BLE rates
   - Target specific sensors (e.g., reduce Accel/Gyro divider from 34 → 30 → 25 → 20...)
   - Push total bandwidth beyond 1500 bytes/sec threshold
4. **Monitor for data loss**:
   - Check for missing packets
   - Look for BLE queue overflow indicators
   - Note when data starts dropping

---

## Test Results

### Test Run 1 - Baseline Verification
**Dividers:** AUTO (Baro=34, Hum=2, Accel=34, Gyro=34, Mag=34)  
**Bandwidth:** 1493.6 bytes/sec (99.6%)  
**Data Loss:** [ x] None / [ ] Some / [ ] Severe  
**Notes:**


---

### Test Run 2 - Reduce Gyro Divider
**Divider Adjustment:** Gyro divider 34 → ___  
**Expected BLE Rate:** Gyro ___ Hz (___ bytes/sec)  
**Total Bandwidth:** ___ bytes/sec (___%)  
**Data Loss:** [ ] None / [ ] Some / [ ] Severe  
**Notes:**


---

### Test Run 3 - Reduce Accel Divider
**Divider Adjustment:** Accel divider 34 → ___  
**Expected BLE Rate:** Accel ___ Hz (___ bytes/sec)  
**Total Bandwidth:** ___ bytes/sec (___%)  
**Data Loss:** [ ] None / [ ] Some / [ ] Severe  
**Notes:**


---

### Test Run 4 - Reduce Multiple Dividers
**Divider Adjustments:**
- Gyro: 34 → ___
- Accel: 34 → ___
- Mag: 34 → ___

**Expected BLE Rates:**
- Gyro: ___ Hz (___ bytes/sec)
- Accel: ___ Hz (___ bytes/sec)
- Mag: ___ Hz (___ bytes/sec)

**Total Bandwidth:** ___ bytes/sec (___%)  
**Data Loss:** [ ] None / [ ] Some / [ ] Severe  
**Notes:**


---

### Test Run 5 - Push to Failure
**Final Dividers:** Baro=___, Hum=___, Accel=___, Gyro=___, Mag=___  
**Total Bandwidth:** ___ bytes/sec (___%)  
**Data Loss:** [ ] None / [ ] Some / [ ] Severe  
**Failure Threshold:** ___ bytes/sec  
**Notes:**


---

## Observations

### When Does Data Loss Begin?
- **Estimated Threshold:** ___ bytes/sec (___% of 1500 limit)
- **First Sensor to Drop:** ___
- **Symptoms:** 


### BLE Queue Behavior
- **Queue Overflow Indicators:**
- **Recovery Time:**
- **Packet Loss Pattern:**


### Firmware Behavior
- **Error Messages:**
- **LED Indicators:**
- **System Stability:**


---

## Conclusions

### Actual BLE Bandwidth Limit
- **Sustained Max:** ___ bytes/sec
- **vs. Theoretical 1500:** ___% difference


### Recommendations
1. 
2. 
3. 


---

## Reference

**Calculator Script Output:**
```
===============================================================
                      INPUT CONFIGURATION
===============================================================
GPS Rate:      50 ms (20.00 Hz)

Sensor ODRs:
  Baro:        25.0 Hz (ODR=3)
  Humidity:    1.0 Hz (ODR=1)
  Accel:       416.0 Hz (ODR=6)
  Gyro:        416.0 Hz (ODR=6)
  Mag:         100.0 Hz (ODR=3)

Input Dividers:
  Baro         0 (AUTO)
  Humidity     0 (AUTO)
  Accel        0 (AUTO)
  Gyro         0 (AUTO)
  Mag          0 (AUTO)

===============================================================
                      CALCULATED DIVIDERS
===============================================================
  Baro:        34
  Humidity:    2
  Accel:       34
  Gyro:        34
  Mag:         34

===============================================================
                           BLE RATES
===============================================================
  GPS:         20.00 Hz (44 bytes/packet)
  Baro:        0.74 Hz (11 bytes/packet)
  Humidity:    0.50 Hz (9 bytes/packet)
  Accel:       12.24 Hz (19 bytes/packet)
  Gyro:        12.24 Hz (27 bytes/packet)
  Mag:         2.94 Hz (13 bytes/packet)

===============================================================
                        BANDWIDTH USAGE
===============================================================
  GPS          880.0 bytes/sec (58.7%)
  Baro         8.1 bytes/sec (0.5%)
  Humidity     4.5 bytes/sec (0.3%)
  Accel        232.5 bytes/sec (15.5%)
  Gyro         330.4 bytes/sec (22.0%)
  Mag          38.2 bytes/sec (2.5%)
  -----------------------------------------------------------
  TOTAL:       1493.6 bytes/sec (99.6% of 1500 limit)

===============================================================
                       VALIDATION RESULT
===============================================================
  [OK] VALID - Configuration within bandwidth limit
===============================================================
```
