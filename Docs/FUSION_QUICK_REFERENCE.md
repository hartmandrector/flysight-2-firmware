# Fusion AHRS Complete Integration Reference

## Quick Start

### 1. Configuration (CONFIG.TXT)
```ini
; Enable sensor fusion (0=off, 1=on)
Enable_Fusion:          1

; Fusion algorithm gain (0.01 to 10.0, default 0.46)
Fusion_Gain:            0.46

; Acceleration rejection threshold (degrees, 0-90, default 10)
Fusion_Accel_Reject:    10

; Magnetic rejection threshold (degrees, 0-90, default 10)  
Fusion_Mag_Reject:      10

; Recovery timeout (seconds, 1-60, default 5)
Fusion_Timeout:         5

; Hard-iron calibration (milligauss, default 0,0,0)
Fusion_Mag_Hard_Iron:   0,0,0

; Soft-iron calibration (3x3 matrix, default identity)
Fusion_Mag_Soft_Iron:   1,0,0,0,1,0,0,0,1
```

### 2. BLE Commands

#### Magnetometer Hard-Iron Calibration (0x20)
```
Byte 0:    0x20 (command ID)
Bytes 1-2: x offset (int16_t, milligauss)
Bytes 3-4: y offset (int16_t, milligauss)
Bytes 5-6: z offset (int16_t, milligauss)
Total: 7 bytes
```

#### Magnetometer Soft-Iron Calibration (0x21)
```
Byte 0:     0x21 (command ID)
Bytes 1-4:  XX element (int32_t, scaled by 1000000)
Bytes 5-8:  XY element
Bytes 9-12: XZ element
Bytes 13-16: YX element
Bytes 17-20: YY element
Bytes 21-24: YZ element
Bytes 25-28: ZX element
Bytes 29-32: ZY element
Bytes 33-36: ZZ element
Total: 37 bytes
```

### 3. CSV Output (SENSOR.CSV)

#### AHRS Data Format
```
$AHRS,time,q_w,q_x,q_y,q_z
```

Example:
```
$AHRS,12.345,10000,0,0,0
$AHRS,12.347,9998,174,-87,0
```

Scaling: Quaternion components × 10000

### 4. BLE Output (Gyroscope Characteristic)

#### Packet Structure
```
Byte 0:     Mask (0xF0 = all fields)
            0x80 = TIME
            0x40 = GYRO
            0x20 = TEMPERATURE
            0x10 = QUATERNION

If TIME bit set:
Bytes 1-4:  Time (uint32_t, milliseconds)

If GYRO bit set:
Bytes 5-8:   wx (int32_t, deg/s × 1000)
Bytes 9-12:  wy (int32_t, deg/s × 1000)
Bytes 13-16: wz (int32_t, deg/s × 1000)

If TEMPERATURE bit set:
Bytes 17-18: temp (int16_t, °C × 100)

If QUATERNION bit set:
Bytes 19-20: q_w (int16_t, × 10000)
Bytes 21-22: q_x (int16_t, × 10000)
Bytes 23-24: q_y (int16_t, × 10000)
Bytes 25-26: q_z (int16_t, × 10000)
```

## Configuration Parameters Explained

### Enable_Fusion
- **Type**: Boolean (0 or 1)
- **Default**: 0 (disabled)
- **Purpose**: Master switch for sensor fusion
- **Note**: When disabled, identity quaternion transmitted (w=1, x=y=z=0)

### Fusion_Gain
- **Type**: Float (0.01 to 10.0)
- **Default**: 0.46
- **Purpose**: Controls filter responsiveness
- **Lower values**: Smoother but slower response
- **Higher values**: Faster response but less filtering
- **Recommended**: 0.3-0.6 for most applications

### Fusion_Accel_Reject
- **Type**: Float (0 to 90 degrees)
- **Default**: 10°
- **Purpose**: Threshold for rejecting accelerometer when it differs from gravity
- **Use case**: Reject during high acceleration (skydiving, turns)
- **Recommended**: 5-15° depending on expected accelerations

### Fusion_Mag_Reject
- **Type**: Float (0 to 90 degrees)  
- **Default**: 10°
- **Purpose**: Threshold for rejecting magnetometer when field is distorted
- **Use case**: Reject near magnetic interference
- **Recommended**: 10-20° depending on environment

### Fusion_Timeout
- **Type**: Integer (1 to 60 seconds)
- **Default**: 5 seconds
- **Purpose**: Time before triggering gyroscope recovery if sensors rejected
- **Lower values**: Faster recovery but more prone to drift
- **Higher values**: More stable but slower recovery

### Fusion_Mag_Hard_Iron
- **Type**: 3 comma-separated integers (milligauss)
- **Default**: 0,0,0
- **Purpose**: Compensate for constant magnetic offsets
- **Example**: 150,-200,50
- **Calibration**: Collect min/max on each axis, offset = (min+max)/2

### Fusion_Mag_Soft_Iron
- **Type**: 9 comma-separated floats (3x3 matrix)
- **Default**: 1,0,0,0,1,0,0,0,1 (identity)
- **Purpose**: Compensate for magnetic field distortion
- **Example**: 1.05,0.02,-0.01,0.02,0.98,0.03,-0.01,0.03,1.02
- **Calibration**: Requires advanced calibration routine

## Coordinate Systems

### Sensor Axes (LSM6DSO IMU)
```
     +Y (North)
      |
      |
      +---- +X (West)
     /
    /
   +Z (Up)
```

### Quaternion Convention
- **Convention**: North-West-Up (NWU)
- **Rotation**: Right-hand rule
- **Identity**: [w=1, x=0, y=0, z=0] = no rotation from NWU frame

### Euler Angle Conversion
```python
# From quaternion to Euler angles (radians)
roll  = atan2(2*(w*x + y*z), 1 - 2*(x² + y²))
pitch = asin(2*(w*y - z*x))
yaw   = atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
```

## Calibration Procedures

### Magnetometer Hard-Iron Calibration

1. **Data Collection**
   - Slowly rotate device in all orientations
   - Collect 30-60 seconds of magnetometer data
   - Cover full sphere of orientations

2. **Calculate Offsets**
   ```
   x_offset = (x_max + x_min) / 2
   y_offset = (y_max + y_min) / 2
   z_offset = (z_max + z_min) / 2
   ```

3. **Apply via BLE**
   - Send command 0x20 with calculated offsets (in milligauss)
   - Or add to CONFIG.TXT

4. **Verify**
   - Rotate device again
   - Check that magnetic field magnitude is consistent (~500 mG typical)

### Magnetometer Soft-Iron Calibration

1. **Data Collection**
   - Same as hard-iron: full sphere rotation
   - Need 100+ samples evenly distributed

2. **Calculate Matrix**
   - Use ellipsoid fitting algorithm
   - Requires numerical optimization (Python script recommended)

3. **Apply**
   - Send via BLE command 0x21 (matrix × 1000000)
   - Or add to CONFIG.TXT

## Testing & Validation

### Quaternion Sanity Checks

1. **Magnitude**
   ```
   magnitude = sqrt(w² + x² + y² + z²)
   Should be ≈ 1.0 (or 10000 for scaled values)
   ```

2. **Stability**
   - Device stationary → quaternion should be constant (±0.001)
   - Drift should be < 1°/minute

3. **Responsiveness**
   - Quick rotation → quaternion should update within 1-2 samples
   - No lag > 50ms

### Heading Accuracy Test

1. **Setup**
   - Place device on flat surface
   - Align with known compass direction

2. **Measure**
   ```python
   yaw = atan2(2*(q_w*q_z + q_x*q_y), 1 - 2*(q_y² + q_z²))
   heading_degrees = yaw * 180 / pi
   ```

3. **Acceptable**
   - Error < 5° with good magnetometer calibration
   - Error < 2° with excellent calibration

### Gyroscope Range Test

1. **Check Configuration**
   - gyro_fs setting (0=250, 1=500, 2=1000, 3=2000 dps)
   - Fusion uses this for saturation detection

2. **Test**
   - Rotate device slowly → should work with any setting
   - Rotate device quickly (>250 dps) → requires higher range
   - If quaternion freezes → increase gyro_fs

## Performance Metrics

### CPU Usage
- **Fusion update**: ~2-5% CPU @ 416 Hz
- **Depends on**: Compiler optimization, CPU speed
- **Monitor**: Check if IMU samples are missed

### Memory Usage
- **Fusion state**: ~200 bytes
- **Config storage**: ~150 bytes
- **Total added**: <1 KB

### Update Rates
- **Fusion algorithm**: 416 Hz (every IMU sample)
- **BLE transmission**: Configurable (typically 50 Hz)
- **CSV logging**: 416 Hz (every sample)

## Troubleshooting

### Quaternion Not Updating
- Check `Enable_Fusion = 1` in CONFIG.TXT
- Verify Fusion_Init() called at startup
- Check that gyro_fs setting matches IMU configuration

### Heading Drifts
- **Cause**: Poor magnetometer calibration
- **Fix**: Run calibration procedure
- **Temporary**: Increase Fusion_Gain slightly

### Quaternion Jumps/Discontinuities
- **Cause**: Accelerometer rejection during movement
- **Fix**: Increase Fusion_Accel_Reject threshold
- **Note**: Some drift is expected during high-g maneuvers

### BLE Packet Parse Errors
- **Cause**: Mask bit not checked correctly
- **Fix**: Always parse based on mask bits, not fixed offsets
- **Verify**: Packet length matches enabled fields

### Magnetometer Rejected Often
- **Cause**: Magnetic interference or poor calibration
- **Fix**: 
  1. Move away from magnetic sources
  2. Recalibrate magnetometer
  3. Increase Fusion_Mag_Reject threshold

## API Reference

### C Functions (fusion_integration.c)

```c
// Initialize Fusion AHRS (call once at startup)
void FS_Fusion_Init(void);

// Update with IMU data (call at 416 Hz)
void FS_Fusion_UpdateIMU(uint32_t time_ms,
                         int32_t wx, int32_t wy, int32_t wz,
                         int32_t ax, int32_t ay, int32_t az);

// Update with magnetometer data (call when available, ~100 Hz)
void FS_Fusion_UpdateMag(uint32_t time_ms,
                         int32_t mx, int32_t my, int32_t mz);

// Get current quaternion
const FusionQuaternion* FS_Fusion_GetQuaternion(void);

// Set magnetometer hard-iron calibration (milligauss)
void FS_Fusion_SetMagHardIron(float x, float y, float z);

// Set magnetometer soft-iron calibration (3x3 matrix)
void FS_Fusion_SetMagSoftIron(const float matrix[9]);
```

### Data Structures

```c
// Fusion quaternion (from Fusion library)
typedef struct {
    struct {
        float w, x, y, z;
    } element;
} FusionQuaternion;

// AHRS data for CSV logging
typedef struct {
    uint32_t time;    // milliseconds
    int16_t q_w;      // quaternion W × 10000
    int16_t q_x;      // quaternion X × 10000
    int16_t q_y;      // quaternion Y × 10000
    int16_t q_z;      // quaternion Z × 10000
} FS_AHRS_Data_t;

// IMU data (includes quaternion)
typedef struct {
    uint32_t time;
    int32_t wx, wy, wz;      // deg/s × 1000
    int32_t ax, ay, az;      // g × 100000
    int16_t temperature;     // °C × 100
    int16_t q_w, q_x, q_y, q_z;  // quaternion × 10000
} FS_IMU_Data_t;
```

## Version History

### v1.0 (Current)
- ✅ Fusion library integrated (x-io Fusion)
- ✅ All 31+ configuration parameters
- ✅ BLE magnetometer calibration commands
- ✅ CSV logging (SENSOR.CSV)
- ✅ BLE streaming (piggybacked on gyroscope)
- ✅ Auto gyroscope range configuration
- ✅ Configurable recovery timeout

### Future Enhancements (Not Implemented)
- Adaptive gain based on motion detection
- Automatic magnetometer calibration
- Quaternion smoothing for BLE (interpolation between samples)
- Euler angle output option
- Heading lock mode (yaw only from magnetometer)
